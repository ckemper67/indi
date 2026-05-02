import requests
import json
import math
import time
import erfa

def _sexa_to_deg(s, is_ra=False):
    parts = s.replace("+", "").split(":")
    d = float(parts[0])
    m = float(parts[1])
    sec = float(parts[2])
    deg = abs(d) + m/60.0 + sec/3600.0
    if d < 0: deg = -deg
    return deg * 15.0 if is_ra else deg


def fetch_imcce_truth(hip_id, name, jd):
    """Fetch geocentric apparent coordinates from IMCCE Miriade for a Hipparcos star."""
    url = "https://ssp.imcce.fr/webservices/miriade/api/ephemcc.php"
    params = {
        "-name": hip_id,
        "-type": "star",
        "-ep": jd,
        "-observer": "500",
        "-teph": "2", # Apparent of date
        "-mime": "json"
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "data" not in data: return None
    row = data["data"][0]
    return _sexa_to_deg(row["RA"], True), _sexa_to_deg(row["DEC"], False)


def fetch_imcce_topo_truth(hip_id, name, jd, site):
    """Fetch topocentric apparent coordinates from IMCCE Miriade for a Hipparcos star.

    Uses the IAU observatory code (-observer: iau_code) so IMCCE applies
    geocentric parallax and diurnal aberration for the given ground station.
    Returns a dict or None.
    """
    url = "https://ssp.imcce.fr/webservices/miriade/api/ephemcc.php"
    params = {
        "-name":     hip_id,
        "-type":     "star",
        "-ep":       jd,
        "-observer": site["iau_code"],
        "-teph":     "2",  # Apparent of date
        "-mime":     "json",
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "data" not in data:
        import sys
        print(f"  WARNING: no IMCCE data for {name} at {site['site']}: {data}", file=sys.stderr)
        return None
    row = data["data"][0]
    return {
        "name":     name,
        "jd":       jd,
        "site":     site["site"],
        "iau_code": site["iau_code"],
        "lon_deg":  site["lon"],
        "lat_deg":  site["lat"],
        "elev_m":   site["elev_m"],
        "ra_deg":   _sexa_to_deg(row["RA"], True),
        "dec_deg":  _sexa_to_deg(row["DEC"], False),
        "source":   "IMCCE Miriade topocentric apparent",
    }


# Observer sites for topocentric truth.  IAU codes matched to the same geodetic
# coordinates used in planet_topo_golden.json (generate_planet_golden.py).
TOPO_SITES = [
    {"site": "Greenwich",     "iau_code": "000", "lon":   0.0000, "lat":  51.4769, "elev_m":   45.0},
    {"site": "Mauna Kea",     "iau_code": "568", "lon": 204.5314, "lat":  19.8262, "elev_m": 4204.0},
    {"site": "Siding Spring", "iau_code": "413", "lon": 149.0658, "lat": -31.2727, "elev_m": 1149.0},
]

# Stars for which topocentric parallax is measurable.  Annual parallax for
# Sirius (379 mas) and Vega (130 mas) is significant; diurnal parallax for all
# nearby stars is < 1" and is the main effect being validated here.
TOPO_STARS = [
    ("32349",  "Sirius"),
    ("91262",  "Vega"),
    ("69673",  "Arcturus"),
    ("102098", "Deneb"),
]


def propagate_icrs(ra_deg, dec_deg, mu_alpha_star_masyr, mu_delta_masyr, dt_yr):
    """
    Propagate an ICRS position by dt_yr years using linear proper motion.
    mu_alpha_star is mu_alpha * cos(delta) in mas/yr.
    Returns (ra_deg, dec_deg) at the new epoch.
    """
    dec_rad = math.radians(dec_deg)
    delta_ra_deg  = mu_alpha_star_masyr * dt_yr / math.cos(dec_rad) / 1000.0 / 3600.0
    delta_dec_deg = mu_delta_masyr      * dt_yr                     / 1000.0 / 3600.0
    return ra_deg + delta_ra_deg, dec_deg + delta_dec_deg




def erfa_apparent_with_pm(ra_j2000_deg, dec_j2000_deg,
                           mu_alpha_star_masyr, mu_delta_masyr, jd):
    """
    Compute apparent RA/Dec using eraAtci13 with proper motion applied.
    This matches what a PM-aware pipeline (KStars, full ERFA) would produce
    given ICRS J2000.0 input coordinates.
    Returns (ra_deg, dec_deg) apparent of date.
    """
    rc = math.radians(ra_j2000_deg)
    dc = math.radians(dec_j2000_deg)
    # eraAtci13 expects pr = d(alpha)/dt in rad/yr (NOT mu_alpha*cos(delta))
    pr = math.radians(mu_alpha_star_masyr / math.cos(dc) / 1000.0 / 3600.0)
    pd = math.radians(mu_delta_masyr                     / 1000.0 / 3600.0)

    utc1 = math.floor(jd) + 0.5
    utc2 = jd - utc1

    ri, di, eo = erfa.atci13(rc, dc, pr, pd, 0.0, 0.0, utc1, utc2)
    ra_app  = math.degrees(erfa.anp(ri - eo))  # CIRS -> apparent RA via EO
    dec_app = math.degrees(di)
    return ra_app, dec_app


if __name__ == "__main__":
    test_jd = 2459019.833333  # 2020-06-19 08:00 UTC

    # HIP ID -> (name, ra_j2000_deg, dec_j2000_deg, mu_alpha_star_masyr, mu_delta_masyr)
    # ICRS J2000.0 positions from Hipparcos-2 (van Leeuwen 2007, VizieR I/311).
    # This is the same catalog IMCCE Miriade uses internally.
    # Parallax note: Sirius (379 mas), Vega (130 mas), Arcturus (89 mas) have non-negligible
    # annual parallax. IMCCE includes it; J2000toObserved (plx=0) does not. Residuals for
    # those stars reflect this engine limitation, not catalog or frame rotation errors.
    stars = {
        "102098": ("Deneb",      310.35798,  45.28034,    1.99,    1.95),
        "11767":  ("Polaris",     37.95456,  89.26411,   44.22,  -11.74),  # was 37.94625 (wrong source)
        "30438":  ("Canopus",     95.98796, -52.69566,   19.93,   23.24),  # was 95.98785
        "32349":  ("Sirius",     101.28715, -16.71612, -545.91,-1223.14),
        "91262":  ("Vega",       279.23473,  38.78369,  200.94,  286.23),
        "24436":  ("Rigel",       78.63447,  -8.20164,    1.31,    0.50),
        "27989":  ("Betelgeuse",  88.79294,   7.40706,   24.95,    9.56),  # was 88.79292
        "69673":  ("Arcturus",   213.91530,  19.18241,-1093.45,-1999.40),
    }

    golden_data = []
    print(f"Generating IMCCE Star Golden Dataset for JD {test_jd}...")

    dt_to_obs = (test_jd - 2451545.0) / 365.25  # years from J2000.0 to observation

    for hid, (name, ra_j2000, dec_j2000, mu_a, mu_d) in stars.items():
        print(f"  Fetching {name}...")
        result = fetch_imcce_truth(hid, name, test_jd)
        if result is None:
            print(f"    WARN: no data for {name}")
            continue
        ra_app, dec_app = result

        # PM-propagated input: ICRS position at the observation epoch.
        # Feed this to each engine so PM is cancelled on both sides of the comparison.
        ra_j2obs, dec_j2obs = propagate_icrs(ra_j2000, dec_j2000, mu_a, mu_d, dt_to_obs)

        golden_data.append({
            "name": name,
            "jd": test_jd,
            "ra_j2000": round(ra_j2000, 7),       # ICRS J2000.0 (bare, no PM)
            "dec_j2000": round(dec_j2000, 7),
            "ra_j2obs": round(ra_j2obs, 7),        # ICRS at observation epoch (J2000 + PM×dt)
            "dec_j2obs": round(dec_j2obs, 7),
            "mu_alpha_star_masyr": mu_a,
            "mu_delta_masyr": mu_d,
            "ra": ra_app,                          # IMCCE apparent (includes PM from J1991.25)
            "dec": dec_app,
        })
        time.sleep(0.5)

    with open("test/data/star_golden.json", "w") as f:
        json.dump(golden_data, f, indent=2)

    print("\nSuccess! Star golden file created at test/data/star_golden.json")

    # --- Topocentric golden ---
    topo_data = []
    print(f"\nGenerating IMCCE Topocentric Star Golden Dataset for JD {test_jd}...")

    for hid, name in TOPO_STARS:
        for site in TOPO_SITES:
            print(f"  Fetching {name} at {site['site']} (topocentric)...")
            res = fetch_imcce_topo_truth(hid, name, test_jd, site)
            if res:
                topo_data.append(res)
                print(f"    RA={res['ra_deg']:.5f}  Dec={res['dec_deg']:.5f}")
            time.sleep(0.5)

    with open("test/data/star_topo_golden.json", "w") as f:
        json.dump(topo_data, f, indent=2)

    print("\nSuccess! Topocentric star golden file created at test/data/star_topo_golden.json")
