import requests
import json
import math
import time
import erfa

def fetch_imcce_truth(hip_id, name, jd):
    """Fetch apparent coordinates from IMCCE Miriade for a Hipparcos star."""
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

    def sexa_to_deg(s, is_ra=False):
        parts = s.replace("+", "").split(":")
        d = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        deg = abs(d) + m/60.0 + sec/3600.0
        if d < 0: deg = -deg
        return deg * 15.0 if is_ra else deg

    return sexa_to_deg(row["RA"], True), sexa_to_deg(row["DEC"], False)


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
    # ICRS J2000.0 positions from Hipparcos/CDS (already propagated to J2000.0 by the catalog).
    stars = {
        "102098": ("Deneb",      310.35798,  45.28034,    1.99,    1.95),
        "11767":  ("Polaris",     37.94625,  89.26411,   44.22,  -11.74),
        "30438":  ("Canopus",     95.98785, -52.69567,   19.93,   23.24),
        "32349":  ("Sirius",     101.28715, -16.71612, -545.91,-1223.14),
        "91262":  ("Vega",       279.23473,  38.78369,  200.94,  286.23),
        "24436":  ("Rigel",       78.63447,  -8.20164,    1.31,    0.50),
        "27989":  ("Betelgeuse",  88.79292,   7.40706,   24.95,    9.56),
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
