import requests
import sys
import json
import time

def fetch_jpl_truth(body_id, name, jd):
    """Fetch truth coordinates from JPL Horizons (Planets)."""
    url = "https://ssd.jpl.nasa.gov/api/horizons.api"
    params = {
        "format": "json",
        "COMMAND": f"'{body_id}'",
        "MAKE_EPHEM": "YES",
        "EPHEM_TYPE": "OBSERVER",
        "CENTER": "'500'",
        "START_TIME": f"JD{jd}",
        "STOP_TIME": f"JD{jd + 0.01}",
        "STEP_SIZE": "1",
        "QUANTITIES": "'2'",
        "ANG_FORMAT": "DEG",
        "CSV_FORMAT": "YES",
        "OBJ_DATA": "NO",
        "CAL_FORMAT": "JD"
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "result" not in data: return None
    lines = data["result"].split("\n")
    in_data = False
    for line in lines:
        if "$$SOE" in line: in_data = True; continue
        if "$$EOE" in line: in_data = False; continue
        if in_data and line.strip():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) > 4:
                return {"name": name, "jd": float(parts[0]), "ra": float(parts[3]), "dec": float(parts[4])}
    return None

def fetch_imcce_truth(hip_id, name, jd):
    """Fetch truth coordinates from IMCCE Miriade (Stars)."""
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

    # Parse sexagesimal RA/Dec: +20:42:08.53306
    def sexa_to_deg(s, is_ra=False):
        parts = s.replace("+", "").split(":")
        d = float(parts[0])
        m = float(parts[1])
        sec = float(parts[2])
        deg = abs(d) + m/60.0 + sec/3600.0
        if d < 0: deg = -deg
        return deg * 15.0 if is_ra else deg

    return {
        "name": name,
        "jd": jd,
        "ra": sexa_to_deg(row["RA"], True),
        "dec": sexa_to_deg(row["DEC"], False)
    }

def fetch_imcce_horizontal(hip_id, name, ra_j2000_h, dec_j2000_deg, jd, iau_code, site_name):
    """Fetch horizontal (az/alt) truth from IMCCE Miriade for a star at a specific IAU site.

    Returns a record with the IMCCE-reported topocenter coordinates (lon/lat/elev) alongside
    the az/alt truth so the test can feed the exact same observer position to
    EquatorialToHorizontal.

    IMCCE uses INPOP19 planetary theory and the same ERFA/SOFA routines for coordinate
    transforms, making this independent from the eraApco00b (IAU 2000B) code path under test.
    The 2000A vs 2000B difference is < 1 mas, well within the 5-arcsecond test tolerance.
    """
    url = "https://ssp.imcce.fr/webservices/miriade/api/ephemcc.php"
    params = {
        "-name": hip_id,
        "-type": "star",
        "-ep": jd,
        "-observer": iau_code,
        "-teph": "2",   # apparent of date
        "-tcoor": "5",  # horizontal coordinates (az/alt)
        "-mime": "json"
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "data" not in data:
        print(f"  WARNING: no data for {name} at {site_name}: {data}", file=sys.stderr)
        return None
    row = data["data"][0]

    # Extract the topocenter coordinates IMCCE actually used so the test can reproduce
    # the exact same observer position.
    topo = data.get("ephemeris", {}).get("reference_frame", {}).get("topocenter", [{}])[0]
    lon = topo.get("longitude", 0.0)
    lat = topo.get("latitude", 0.0)
    elev = topo.get("altitude", 0.0)

    return {
        "label": f"{iau_code}_{name.lower()}",
        "object": name,
        "hip_id": int(hip_id),
        "ra_j2000_h": ra_j2000_h,
        "dec_j2000_deg": dec_j2000_deg,
        "jd": jd,
        "site": site_name,
        "iau_code": iau_code,
        "lon_deg": lon,
        "lat_deg": lat,
        "elev_m": elev,
        "az_deg": float(row["Az"]),
        "alt_deg": float(row["H"]),
        "source": "IMCCE Miriade INPOP19"
    }


if __name__ == "__main__":
    test_jd = 2459019.833333 # 2020-06-19 08:00 UTC

    planets = {"199": "Mercury", "299": "Venus", "499": "Mars", "301": "Moon", "10": "Sun"}
    stars = {"102098": "Deneb", "11767": "Polaris", "30438": "Canopus"}

    golden_data = []
    print(f"Generating Comprehensive Golden Dataset for JD {test_jd}...")

    for bid, name in planets.items():
        print(f"  Fetching Planet {name} (JPL)...")
        res = fetch_jpl_truth(bid, name, test_jd)
        if res: golden_data.append(res)
        time.sleep(0.5)

    for hid, name in stars.items():
        print(f"  Fetching Star {name} (IMCCE)...")
        res = fetch_imcce_truth(hid, name, test_jd)
        if res: golden_data.append(res)
        time.sleep(0.5)

    with open("test/data/jpl_golden.json", "w") as f:
        json.dump(golden_data, f, indent=2)

    print("\nSuccess! Combined golden file created at test/data/jpl_golden.json")
    for entry in golden_data:
        print(f"  {entry['name']:<10}: RA={entry['ra']/15.0:12.8f}h Dec={entry['dec']:12.8f}d")

    # --- Horizontal golden file ---
    # JD 2459580.5 = 2022-Jan-01 00:00:00 UTC
    horiz_jd = 2459580.5

    # Hipparcos J2000 catalog coordinates
    stars_horiz = {
        # HIP ID: (name, ra_j2000_h, dec_j2000_deg)
        "91262":  ("Vega",    18.61564232, 38.78368900),  # HIP 91262
        "11767":  ("Polaris",  2.53030388, 89.26410897),  # HIP 11767
    }

    # IAU observatory codes to query.
    # Three sites spanning ~300 deg of longitude prove observer longitude is used (Case A).
    # Pairing Greenwich (lat 51.5) with Siding Spring (lat -31.3) proves observer latitude
    # is used — Polaris is circumpolar from Greenwich and below the horizon from Siding Spring (Case C).
    sites = {
        # IAU code: site name
        "000": "Greenwich",
        "381": "Mitaka (Tokyo)",
        "568": "Mauna Kea",
        "413": "Siding Spring",
    }

    horiz_data = []
    print(f"\nGenerating Horizontal Golden Dataset for JD {horiz_jd}...")
    for hip_id, (name, ra_h, dec_deg) in stars_horiz.items():
        for iau_code, site_name in sites.items():
            print(f"  Fetching {name} at {site_name} (IMCCE horizontal)...")
            res = fetch_imcce_horizontal(hip_id, name, ra_h, dec_deg, horiz_jd,
                                         iau_code, site_name)
            if res:
                horiz_data.append(res)
            time.sleep(0.5)

    with open("test/data/horizontal_golden.json", "w") as f:
        json.dump(horiz_data, f, indent=2)

    print("\nSuccess! Horizontal golden file created at test/data/horizontal_golden.json")
    for entry in horiz_data:
        print(f"  {entry['object']:<8} {entry['site']:<20}: Az={entry['az_deg']:9.4f}  Alt={entry['alt_deg']:9.4f}")
