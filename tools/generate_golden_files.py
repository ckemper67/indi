"""
Generate golden reference data for libastro coordinate tests.

Produces two files:

  test/data/horizontal_golden.json
    Apparent horizontal coordinates (az/alt) of Vega and Polaris at four IAU
    observatory sites, queried from IMCCE Miriade (INPOP19).  Truth source for
    HorizontalAccuracy_vs_IMCCE in test/core/test_libastro.cpp.

  test/data/jpl_golden.json
    Geocentric apparent equatorial coordinates (RA/Dec) of Mars, Jupiter and
    Saturn at a single epoch, queried from JPL Horizons (DE440).  Truth source
    for PlanetAccuracy in test/core/test_libastro_planets.cpp.

Usage:
    python3 tools/generate_golden_files.py

Requires: requests  (pip install requests)
"""

import requests
import sys
import json
import time


# ---------------------------------------------------------------------------
# JPL Horizons
# ---------------------------------------------------------------------------

def fetch_jpl_planet(body_id, name, jd):
    """Fetch geocentric apparent RA/Dec from JPL Horizons for a solar system body."""
    url = "https://ssd.jpl.nasa.gov/api/horizons.api"
    params = {
        "format":     "json",
        "COMMAND":    f"'{body_id}'",
        "MAKE_EPHEM": "YES",
        "EPHEM_TYPE": "OBSERVER",
        "CENTER":     "'500'",          # geocenter
        "START_TIME": f"JD{jd}",
        "STOP_TIME":  f"JD{jd + 0.01}",
        "STEP_SIZE":  "1",
        "QUANTITIES": "'1'",            # astrometric RA/Dec (J2000 ICRS)
        "ANG_FORMAT": "DEG",
        "CSV_FORMAT": "YES",
        "OBJ_DATA":   "NO",
        "CAL_FORMAT": "JD"
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "result" not in data:
        print(f"  WARNING: no result for {name}: {data}", file=sys.stderr)
        return None
    lines = data["result"].split("\n")
    in_data = False
    for line in lines:
        if "$$SOE" in line: in_data = True; continue
        if "$$EOE" in line: break
        if in_data and line.strip():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) > 4:
                return {
                    "name":    name,
                    "jd":      float(parts[0]),
                    "ra_deg":  float(parts[3]),
                    "dec_deg": float(parts[4]),
                    "source":  "JPL Horizons DE440 astrometric J2000"
                }
    return None


# ---------------------------------------------------------------------------
# IMCCE Miriade
# ---------------------------------------------------------------------------

def fetch_imcce_horizontal(hip_id, name, ra_j2000_h, dec_j2000_deg, jd, iau_code, site_name):
    """Fetch apparent horizontal (az/alt) coordinates from IMCCE Miriade for a star.

    Returns a record with the IMCCE-reported topocenter coordinates (lon/lat/elev)
    alongside the az/alt truth so the test can feed the exact same observer position
    to EquatorialToHorizontal.
    """
    url = "https://ssp.imcce.fr/webservices/miriade/api/ephemcc.php"
    params = {
        "-name":     hip_id,
        "-type":     "star",
        "-ep":       jd,
        "-observer": iau_code,
        "-teph":     "2",   # apparent of date
        "-tcoor":    "5",   # horizontal coordinates (az/alt)
        "-mime":     "json"
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
    lon  = topo.get("longitude", 0.0)
    lat  = topo.get("latitude",  0.0)
    elev = topo.get("altitude",  0.0)

    return {
        "label":        f"{iau_code}_{name.lower()}",
        "object":       name,
        "hip_id":       int(hip_id),
        "ra_j2000_h":   ra_j2000_h,
        "dec_j2000_deg": dec_j2000_deg,
        "jd":           jd,
        "site":         site_name,
        "iau_code":     iau_code,
        "lon_deg":      lon,
        "lat_deg":      lat,
        "elev_m":       elev,
        "az_deg":       float(row["Az"]),
        "alt_deg":      float(row["H"]),
        "source":       "IMCCE Miriade INPOP19"
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    # --- Planet golden file (JPL Horizons DE440) ---
    # JD 2459019.833333 = 2020-06-19 08:00 UTC
    planet_jd = 2459019.833333

    planets = {
        "199": "Mercury",
        "299": "Venus",
        "10":  "Sun",
        "499": "Mars",
        "599": "Jupiter",
        "699": "Saturn",
        "799": "Uranus",
        "899": "Neptune",
        "999": "Pluto",
        "301": "Moon",
    }

    planet_data = []
    print(f"Generating planet golden dataset for JD {planet_jd}...")
    for body_id, name in planets.items():
        print(f"  Fetching {name} (JPL Horizons)...")
        res = fetch_jpl_planet(body_id, name, planet_jd)
        if res:
            planet_data.append(res)
        time.sleep(0.5)

    # All bodies also at J2000.0, providing a second epoch to catch epoch-dependent bugs
    # and to document theory accuracy at the reference epoch.
    j2000_jd = 2451545.0
    print(f"\nGenerating planet golden dataset for JD {j2000_jd} (J2000.0)...")
    for body_id, name in planets.items():
        print(f"  Fetching {name} at J2000.0 (JPL Horizons)...")
        res = fetch_jpl_planet(body_id, name, j2000_jd)
        if res:
            planet_data.append(res)
        time.sleep(0.5)

    with open("test/data/jpl_golden.json", "w") as f:
        json.dump(planet_data, f, indent=2)

    print("\nSuccess! test/data/jpl_golden.json written.")
    for entry in planet_data:
        print(f"  {entry['name']:<10}: RA={entry['ra_deg']/15.0:12.8f}h  Dec={entry['dec_deg']:12.8f}d")

    # --- Horizontal golden file (IMCCE Miriade INPOP19) ---
    # JD 2459580.5 = 2022-Jan-01 00:00:00 UTC
    horiz_jd = 2459580.5

    # Hipparcos J2000 catalog coordinates
    stars_horiz = {
        "91262": ("Vega",    18.61564232, 38.78368900),
        "11767": ("Polaris",  2.53030388, 89.26410897),
    }

    # IAU observatory codes to query.
    # Three sites spanning ~300 deg of longitude prove observer longitude is used.
    # Pairing Greenwich (lat 51.5 N) with Siding Spring (lat 31.3 S) proves observer
    # latitude is used — Polaris is circumpolar from Greenwich and below the horizon
    # from Siding Spring.
    sites = {
        "000": "Greenwich",
        "381": "Mitaka (Tokyo)",
        "568": "Mauna Kea",
        "413": "Siding Spring",
    }

    horiz_data = []
    print(f"\nGenerating horizontal golden dataset for JD {horiz_jd}...")
    for hip_id, (name, ra_h, dec_deg) in stars_horiz.items():
        for iau_code, site_name in sites.items():
            print(f"  Fetching {name} at {site_name}...")
            res = fetch_imcce_horizontal(hip_id, name, ra_h, dec_deg, horiz_jd,
                                         iau_code, site_name)
            if res:
                horiz_data.append(res)
            time.sleep(0.5)

    with open("test/data/horizontal_golden.json", "w") as f:
        json.dump(horiz_data, f, indent=2)

    print("\nSuccess! test/data/horizontal_golden.json written.")
    for entry in horiz_data:
        print(f"  {entry['object']:<8} {entry['site']:<20}: Az={entry['az_deg']:9.4f}  Alt={entry['alt_deg']:9.4f}")
