import requests
import sys
import json
import time

def fetch_jpl_truth(body_id, name, jd):
    """Fetch geocentric apparent coordinates from JPL Horizons (CENTER=500)."""
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


def fetch_jpl_topo_truth(body_id, name, np, jd, site):
    """Fetch topocentric apparent coordinates from JPL Horizons for a surface observer.

    Uses CENTER='coord@399' with COORD_TYPE=GEODETIC and SITE_COORD='lon,lat,elev_km'
    (east longitude in degrees, geodetic latitude in degrees, elevation in km).
    QUANTITIES=2 returns airless apparent RA/Dec of-date with light-time, aberration,
    precession, and nutation — directly comparable to GetPlanetTopocentric output.
    """
    lon_deg  = site["lon"]
    lat_deg  = site["lat"]
    elev_km  = site["elev_km"]
    url = "https://ssd.jpl.nasa.gov/api/horizons.api"
    params = {
        "format":       "json",
        "COMMAND":      f"'{body_id}'",
        "MAKE_EPHEM":   "YES",
        "EPHEM_TYPE":   "OBSERVER",
        "CENTER":       "'coord@399'",
        "COORD_TYPE":   "'GEODETIC'",
        "SITE_COORD":   f"'{lon_deg},{lat_deg},{elev_km}'",
        "START_TIME":   f"JD{jd}",
        "STOP_TIME":    f"JD{jd + 0.01}",
        "STEP_SIZE":    "1",
        "QUANTITIES":   "'2'",
        "ANG_FORMAT":   "DEG",
        "CSV_FORMAT":   "YES",
        "OBJ_DATA":     "NO",
        "CAL_FORMAT":   "JD",
    }
    response = requests.get(url, params=params)
    data = response.json()
    if "result" not in data:
        print(f"  WARNING: no result for {name} at {site['site']}: {data}", file=sys.stderr)
        return None
    lines = data["result"].split("\n")
    in_data = False
    for line in lines:
        if "$$SOE" in line: in_data = True; continue
        if "$$EOE" in line: in_data = False; continue
        if in_data and line.strip():
            parts = [p.strip() for p in line.split(",")]
            if len(parts) > 4:
                return {
                    "planet":  name,
                    "np":      np,
                    "jd":      float(parts[0]),
                    "site":    site["site"],
                    "lon_deg": lon_deg,
                    "lat_deg": lat_deg,
                    "elev_m":  elev_km * 1000.0,
                    "ra_deg":  float(parts[3]),
                    "dec_deg": float(parts[4]),
                    "source":  "JPL Horizons DE440 topocentric apparent",
                }
    return None


# Observer sites used for topocentric truth.  East longitude in degrees,
# geodetic latitude in degrees, elevation in km (JPL Horizons convention).
# The same sites are used in horizontal_golden.json so observer coordinates
# are consistent across test files.
OBSERVER_SITES = [
    {"site": "Greenwich",     "lon":   0.0000, "lat":  51.4769, "elev_km": 0.045},
    {"site": "Mauna Kea",     "lon": 204.5314, "lat":  19.8262, "elev_km": 4.204},
    {"site": "Siding Spring", "lon": 149.0658, "lat": -31.2727, "elev_km": 1.149},
]

# Bodies for which topocentric parallax is significant enough to validate.
# Moon (~57' max parallax) and Mars (~24" at opposition) are the key cases.
# Outer planets have parallax < 2" and are covered by the multi-epoch geocentric test.
TOPO_PLANETS = [
    ("301", "Moon", 3),
    ("499", "Mars", 4),
]


if __name__ == "__main__":
    test_jd = 2459019.833333  # 2020-06-19 08:00 UTC
    planets = {"199": "Mercury", "299": "Venus", "499": "Mars", "301": "Moon", "10": "Sun"}

    # --- Geocentric golden ---
    golden_data = []
    print(f"Generating JPL Planetary Golden Dataset for JD {test_jd}...")

    for bid, name in planets.items():
        print(f"  Fetching {name} (geocentric)...")
        res = fetch_jpl_truth(bid, name, test_jd)
        if res: golden_data.append(res)
        time.sleep(0.5)

    with open("test/data/planet_golden.json", "w") as f:
        json.dump(golden_data, f, indent=2)

    print("Success! Planet golden file created at test/data/planet_golden.json")

    # --- Topocentric golden ---
    topo_data = []
    print(f"\nGenerating JPL Topocentric Planet Golden Dataset for JD {test_jd}...")

    for bid, name, np in TOPO_PLANETS:
        for site in OBSERVER_SITES:
            print(f"  Fetching {name} at {site['site']} (topocentric)...")
            res = fetch_jpl_topo_truth(bid, name, np, test_jd, site)
            if res:
                topo_data.append(res)
                print(f"    RA={res['ra_deg']:.5f}°  Dec={res['dec_deg']:.5f}°")
            time.sleep(0.5)

    with open("test/data/planet_topo_golden.json", "w") as f:
        json.dump(topo_data, f, indent=2)

    print("\nSuccess! Topocentric planet golden file created at test/data/planet_topo_golden.json")
