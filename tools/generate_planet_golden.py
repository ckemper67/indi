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

if __name__ == "__main__":
    test_jd = 2459019.833333 # 2020-06-19 08:00 UTC
    planets = {"199": "Mercury", "299": "Venus", "499": "Mars", "301": "Moon", "10": "Sun"}
    
    golden_data = []
    print(f"Generating JPL Planetary Golden Dataset for JD {test_jd}...")
    
    for bid, name in planets.items():
        print(f"  Fetching {name}...")
        res = fetch_jpl_truth(bid, name, test_jd)
        if res: golden_data.append(res)
        time.sleep(0.5)
            
    with open("test/data/planet_golden.json", "w") as f:
        json.dump(golden_data, f, indent=2)
        
    print("\nSuccess! Planet golden file created at test/data/planet_golden.json")
