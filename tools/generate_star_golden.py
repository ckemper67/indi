import requests
import sys
import json
import time

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

if __name__ == "__main__":
    test_jd = 2459019.833333 # 2020-06-19 08:00 UTC
    stars = {"102098": "Deneb", "11767": "Polaris", "30438": "Canopus"}
    
    golden_data = []
    print(f"Generating IMCCE Star Golden Dataset for JD {test_jd}...")
    
    for hid, name in stars.items():
        print(f"  Fetching {name}...")
        res = fetch_imcce_truth(hid, name, test_jd)
        if res: golden_data.append(res)
        time.sleep(0.5)
            
    with open("test/data/star_golden.json", "w") as f:
        json.dump(golden_data, f, indent=2)
        
    print("\nSuccess! Star golden file created at test/data/star_golden.json")
