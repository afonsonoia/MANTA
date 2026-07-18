import numpy as np
import requests
import math
import time
from PIL import Image
import io

try:
    import pyvista as pv
except ImportError:
    print("Error: 'pyvista' library is not installed.")
    exit(1)

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def fetch_elevation(lat, lon, zoom=14):
    print(f"[{lat}, {lon}] Downloading topographic map...")
    x, y = deg2num(lat, lon, zoom)
    url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
    
    resp = requests.get(url)
    if resp.status_code != 200:
        raise Exception(f"HTTP Error {resp.status_code}")
    
    img = Image.open(io.BytesIO(resp.content))
    img_arr = np.array(img)
    
    # Decode Terrarium to MSL meters
    R = img_arr[:,:,0].astype(np.float64)
    G = img_arr[:,:,1].astype(np.float64)
    B = img_arr[:,:,2].astype(np.float64)
    elevation = (R * 256.0 + G + B / 256.0) - 32768.0
    return elevation

def main():
    lat = 32.723
    lon = -16.892
    
    elev_data = fetch_elevation(lat, lon, zoom=14)
    Z = elev_data
    x_len, y_len = Z.shape
    
    size_m = 3000 
    
    x = np.linspace(0, size_m, x_len)
    y = np.linspace(0, size_m, y_len)
    x, y = np.meshgrid(x, y)

    print("Initializing PyVista 3D engine...")
    grid = pv.StructuredGrid(x, y, Z)
    grid.point_data['Elevation'] = Z.flatten()

    plotter = pv.Plotter(title="MANTA - Full FPV Simulator")
    plotter.add_mesh(grid, scalars='Elevation', cmap='terrain', show_edges=False, lighting=True)
    plotter.set_background('lightblue')
    
    # Initial plane state
    plane = {
        'x': 800.0,
        'y': 800.0,
        'alt': 1800.0,    
        'speed': 2.0,
        'heading': 45.0   
    }

    print("Simulation Ready. Plane taking off.")

    def update_flight(step):
        rad = math.radians(plane['heading'])
        plane['x'] += plane['speed'] * math.cos(rad)
        plane['y'] += plane['speed'] * math.sin(rad)
        
        ix = int(plane['x'] / size_m * (x_len - 1))
        iy = int(plane['y'] / size_m * (y_len - 1))
        
        margin = 60
        if ix <= margin or ix >= x_len - margin or iy <= margin or iy >= y_len - margin:
            plane['heading'] += 3.0 
            plane['heading'] %= 360
            
        ground_alt = Z[iy, ix]
        target_alt = ground_alt + 120
        
        plane['alt'] += (target_alt - plane['alt']) * 0.03
        
        cam_pos = [plane['x'], plane['y'], plane['alt']]
        
        look_dist = 500
        focal_point = [
            plane['x'] + look_dist * math.cos(rad),
            plane['y'] + look_dist * math.sin(rad),
            plane['alt'] - 20
        ]
        
        plotter.camera.position = cam_pos
        plotter.camera.focal_point = focal_point
        plotter.camera.up = (0, 0, 1)

    print("Starting flight loop...")
    plotter.show(auto_close=False, interactive_update=True)
    
    step = 0
    try:
        while True:
            update_flight(step)
            plotter.update()
            step += 1
            time.sleep(0.03)
    except Exception:
        pass

if __name__ == '__main__':
    main()
