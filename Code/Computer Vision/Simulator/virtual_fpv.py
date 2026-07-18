import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests
import math
from PIL import Image
import io

def deg2num(lat_deg, lon_deg, zoom):
    """Convert latitude and longitude to Tile coordinates"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def fetch_elevation(lat, lon, zoom=14):
    """Download elevation data (DEM)"""
    print(f"Downloading topography data for [{lat}, {lon}]...")
    x, y = deg2num(lat, lon, zoom)
    
    # Mapzen Terrain Tiles on Amazon S3
    url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
    
    resp = requests.get(url)
    if resp.status_code != 200:
        raise Exception(f"Failed to download map: HTTP {resp.status_code}")
    
    img = Image.open(io.BytesIO(resp.content))
    img_arr = np.array(img)
    
    # Decode Terrarium format to real altitude in meters
    R = img_arr[:,:,0].astype(np.float64)
    G = img_arr[:,:,1].astype(np.float64)
    B = img_arr[:,:,2].astype(np.float64)
    elevation = (R * 256.0 + G + B / 256.0) - 32768.0
    return elevation

def main():
    # Coordinates: Chão das Feiteiras, Madeira
    lat = 32.723
    lon = -16.892
    
    try:
        elev_data = fetch_elevation(lat, lon, zoom=14)
    except Exception as e:
        print(f"Error: {e}")
        return

    # Reduce mesh resolution
    skip = 3
    Z = elev_data[::skip, ::skip]
    x_len, y_len = Z.shape
    X, Y = np.meshgrid(np.arange(x_len), np.arange(y_len))

    print(f"Map loaded. Min altitude: {np.min(Z):.0f}m, Max: {np.max(Z):.0f}m.")
    print("Generating Virtual FPV 3D...")

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    
    surf = ax.plot_surface(X, Y, Z, cmap='terrain', edgecolor='none', alpha=0.9)
    
    ax.set_title("MANTA - Virtual FPV Ground Station\nLocation: Chão das Feiteiras (Madeira)", fontsize=14, color='white')
    ax.set_zlim(np.min(Z) - 100, np.max(Z) + 500)
    ax.axis('off')
    
    def update_camera(frame):
        # Simulate aircraft attitude (Yaw and Pitch)
        azim = 30 + frame * 0.4 
        pitch = 30 + 15 * math.sin(frame * 0.05)
        
        ax.view_init(elev=pitch, azim=azim)
        return fig,

    print("Starting flight simulation. Close the window to stop.")
    ani = animation.FuncAnimation(fig, update_camera, frames=500, interval=50, blit=False)
    
    plt.show()

if __name__ == "__main__":
    main()
