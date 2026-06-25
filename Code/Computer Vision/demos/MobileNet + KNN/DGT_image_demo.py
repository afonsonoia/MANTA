import requests
import random
import cv2
import numpy as np


def get_random_dgt_image():
    """Generates a random coordinate and downloads the DGT Orthophotomap via WMS."""

    # Official DGT server URL
    wms_url = "https://cartografia.dgterritorio.gov.pt/wms/ortos2021"

    # 1. Generate random coordinates within Continental Portugal
    lat = random.uniform(37.2, 41.5)
    lon = random.uniform(-8.8, -7.2)

    # 2. Bounding Box adjusted to SIMULATE HIGHER ALTITUDE (~400 meters area)
    delta_lat = 0.0018
    delta_lon = 0.0022

    min_lat, max_lat = lat - delta_lat, lat + delta_lat
    min_lon, max_lon = lon - delta_lon, lon + delta_lon

    # 3. Build the WMS request
    params = {
        "SERVICE": "WMS",
        "VERSION": "1.3.0",
        "REQUEST": "GetMap",
        "LAYERS": "Ortos2021-RGB",
        "CRS": "EPSG:4326",
        "BBOX": f"{min_lat},{min_lon},{max_lat},{max_lon}",
        "WIDTH": "1200",
        "HEIGHT": "1200",
        "FORMAT": "image/png",
        "STYLES": ""
    }

    try:
        response = requests.get(wms_url, params=params, timeout=10)

        # Check if we received a response with an image
        if response.status_code == 200 and 'image' in response.headers.get('content-type', ''):
            image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

            if img is not None:
                # ==========================================
                # THE QUALITY FILTER (Computer Vision)
                # ==========================================
                # Converts to grayscale for faster mathematical calculations
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Counts the percentage of "No Data" pixels (cropped borders)
                white_pixels = np.sum(gray >= 250)
                total_pixels = gray.size
                white_percentage = white_pixels / total_pixels

                # Measures the contrast of the image (Sea/Fog have low contrast)
                std_deviation = np.std(gray)

                # Only accepts the image if it is firm land with useful textures
                if white_percentage < 0.05 and std_deviation > 15.0:
                    return img, lat, lon
                # If the test fails, returns None and the script automatically requests another one

        return None, None, None

    except Exception as e:
        print("DGT connection error:", e)
        return None, None, None


def play_geoguessr():
    print("🚀 Starting DGT Drone Simulator (With Quality Filter)...")

    # Configure OpenCV for a small and tidy window (400x400)
    cv2.namedWindow("Drone GeoGuessr - DGT", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone GeoGuessr - DGT", 400, 400)

    # Mouse control logic
    click_registered = False

    def mouse_callback(event, x, y, flags, param):
        nonlocal click_registered
        if event == cv2.EVENT_LBUTTONDOWN:
            click_registered = True

    cv2.setMouseCallback("Drone GeoGuessr - DGT", mouse_callback)

    while True:
        click_registered = False
        img, lat, lon = None, None, None

        print("\nSearching for valid terrain (filtering oceans and DGT borders)...")

        # Loops requesting images until it passes our Quality Filter
        while img is None:
            img, lat, lon = get_random_dgt_image()

        print(f"🌍 PHOTO FOUND! Exact center: Latitude {lat:.5f}, Longitude {lon:.5f}")
        print("-> Click on the image to simulate the next flight or press 'q' to exit.")

        cv2.imshow("Drone GeoGuessr - DGT", img)

        # Wait for a click on the image or the 'q' key
        while True:
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                print("Closing simulator...")
                cv2.destroyAllWindows()
                return
            if click_registered:
                break


if __name__ == "__main__":
    play_geoguessr()