import os
import warnings

# Silence everything! (TensorFlow and Python Warnings)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore')

import requests
import random
import cv2
import numpy as np
import tensorflow as tf
import pickle
import time
import math
import threading
import queue
from sklearn.neighbors import NearestNeighbors

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

# =========================================================
# 1. DATABASE CONFIGURATION
# =========================================================
DATA_DIR = "data"
DB_FILE = os.path.join(DATA_DIR, "mapa_db.pkl")

if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)


def load_database():
    if os.path.exists(DB_FILE):
        with open(DB_FILE, 'rb') as f:
            try:
                db = pickle.load(f)
                # Backward compatibility check for Portuguese keys
                if 'vetores' in db:
                    db['vectors'] = db.pop('vetores')
                if 'coordenadas' in db:
                    db['coordinates'] = db.pop('coordenadas')
                print(f"Map loaded! {len(db['vectors'])} areas (Mega Signatures 5x5) already known.")
                return db
            except EOFError:
                pass
    print("New database created.")
    return {"vectors": [], "coordinates": []}


def save_database(db):
    with open(DB_FILE, 'wb') as f:
        pickle.dump(db, f)


# =========================================================
# 2. MATHEMATICAL AND AI FUNCTIONS
# =========================================================
def calculate_distance_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


print(f"Loading MobileNetV2... (Ultra-Light Mode: 224x224)")
base_model = tf.keras.applications.MobileNetV2(
    input_shape=(224, 224, 3), include_top=False, weights='imagenet'
)
inputs = tf.keras.Input(shape=(224, 224, 3))
x = base_model(inputs, training=False)
outputs = tf.keras.layers.GlobalAveragePooling2D()(x)
extractor = tf.keras.Model(inputs, outputs)
print("✅ Brain optimized and ready to fly.")


def apply_clahe_filter(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    cl = clahe.apply(l_channel)
    limg = cv2.merge((cl, a_channel, b_channel))
    return cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)


def get_random_dgt_image():
    """Downloads the macro-area (5x5) in a single 1000x1000 image."""
    wms_url = "https://cartografia.dgterritorio.gov.pt/wms/ortos2021"

    lat = random.uniform(37.0, 42.2)
    lon = random.uniform(-9.5, -6.2)

    delta_lat = 0.001
    delta_lon = 0.0014

    macro_delta_lat = delta_lat * 5
    macro_delta_lon = delta_lon * 5

    min_lat, max_lat = lat - macro_delta_lat, lat + macro_delta_lat
    min_lon, max_lon = lon - macro_delta_lon, lon + macro_delta_lon

    params = {
        "SERVICE": "WMS", "VERSION": "1.3.0", "REQUEST": "GetMap",
        "LAYERS": "Ortos2021-RGB", "CRS": "CRS:84",
        "BBOX": f"{min_lon},{min_lat},{max_lon},{max_lat}",
        "WIDTH": "1000", "HEIGHT": "1000",
        "FORMAT": "image/png", "STYLES": ""
    }

    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"
    }

    try:
        response = requests.get(wms_url, params=params, headers=headers, timeout=10)

        if response.status_code == 200 and 'image' in response.headers.get('content-type', ''):
            image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            img = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

            if img is not None:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if np.mean(gray) < 252.0:
                    return img, lat, lon

        return None, None, None
    except Exception:
        return None, None, None


# =========================================================
# 3. THREADING AND GRAPHICS SYSTEM (Producer)
# =========================================================
image_queue = queue.Queue(maxsize=10)


def thread_download_images():
    while True:
        img, lat, lon = get_random_dgt_image()
        if img is not None:
            image_queue.put((img, lat, lon))
        else:
            time.sleep(0.5)


def generate_chart_image(all_averages, num_errors):
    # Perfectly square chart (600x600 equivalent) to stick next to it
    fig, ax = plt.subplots(figsize=(6, 6))

    if num_errors >= 200 and len(all_averages) > 0:
        x_all = list(range(200, 200 + len(all_averages)))
        y_all = all_averages

        if len(x_all) <= 50:
            indices = list(range(len(x_all)))
        else:
            indices = np.linspace(0, len(x_all) - 1, 50, dtype=int)

        x_plot = [x_all[i] for i in indices]
        y_plot = [y_all[i] for i in indices]

        ax.plot(x_plot, y_plot, color='purple', linewidth=2, marker='o', markersize=4, label=f'Average (200 img)')
        ax.legend()
    else:
        ax.text(0.5, 0.5, f"Calibrating long-term...\n({num_errors}/200 images)",
                horizontalalignment='center', verticalalignment='center',
                transform=ax.transAxes, fontsize=11, color='gray')

    ax.set_title("Evolution of Localization Error")
    ax.set_ylabel("Error (km)")
    ax.set_xlabel("No. of Processed Images")
    ax.grid(True)
    fig.tight_layout()

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    img_chart = np.array(canvas.buffer_rgba())
    plt.close(fig)

    return cv2.cvtColor(img_chart, cv2.COLOR_RGBA2BGR)


# =========================================================
# 4. THE MAIN LOOP (Consumer and AI)
# =========================================================
def play_geoguessr():
    db = load_database()
    knn = NearestNeighbors(n_neighbors=1, metric='cosine')

    error_history = []
    moving_averages_200 = []

    # Window now adapted for 1200x600 (600 for image + 600 for chart)
    cv2.namedWindow("Dashboard SLAM", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dashboard SLAM", 1200, 600)

    print("\nStarting preloading Thread (Background Download with Queue of 10)...")
    thread_download = threading.Thread(target=thread_download_images, daemon=True)
    thread_download.start()

    print("Establishing connection to DGT", end="")
    while image_queue.empty():
        print(".", end="", flush=True)
        time.sleep(0.5)

    print("\nPeripheral 5x5 Mission starting at MAXIMUM SPEED... (CTRL+C to stop)")

    while True:
        img_original, lat_real, lon_real = image_queue.get()
        print(f"\nMacro-area (5x5) received! (Remaining images in buffer: {image_queue.qsize()})")

        img_filtered = apply_clahe_filter(img_original)

        # -------------------------------------------------------------
        # THE MATRIX OF 25 SQUARES: Slicing to 224x224
        # -------------------------------------------------------------
        grid_imgs = []
        for row in range(5):
            for col in range(5):
                y1, y2 = row * 200, (row + 1) * 200
                x1, x2 = col * 200, (col + 1) * 200
                slice_img = img_filtered[y1:y2, x1:x2]

                slice_resized = cv2.resize(slice_img, (224, 224))
                slice_rgb = cv2.cvtColor(slice_resized, cv2.COLOR_BGR2RGB)
                grid_imgs.append(slice_rgb)

        batch_norm = (np.array(grid_imgs, dtype=np.float32) / 127.5) - 1.0

        vectors_25 = extractor.predict(batch_norm, verbose=0)
        mega_signature_vector = vectors_25.flatten()

        # -------------------------------------------------------------
        # PREDICTION SYSTEM
        # -------------------------------------------------------------
        if len(db["vectors"]) > 0:
            knn.fit(db["vectors"])
            math_distance, indices = knn.kneighbors([mega_signature_vector])
            winning_index = indices[0][0]
            lat_predicted, lon_predicted = db["coordinates"][winning_index]

            error_km = calculate_distance_km(lat_real, lon_real, lat_predicted, lon_predicted)
            print(f"Real: {lat_real:.5f}, {lon_real:.5f} | Predicted: {lat_predicted:.5f}, {lon_predicted:.5f}")
            print(f"ERROR: {error_km:.2f} km")

            error_history.append(error_km)
            num_errors = len(error_history)

            if num_errors >= 200:
                moving_averages_200.append(np.mean(error_history[-200:]))

            if error_km > 0.2:
                db["vectors"].append(mega_signature_vector)
                db["coordinates"].append((lat_real, lon_real))
                save_database(db)
            else:
                print("Mega-Signature known (Error < 200m).")
        else:
            print("\nFirst macro-area acquired. Starting the map...")
            db["vectors"].append(mega_signature_vector)
            db["coordinates"].append((lat_real, lon_real))
            save_database(db)
            print("Starting point saved.")

        # -------------------------------------------------------------
        # UI / DASHBOARD: CLEAN IMAGE WITH HIGH DENSITY (INTER_AREA)
        # -------------------------------------------------------------
        img_display = img_filtered.copy()  # No yellow lines drawn!

        img_chart = generate_chart_image(moving_averages_200, len(error_history))

        # Shrinking from 1000x1000 to 600x600 using INTER_AREA drastically increases sharpness
        img_cam_resized = cv2.resize(img_display, (600, 600), interpolation=cv2.INTER_AREA)
        img_chart_resized = cv2.resize(img_chart, (600, 600), interpolation=cv2.INTER_AREA)

        dashboard = cv2.hconcat([img_cam_resized, img_chart_resized])
        cv2.imshow("Dashboard SLAM", dashboard)

        cv2.imwrite(os.path.join(DATA_DIR, "error_chart.png"), img_chart)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Closing simulator...")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    play_geoguessr()