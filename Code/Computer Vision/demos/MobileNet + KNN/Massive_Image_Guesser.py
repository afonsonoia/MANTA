import os
import warnings
import sys

# Disables OpenCV pixel limit to allow giant NASA images
os.environ["OPENCV_IO_MAX_IMAGE_PIXELS"] = str(pow(2,40))

# Silence everything! (TensorFlow and Python Warnings)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
warnings.filterwarnings('ignore')

import random
import cv2
import numpy as np
import tensorflow as tf
import pickle
import time
import math
from sklearn.neighbors import NearestNeighbors

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

# =========================================================
# 1. DATABASE AND IMAGE CONFIGURATION
# =========================================================
TURBO_MODE = True
UPDATE_FREQ = 5

DATA_DIR = "data"
DB_FILE = os.path.join(DATA_DIR, "mapa_gigante_db.pkl")
IMG_PATH = "./massive_images/heic1502a.png"

# Exact size accepted by MobileNetV2
PATCH_SIZE = 224
GRID_SIZE = 3  # 3x3 = 9 blocks
CROP_TOTAL_SIZE = PATCH_SIZE * GRID_SIZE  # 672x672

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
                print(f"Map loaded! {len(db['vectors'])} signatures (3x3) known.")
                return db
            except EOFError:
                pass
    print("New database created.")
    return {"vectors": [], "coordinates": []}


def save_database(db):
    with open(DB_FILE, 'wb') as f:
        pickle.dump(db, f)


# =========================================================
# 2. AI AND IMAGE PROCESSING
# =========================================================
print(f"Loading MobileNetV2... (Ultra-Light Mode: 224x224)")
base_model = tf.keras.applications.MobileNetV2(
    input_shape=(224, 224, 3), include_top=False, weights='imagenet'
)
inputs = tf.keras.Input(shape=(224, 224, 3))
x = base_model(inputs, training=False)
outputs = tf.keras.layers.GlobalAveragePooling2D()(x)
extractor = tf.keras.Model(inputs, outputs)
print("Brain optimized and ready to fly.")


def calculate_pixel_error(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def augment_patch(patch):
    alpha = random.uniform(0.85, 1.15)
    beta = random.randint(-15, 15)
    return cv2.convertScaleAbs(patch, alpha=alpha, beta=beta)


def load_giant_image():
    if not os.path.exists(IMG_PATH):
        print(f"\nCRITICAL ERROR: The file '{IMG_PATH}' was not found.")
        sys.exit(1)

    img = cv2.imread(IMG_PATH)
    if img is None:
        print(f"\nCRITICAL ERROR: The file '{IMG_PATH}' exists, but OpenCV could not read it.")
        sys.exit(1)

    print(f"Giant Image loaded successfully! Resolution: {img.shape[1]}x{img.shape[0]}")
    return img


# =========================================================
# 3. GRAPHICS AND DOWNSAMPLING SYSTEM
# =========================================================
def downsample_data(data, max_points=1000):
    """Reduces the number of points to draw to avoid extreme slowdown in Matplotlib."""
    if len(data) <= max_points:
        return range(len(data)), data
    indices = np.linspace(0, len(data) - 1, max_points, dtype=int)
    return indices, np.array(data)[indices]


def generate_turbo_charts(error_history):
    """Generates two charts side-by-side: Raw Error and Moving Average (100)."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # Chart 1: Raw Error
    x_raw, y_raw = downsample_data(error_history)
    if len(y_raw) > 0:
        ax1.plot(x_raw, y_raw, color='red', linewidth=1, alpha=0.6)
    ax1.set_title("Raw Error Over Time")
    ax1.set_ylabel("Error (Pixels)")
    ax1.set_xlabel("Iteration")
    ax1.grid(True)

    # Chart 2: Exact Moving Average of 100
    window = 100
    if len(error_history) >= window:
        averages = np.convolve(error_history, np.ones(window) / window, mode='valid')
        x_ma_full = range(window - 1, len(error_history))

        # Also downsample the average to maintain speed
        x_ma_idx, y_ma = downsample_data(averages)
        x_ma = np.array(x_ma_full)[x_ma_idx]

        ax2.plot(x_ma, y_ma, color='blue', linewidth=2)
    else:
        ax2.text(0.5, 0.5, f"Waiting for 100 iterations...\n({len(error_history)}/100)",
                 horizontalalignment='center', verticalalignment='center',
                 transform=ax2.transAxes, fontsize=12, color='gray')

    ax2.set_title("Moving Average (100 Iterations)")
    ax2.set_ylabel("Mean Error (Pixels)")
    ax2.set_xlabel("Iteration")
    ax2.grid(True)

    fig.tight_layout()
    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    img_chart = np.array(canvas.buffer_rgba())
    plt.close(fig)

    return cv2.cvtColor(img_chart, cv2.COLOR_RGBA2BGR)


def generate_normal_chart_image(error_history):
    """Original chart for when Turbo Mode is off."""
    fig, ax = plt.subplots(figsize=(6, 6))
    if len(error_history) > 0:
        x_raw, y_raw = downsample_data(error_history)
        ax.plot(x_raw, y_raw, color='red', linewidth=2, label='Current Error (Pixels)')

        if len(error_history) >= 10:
            window = min(100, len(error_history))
            averages = np.convolve(error_history, np.ones(window) / window, mode='valid')
            x_medias = range(window - 1, len(error_history))
            x_ma_idx, y_ma = downsample_data(averages)
            x_ma = np.array(x_medias)[x_ma_idx]
            ax.plot(x_ma, y_ma, color='blue', linewidth=2, linestyle='--', label=f'Moving Average ({window})')
        ax.legend()
    else:
        ax.text(0.5, 0.5, "Waiting for data...", horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize=12,
                color='gray')

    ax.set_title("Evolution of Localization Error (Pixels)")
    ax.set_ylabel("Error (Pixels)")
    ax.set_xlabel("Iteration")
    ax.grid(True)
    fig.tight_layout()

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    img_chart = np.array(canvas.buffer_rgba())
    plt.close(fig)

    return cv2.cvtColor(img_chart, cv2.COLOR_RGBA2BGR)


# =========================================================
# 4. THE MAIN LOOP (Offline Random Cropping)
# =========================================================
def play_geoguessr_local():
    img_giant = load_giant_image()
    h_img, w_img, _ = img_giant.shape

    if h_img < CROP_TOTAL_SIZE or w_img < CROP_TOTAL_SIZE:
        print(f"\n❌ CRITICAL ERROR: Your image ({w_img}x{h_img}) is too small!")
        sys.exit(1)

    db = load_database()
    knn = NearestNeighbors(n_neighbors=1, metric='cosine')

    error_history = []
    iteration_counter = 0

    cv2.namedWindow("Dashboard SLAM Local", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dashboard SLAM Local", 1200, 600)
    cv2.setWindowProperty("Dashboard SLAM Local", cv2.WND_PROP_TOPMOST, 1)
    cv2.waitKey(1)

    print(f"\nExploratory Mission Started! [TURBO_MODE = {TURBO_MODE}] (Press 'Q' on the window to exit)")

    while True:
        iteration_counter += 1
        start_x = random.randint(0, w_img - CROP_TOTAL_SIZE)
        start_y = random.randint(0, h_img - CROP_TOTAL_SIZE)

        center_x = start_x + (CROP_TOTAL_SIZE // 2)
        center_y = start_y + (CROP_TOTAL_SIZE // 2)

        crop_master = img_giant[start_y:start_y + CROP_TOTAL_SIZE, start_x:start_x + CROP_TOTAL_SIZE]

        if not TURBO_MODE:
            img_display = crop_master.copy()

        # Crop into 9 pieces (3x3)
        grid_imgs = []
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                y1, y2 = row * PATCH_SIZE, (row + 1) * PATCH_SIZE
                x1, x2 = col * PATCH_SIZE, (col + 1) * PATCH_SIZE

                slice_img = crop_master[y1:y2, x1:x2]
                slice_aug = augment_patch(slice_img)
                slice_rgb = cv2.cvtColor(slice_aug, cv2.COLOR_BGR2RGB)
                grid_imgs.append(slice_rgb)

                if not TURBO_MODE:
                    cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if not TURBO_MODE:
            cv2.rectangle(img_display, (PATCH_SIZE, PATCH_SIZE), (PATCH_SIZE * 2, PATCH_SIZE * 2), (0, 0, 255), 3)

        # Neural Network and K-NN
        batch_norm = (np.array(grid_imgs, dtype=np.float32) / 127.5) - 1.0
        vectors_9 = extractor.predict(batch_norm, verbose=0)
        mega_signature_vector = vectors_9.flatten()

        if len(db["vectors"]) > 0:
            knn.fit(db["vectors"])
            _, indices = knn.kneighbors([mega_signature_vector])
            winning_index = indices[0][0]
            x_predicted, y_predicted = db["coordinates"][winning_index]

            error_pixels = calculate_pixel_error(center_x, center_y, x_predicted, y_predicted)

            # Minimalist print in turbo mode to not clutter the console
            if TURBO_MODE and iteration_counter % UPDATE_FREQ == 0:
                print(f"[Iteration {iteration_counter}] ERROR: {error_pixels:.1f} pixels")
            elif not TURBO_MODE:
                print(f"\nReal: (X:{center_x}, Y:{center_y}) | Predicted: (X:{x_predicted}, Y:{y_predicted})")
                print(f"ERROR: {error_pixels:.1f} pixels")

            error_history.append(error_pixels)

            if error_pixels > 50.0:
                db["vectors"].append(mega_signature_vector)
                db["coordinates"].append((center_x, center_y))
                save_database(db)
        else:
            print("\nFirst crop extracted. Starting the map...")
            db["vectors"].append(mega_signature_vector)
            db["coordinates"].append((center_x, center_y))
            save_database(db)
            error_history.append(0.0)  # Placeholder for the first iteration

        # ==============================================================
        # UI & ADAPTIVE RENDERING
        # ==============================================================
        if TURBO_MODE:
            # In Turbo Mode: Only draws the 2 charts every X iterations
            if iteration_counter % UPDATE_FREQ == 0 or iteration_counter == 1:
                dashboard = generate_turbo_charts(error_history)
                cv2.imshow("Dashboard SLAM Local", dashboard)

            # Waits 1ms purely to catch key presses and not lock the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Closing simulator...")
                break
        else:
            # In Normal Mode: Updates every frame, shows image + chart, with observation delay
            img_chart = generate_normal_chart_image(error_history)
            img_cam_resized = cv2.resize(img_display, (600, 600), interpolation=cv2.INTER_AREA)
            img_chart_resized = cv2.resize(img_chart, (600, 600), interpolation=cv2.INTER_AREA)

            dashboard = cv2.hconcat([img_cam_resized, img_chart_resized])
            cv2.imshow("Dashboard SLAM Local", dashboard)

            start_time = time.time()
            exit_flag = False
            while time.time() - start_time < 0.5:
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    exit_flag = True
                    break
            if exit_flag:
                print("Closing simulator...")
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    play_geoguessr_local()