import json
import numpy as np
import matplotlib.pyplot as plt

def load_translations(json_file):
    """Load translations from a JSON file."""
    with open(json_file, "r") as f:
        data = json.load(f)
    
    # Extract translations and convert to numpy array
    # Extract only the "data" field
    # translations_rel = np.array([np.array(entry["data"]) for entry in data["TotalRotation"]]) # Flatten each matrix
    # # Handle NaNs: Replace with 0 (or use np.isnan to remove them)
    # translations_rel = np.nan_to_num(translations_rel.reshape((-1,4,4)), nan=0.0)  # Replace NaN with 0
    
    translations = np.array([np.array(entry["data"]) for entry in data["TotalTranslation"]]) # Flatten each matrix
    # Handle NaNs: Replace with 0 (or use np.isnan to remove them)
    translations = np.nan_to_num(translations.reshape((-1,4,4)), nan=0.0)  # Replace NaN with 0

    return translations  # Convert list to numpy array

# Load translations
camera_translations = load_translations("transformations_camera.json")
lidar_translations = load_translations("transformations_lidar.json")

# Extract X, Y coordinates
print(lidar_translations[-1])
# transf = np.array([np.eye(4, 4) for _ in range(camera_translations.shape[0]+1)])
# print(transf[0].shape)
# for i in range(camera_translations.shape[0]):
#     transf[i+1] = transf[i] @ np.linalg.inv(camera_translations[i])

# print(transf.shape)
x_cam, y_cam = camera_translations[:, 0, 3], camera_translations[:, 2, 3]
x_lidar, y_lidar = lidar_translations[:, 0, 3], lidar_translations[:, 1, 3]

# Plot both trajectories
plt.figure(figsize=(8, 6))
plt.plot(x_cam, y_cam, marker="o", linestyle="-", color="b", label="Camera Trajectory")
plt.plot(x_lidar, y_lidar, marker="s", linestyle="--", color="r", label="LiDAR Trajectory")

# Labels and legend
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Camera vs LiDAR Translation Trajectory")
plt.legend()
plt.grid()
plt.show()
