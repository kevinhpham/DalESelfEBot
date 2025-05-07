import cv2
import numpy as np
import json
import yaml
import matplotlib.pyplot as plt

# === CONFIG ===
image_path = "/home/jarred/git/DalESelfEBot/ur3_control/img/Goku.png"  # Use a clean, black-on-white line drawing
yaml_path = "/home/jarred/ros2_ws/src/ur3_localisation/config/params.yaml"
output_json_path = "/home/jarred/ros2_ws/src/ur3_control/config/image_waypoints.json"
use_canny = False  # Toggle to True to use Canny edge detection instead of thresholding

# === Load localisation rectangle for centering/scaling ===
with open(yaml_path, "r") as file:
    params = yaml.safe_load(file)

corner_positions = params["corner_positions"]
center_x = sum(pos["x"] for pos in corner_positions) / 4
center_y = sum(pos["y"] for pos in corner_positions) / 4
center_z = sum(pos["z"] for pos in corner_positions) / 4

width = max(pos["x"] for pos in corner_positions) - min(pos["x"] for pos in corner_positions)
height = max(pos["y"] for pos in corner_positions) - min(pos["y"] for pos in corner_positions)
scale_factor = min(width, height) * 0.9

# === Load image and extract edges ===
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
assert image is not None, f"Image not found: {image_path}"

# Apply Gaussian blur to remove noise
blurred = cv2.GaussianBlur(image, (5, 5), 0)

if use_canny:
    edges = cv2.Canny(blurred, 100, 200)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
else:
    # Use binary threshold (do NOT invert unless image is white-on-black)
    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# === Optional debug visualization ===
# debug_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
# cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 1)
# cv2.imshow("Input Image", image)
# cv2.imshow("Contours", debug_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# === Convert contours to splines ===
img_h, img_w = image.shape
splines = []

for i, contour in enumerate(contours):
    waypoints = []
    for pt in contour:
        x_norm = (pt[0][0] - img_w / 2) / img_w
        y_norm = (pt[0][1] - img_h / 2) / img_h
        x = center_x + scale_factor * x_norm
        y = center_y - scale_factor * y_norm  # Invert Y
        waypoints.append([x, y, center_z])
    splines.append({"id": i + 1, "waypoints": waypoints})

# === Save splines to JSON ===
with open(output_json_path, "w") as outfile:
    json.dump({"splines": splines}, outfile, indent=4)

print(f"Saved {len(splines)} splines to {output_json_path}")

# === Optional matplotlib visualization ===
plt.figure(figsize=(6, 6))
for spline in splines:
    xs = [pt[0] for pt in spline["waypoints"]]
    ys = [pt[1] for pt in spline["waypoints"]]
    plt.plot(xs, ys)
plt.title("Extracted Drawing Splines")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.grid(True)
plt.show()
