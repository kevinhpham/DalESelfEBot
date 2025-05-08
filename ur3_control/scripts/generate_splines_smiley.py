import yaml
import json
import math
import matplotlib.pyplot as plt

# Load localisation data from params.yaml
with open("/home/jarred/ros2_ws/src/ur3_localisation/config/params.yaml", "r") as file:
    params = yaml.safe_load(file)

corner_positions = params["corner_positions"]

# Calculate center position
center_x = sum(pos["x"] for pos in corner_positions) / 4
center_y = sum(pos["y"] for pos in corner_positions) / 4
center_z = sum(pos["z"] for pos in corner_positions) / 4

splines = []

# 1. Face Circle
radius = 0.05
num_points = 36
face_circle = []
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = center_x + radius * math.cos(angle)
    y = center_y + radius * math.sin(angle)
    face_circle.append([x, y, center_z])
splines.append({"id": 1, "waypoints": face_circle})

# 2. Left Eye
eye_radius = 0.008
eye_offset_x = 0.02
eye_offset_y = 0.015
left_eye = []
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = center_x - eye_offset_x + eye_radius * math.cos(angle)
    y = center_y + eye_offset_y + eye_radius * math.sin(angle)
    left_eye.append([x, y, center_z])
splines.append({"id": 2, "waypoints": left_eye})

# 3. Right Eye
right_eye = []
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = center_x + eye_offset_x + eye_radius * math.cos(angle)
    y = center_y + eye_offset_y + eye_radius * math.sin(angle)
    right_eye.append([x, y, center_z])
splines.append({"id": 3, "waypoints": right_eye})

# 4. Smile (arc from 210° to 330°)
smile_radius = 0.025
smile = []
for i in range(13):  # 12 segments for arc (210° to 330°)
    angle = math.radians(210 + 10 * i)
    x = center_x + smile_radius * math.cos(angle)
    y = center_y - 0.015 + smile_radius * math.sin(angle)  # lower than center
    smile.append([x, y, center_z])
splines.append({"id": 4, "waypoints": smile})

# 5. Nose (small circle below center)
nose_radius = 0.005  # Smaller than eyes
nose_offset_y = -0.005  # Slightly below center
nose = []
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = center_x + nose_radius * math.cos(angle)
    y = center_y + nose_offset_y + nose_radius * math.sin(angle)
    nose.append([x, y, center_z])
splines.append({"id": 5, "waypoints": nose})

# Save to JSON
with open("/home/jarred/ros2_ws/src/ur3_control/config/smiley_waypoints.json", "w") as outfile:
    json.dump({"splines": splines}, outfile, indent=4)

print("smiley_waypoints.json created successfully.")

# Plot everything for visual verification
plt.figure(figsize=(6, 6))
colors = ['blue', 'black', 'black', 'orange', 'red']
labels = ['Face', 'Left Eye', 'Right Eye', 'Smile', 'Nose']

for idx, spline in enumerate(splines):
    xs = [pt[0] for pt in spline["waypoints"]]
    ys = [pt[1] for pt in spline["waypoints"]]
    plt.plot(xs, ys, marker='o', label=labels[idx], color=colors[idx])

# Localisation rectangle
corner_x = [pos["x"] for pos in corner_positions] + [corner_positions[0]["x"]]
corner_y = [pos["y"] for pos in corner_positions] + [corner_positions[0]["y"]]
plt.plot(corner_x, corner_y, linestyle='--', color='green', label='Localisation Rectangle')
plt.scatter(center_x, center_y, color='red', label='Center')

plt.title("Smiley Face with Nose and Localisation Rectangle")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
