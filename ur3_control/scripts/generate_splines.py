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

# You can choose a Z height where the circle will be drawn; using average corner height or fixed height
center_z = sum(pos["z"] for pos in corner_positions) / 4

# Generate circle waypoints
radius = 0.05  # 50mm radius
num_points = 36  # More points for a smoother circle

waypoints = []
for i in range(num_points + 1):  # +1 to close the circle
    angle = 2 * math.pi * i / num_points
    x = center_x + radius * math.cos(angle)
    y = center_y + radius * math.sin(angle)
    z = center_z  # same height
    waypoints.append([x, y, z])

# Create the JSON structure
circle_data = {
    "splines": [
        {
            "id": 1,
            "waypoints": waypoints
        }
    ]
}

# Save to json file
with open("/home/jarred/ros2_ws/src/ur3_control/config/circle_waypoints.json", "w") as outfile:
    json.dump(circle_data, outfile, indent=4)

print("circle_waypoints.json created successfully.")

# Plot the circle and the localisation corners for verification
waypoints_x = [point[0] for point in waypoints]
waypoints_y = [point[1] for point in waypoints]

corner_x = [pos["x"] for pos in corner_positions]
corner_y = [pos["y"] for pos in corner_positions]
# Close the rectangle by adding the first corner at the end
corner_x.append(corner_x[0])
corner_y.append(corner_y[0])

plt.figure(figsize=(6, 6))
plt.plot(waypoints_x, waypoints_y, marker='o', label='Circle Waypoints')
plt.scatter(center_x, center_y, color='red', label='Center')
plt.plot(corner_x, corner_y, color='green', linestyle='--', label='Localisation Rectangle')
plt.scatter(corner_x[:-1], corner_y[:-1], color='green')
plt.title('Generated Circle and Localisation Rectangle')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()