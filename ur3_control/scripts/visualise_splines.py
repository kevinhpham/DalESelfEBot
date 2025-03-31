import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("/home/jarred/git/DalESelfEBot/ur3_control/scripts/splines.csv")

# Separate corners and spline points
corners = df[df["type"] == "corner"]
splines = df[df["type"] == "spline"]

# Plot
plt.figure(figsize=(8, 8))

# Plot the localisation rectangle
corner_x = corners["x"].to_numpy()
corner_y = corners["y"].to_numpy()
corner_x = list(corner_x) + [corner_x[0]]
corner_y = list(corner_y) + [corner_y[0]]
plt.plot(corner_x, corner_y, 'ro-', label="Localisation Box")

# Plot each spline separately by ID
for spline_id, group in splines.groupby("id"):
    x_vals = group["x"].to_numpy()
    y_vals = group["y"].to_numpy()
    plt.plot(x_vals, y_vals, '-', linewidth=1.5, label=f"Spline {spline_id}")

plt.title("2D Spline Preview")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
