import subprocess

# Path where you want to save the rosbag
output_folder = "/home/jarred/git/DalESelfEBot/ur3_control/rosbags/sim_toolpath"

print(f"Recording /joint_states to: {output_folder}")
print("Press Ctrl+C to stop...")

try:
    subprocess.run([
        "ros2", "bag", "record",
        "/joint_states",
        "-o", output_folder
    ])
except KeyboardInterrupt:
    print("\nRecording stopped.")
