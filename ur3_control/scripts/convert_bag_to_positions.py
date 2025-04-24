import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import JointState
from roboticstoolbox import Robot
from scipy.spatial.transform import Rotation as R
import json
import os

# Bag paths
bag_paths = [
    # "/home/jarred/git/DalESelfEBot/ur3_control/rosbags/sim_toolpath/" # For Simulation
    "/home/jarred/git/DalESelfEBot/ur3_control/rosbags/hardware_toolpath" # For Hardware
]

# Load UR3e robot model
robot = Robot.URDF('/home/jarred/git/DalESelfEBot/ur3_localisation/model/ur3e.urdf')

# Expected joint order
expected_order = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

# Output spline waypoints
waypoints = []

# Process each bag
for bag_path in bag_paths:
    print(f"\nReading: {bag_path}")
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_type = get_message(type_map['/joint_states'])

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == '/joint_states':
            msg = deserialize_message(data, msg_type)

            if len(msg.position) != 6:
                continue

            name_to_pos = dict(zip(msg.name, msg.position))
            try:
                joint_positions = [name_to_pos[joint] for joint in expected_order]
            except KeyError:
                continue

            # Forward kinematics
            T = robot.fkine(joint_positions)
            pos = T.t
            waypoints.append([float(pos[0]), float(pos[1]), float(pos[2])])

# Save to JSON
output = {
    "splines": [
        {
            "id": 1,
            "waypoints": waypoints
        }
    ]
}

output_file = "/home/jarred/git/DalESelfEBot/ur3_control/scripts/hardware_ground_truth.json"
with open(output_file, 'w') as f:
    json.dump(output, f, indent=4)

print(f"\n Spline data saved to: {output_file}")
