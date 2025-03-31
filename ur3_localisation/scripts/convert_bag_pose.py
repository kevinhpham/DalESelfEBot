# ros2 bag record /joint_states -o /home/jarred/git/DalESelfEBot/ur3_localisation/rosbags/sim_pos_1

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import JointState
from roboticstoolbox import Robot
from scipy.spatial.transform import Rotation as R
import yaml
import os

# Your 4 ROS bag paths (update as needed)
bag_paths = [
    "/home/jarred/git/DalESelfEBot/ur3_localisation/rosbags/hardware_pos1",
    "/home/jarred/git/DalESelfEBot/ur3_localisation/rosbags/hardware_pos2",
    "//home/jarred/git/DalESelfEBot/ur3_localisation/rosbags/hardware_pos3",
    "/home/jarred/git/DalESelfEBot/ur3_localisation/rosbags/hardware_pos4"
]

# Path to your UR3e URDF
robot = Robot.URDF('/home/jarred/git/DalESelfEBot/ur3_localisation/model/ur3e.urdf')

# Joint order expected by the URDF model
expected_order = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

corner_positions = []

for bag_path in bag_paths:
    print(f"\nProcessing bag: {bag_path}")
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_type = get_message(type_map['/joint_states'])

    pose_found = False

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
                print("Joint mismatch, skipping message.")
                continue

            T = robot.fkine(joint_positions)
            pos = T.t
            rot = R.from_matrix(T.R)
            quat = rot.as_quat()  # [x, y, z, w]
            quat = [quat[3], quat[0], quat[1], quat[2]]  # [w, x, y, z]

            pose = {
                'x': float(pos[0]),
                'y': float(pos[1]),
                'z': float(pos[2]),
                'qx': float(quat[1]),
                'qy': float(quat[2]),
                'qz': float(quat[3]),
                'qw': float(quat[0]),
            }

            corner_positions.append(pose)
            pose_found = True
            break  # Only grab first valid message

    if not pose_found:
        print(f"No valid joint_states found in bag: {bag_path}")

# Output to YAML
output = {'corner_positions': corner_positions}
output_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/scripts/hardware_ground_truth.yaml"

with open(output_file, 'w') as f:
    yaml.dump(output, f)

print(f"\n Done! FK poses written to {output_file}")
