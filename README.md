DRAFT BY WIL 

# DalESelfEBot
ROS2 Framework for a Selfie Drawing Robot Implementation using the UR3/UR3e

## system requirments
OS: Ubuntu 22.04
ROS2 Distro: ROS2 Humble Hawksbill
Python 3.10+

####################################################################################

### SETUP

## Step 1: Create and Build the ROS2 Workspace

# Create workspace directories
mkdir -p ~/daleselfebot_ws/src
cd ~/daleselfebot_ws

# Clone the repository into the src folder
cd src
git clone https://github.com/JayDeeBot/DalESelfEBot.git

# Go back to workspace root and install dependencies
cd ~/daleselfebot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
**use this to exclude unfinshed nodes: (rebuild after any nodes are edited)

colcon build --packages-select img_prc_interface img_prc ur3_localisation


**use this when system is complete:
colcon build

## Step 2: Install Python Dependencies

cd ~/daleselfebot_ws/src/DalESelfEBot
pip3 install --user -r requirements.txt





###################################################################

### Maunal launch Nodes

## Image Processing
# in new termainal. Source ROS2 & the Workspace. launch image processing
source /opt/ros/humble/setup.bash
source ~/daleselfebot_ws/install/setup.bash
ros2 run img_prc image_processor


## GUI
# in new termainal. Source ROS2 & the Workspace. launch GUI script
source /opt/ros/humble/setup.bash
source ~/daleselfebot_ws/install/setup.bash
python3 ~/daleselfebot_ws/src/DalESelfEBot/GUI/gui_node.py





