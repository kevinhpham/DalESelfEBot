#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import String as RosString
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tkinter as tk
from tkinter import messagebox, Toplevel, DISABLED, NORMAL, FLAT
from tkinter import simpledialog # For Set Robot IP
from PIL import Image as PILImage, ImageTk
import threading
import time
import numpy as np
import sys
import subprocess
import os
import traceback
import yaml # For Set Robot IP

# Try to import the action definition
try:
    from img_prc_interface.action import Img
    from action_msgs.msg import GoalStatus
except ImportError:
    print("Error: Could not import Img action or GoalStatus.")
    print("Ensure the img_prc_interface package is built and sourced,")
    print("and that action_msgs is available.")
    Img = None
    GoalStatus = None

# Define the ROS 2 Node for GUI interaction
class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info('GUI Node started')
        self.bridge = CvBridge()

        self.latest_webcam_frame = None
        self.webcam_frame_lock = threading.Lock()
        self.latest_edge_frame = None
        self.edge_frame_lock = threading.Lock()
        self.latest_processed_frame = None
        self.processed_frame_lock = threading.Lock()
        self.processing_active = False
        self.canvas_localisation_process = None
        self.robot_calibration_process = None

        self.latest_error_message = None
        self.error_message_lock = threading.Lock()

        self._send_goal_future = None
        self._get_result_future = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_main_folder = os.path.dirname(script_dir)
        self.params_file_path = os.path.join(project_main_folder, "parameters.yaml")
        self.get_logger().info(f"Parameters YAML file path set to: {self.params_file_path}")

        self.pixel_map_publisher = self.create_publisher(Image, '/pixel_map', 10)
        self.save_pos_publisher = self.create_publisher(Empty, '/save_position', 10)
        self.get_logger().info("Publishers created for /pixel_map and /save_position")

        self.webcam_subscription = self.create_subscription(
            Image, '/webcam_image', self.webcam_callback, 10)
        self.edge_subscription #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import String as RosString
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tkinter as tk
from tkinter import messagebox, Toplevel, DISABLED, NORMAL, FLAT
from tkinter import simpledialog # For Set Robot IP
from PIL import Image as PILImage, ImageTk
import threading
import time
import numpy as np
import sys
import subprocess
import os
import traceback
import yaml # For Set Robot IP

# Try to import the action definition
try:
    from img_prc_interface.action import Img
    from action_msgs.msg import GoalStatus
except ImportError:
    print("Error: Could not import Img action or GoalStatus.")
    print("Ensure the img_prc_interface package is built and sourced,")
    print("and that action_msgs is available.")
    Img = None
    GoalStatus = None

# Define the ROS 2 Node for GUI interaction
class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info('GUI Node started')
        self.bridge = CvBridge()

        self.latest_webcam_frame = None
        self.webcam_frame_lock = threading.Lock()
        self.latest_edge_frame = None
        self.edge_frame_lock = threading.Lock()
        self.latest_processed_frame = None
        self.processed_frame_lock = threading.Lock()
        self.processing_active = False
        self.canvas_localisation_process = None
        self.robot_calibration_process = None

        self.latest_error_message = None
        self.error_message_lock = threading.Lock()

        self._send_goal_future = None
        self._get_result_future = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_main_folder = os.path.dirname(script_dir)
        self.params_file_path = os.path.join(project_main_folder, "parameters.yaml")
        self.get_logger().info(f"Parameters YAML file path set to: {self.params_file_path}")

        self.pixel_map_publisher = self.create_publisher(Image, '/pixel_map', 10)
        self.save_pos_publisher = self.create_publisher(Empty, '/save_position', 10)
        self.get_logger().info("Publishers created for /pixel_map and /save_position")

        self.webcam_subscription = self.create_subscription(
            Image, '/webcam_image', self.webcam_callback, 10)
        self.edge_subscription = self.create_subscription(
            Image, '/edge_image', self.edge_callback, 10)
        self.error_subscription = self.create_subscription(
            RosString, '/control_error', self.error_message_callback, 10)
        self.get_logger().info("Subscribed to /webcam_image, /edge_image, and /control_error")

        if Img is not None:
            self._action_client = ActionClient(self, Img, '/process_edge_image')
            self.get_logger().info("Action client created for /process_edge_image.")
        else:
            self._action_client = None
            self.get_logger().error("Img action type not available. Action client NOT created.")

    def webcam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.webcam_frame_lock:
                self.latest_webcam_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Webcam CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in webcam_callback: {e}\n{traceback.format_exc()}')

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.edge_frame_lock:
                self.latest_edge_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Edge CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in edge_callback: {e}\n{traceback.format_exc()}')

    def error_message_callback(self, msg):
        try:
            with self.error_message_lock:
                self.latest_error_message = msg.data
        except Exception as e:
            self.get_logger().error(f'Error in error_message_callback: {e}\n{traceback.format_exc()}')

    def get_latest_webcam_frame(self):
        with self.webcam_frame_lock:
            return self.latest_webcam_frame.copy() if self.latest_webcam_frame is not None else None

    def get_latest_edge_frame(self):
        with self.edge_frame_lock:
            return self.latest_edge_frame.copy() if self.latest_edge_frame is not None else None

    def get_latest_processed_frame(self):
        with self.processed_frame_lock:
            return self.latest_processed_frame.copy() if self.latest_processed_frame is not None else None

    def get_latest_error_message(self):
        with self.error_message_lock:
            return self.latest_error_message

    def send_process_image_goal(self):
        if self._action_client is None:
            self.get_logger().error("Action client not available.")
            return False
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Action server '/process_edge_image' not available yet.")
            return False
        with self.webcam_frame_lock:
            frame_to_send = self.latest_webcam_frame
            if frame_to_send is None:
                self.get_logger().warn("No webcam frame available to send.")
                return False
            frame_copy = frame_to_send.copy()
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
        except CvBridgeError as e:
             self.get_logger().error(f"CV Bridge error converting frame for action: {e}")
             return False
        except Exception as e:
             self.get_logger().error(f"Unexpected error converting frame for action: {e}\n{traceback.format_exc()}")
             return False
        goal_msg = Img.Goal()
        goal_msg.image = image_msg
        self.get_logger().info('Sending goal request...')
        self.processing_active = True
        with self.processed_frame_lock:
            self.latest_processed_frame = None
        if not self._action_client.server_is_ready():
             self.get_logger().warn("Action server became unavailable before sending goal.")
             self.processing_active = False
             return False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = None
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while getting goal handle: {e}\n{traceback.format_exc()}')
            self.processing_active = False
            return
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().info('Goal rejected or failed :(')
            self.processing_active = False
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        processed_cv_image = None
        if GoalStatus is None:
             self.get_logger().error("GoalStatus message type not available. Cannot check action result status.")
             self.processing_active = False
             return
        try:
            result_response = future.result()
            if result_response is None:
                self.get_logger().error("Future for get_result_async returned None.")
                self.processing_active = False
                return
            result = result_response.result
            status = result_response.status
            if status == GoalStatus.STATUS_SUCCEEDED and result and result.processed_image:
                self.get_logger().info(f'Result received (status: {status}).')
                processed_cv_image = self.bridge.imgmsg_to_cv2(result.processed_image, desired_encoding='mono8')
            else:
                if status != GoalStatus.STATUS_SUCCEEDED:
                     self.get_logger().warn(f'Action did not succeed. Status: {status}')
                elif not result or not result.processed_image:
                     self.get_logger().warn(f'Action succeeded but returned no processed image (status: {status}).')
                else:
                     self.get_logger().warn(f'Action finished with unexpected state (status: {status}).')
        except CvBridgeError as e:
             self.get_logger().error(f'CV Bridge error processing action result image: {e}')
        except Exception as e:
             self.get_logger().error(f'Error processing action result: {e}\n{traceback.format_exc()}')
        finally:
            with self.processed_frame_lock:
                self.latest_processed_frame = processed_cv_image
            self.processing_active = False
            self.get_logger().info(f'Processing finished. Processed frame is {"available" if processed_cv_image is not None else "not available"}.')

    def publish_processed_image(self):
        frame_to_publish = None
        with self.processed_frame_lock:
            if self.latest_processed_frame is not None:
                frame_to_publish = self.latest_processed_frame.copy()
        if frame_to_publish is None:
            self.get_logger().warn("No processed image available to publish.")
            return False
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_to_publish, encoding='mono8')
            self.pixel_map_publisher.publish(image_msg)
            self.get_logger().info(f"Published processed image to {self.pixel_map_publisher.topic_name}")
            return True
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error converting processed frame for publishing: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing processed frame: {e}\n{traceback.format_exc()}")
            return False

    def publish_save_position(self):
        try:
            msg = Empty()
            self.save_pos_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.save_pos_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish to /save_position: {e}\n{traceback.format_exc()}")
            return False

    def launch_canvas_localisation_node(self):
        if self.canvas_localisation_process and self.canvas_localisation_process.poll() is None:
            self.get_logger().warn("Canvas localisation node might already be running.")
            return False, "Canvas localisation node might already be running."
        command = ['ros2', 'run', 'ur3_localisation', 'localisation_node']
        try:
            current_env = os.environ.copy()
            self.canvas_localisation_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched localisation_node."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 run' command not found. Is ROS 2 environment sourced correctly?")
            self.canvas_localisation_process = None
            return False, "Error: 'ros2 run' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch localisation_node: {e}\n{traceback.format_exc()}")
            self.canvas_localisation_process = None
            return False, f"Failed to launch node: {e}"

    def launch_robot_calibration_launchfile(self):
        if self.robot_calibration_process and self.robot_calibration_process.poll() is None:
            self.get_logger().warn("Robot calibration launch file might already be running.")
            return False, "Robot calibration might already be running."
        target_file = "/home/jarred/git/DalESelfEBot/ur3_control/calibration/ur3e_calibration_1.yaml"
        if not os.path.exists(target_file):
             self.get_logger().error(f"Calibration target file not found: {target_file}")
             return False, f"Calibration target file not found: {target_file}"
        command = [
            'ros2', 'launch', 'ur_calibration', 'calibration_correction.launch.py',
            'robot_ip:=192.168.0.191',
            f'target_filename:={target_file}'
        ]
        try:
            current_env = os.environ.copy()
            self.robot_calibration_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched robot calibration."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 launch' command not found. Is ROS 2 environment sourced correctly?")
            self.robot_calibration_process = None
            return False, "Error: 'ros2 launch' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot calibration: {e}\n{traceback.format_exc()}")
            self.robot_calibration_process = None
            return False, f"Failed to launch calibration: {e}"

    def set_robot_ip_in_yaml(self, ip_address):
        try:
            data = {}
            if os.path.exists(self.params_file_path):
                with open(self.params_file_path, 'r') as f:
                    try:
                        loaded_data = yaml.safe_load(f)
                        if loaded_data is not None: data = loaded_data
                    except yaml.YAMLError as e:
                        self.get_logger().error(f"Error parsing YAML {self.params_file_path}: {e}")
                        return False, f"Error parsing existing YAML: {e}"
            if 'robot' not in data or not isinstance(data.get('robot'), dict): data['robot'] = {}
            data['robot']['ip_address'] = ip_address
            with open(self.params_file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Robot IP set to {ip_address} in {self.params_file_path}")
            return True, f"Robot IP updated to {ip_address}."
        except PermissionError:
            self.get_logger().error(f"Permission denied for {self.params_file_path}.")
            return False, "Permission denied for parameters file."
        except Exception as e:
            self.get_logger().error(f"Failed to set IP in YAML: {e}\n{traceback.format_exc()}")
            return False, f"Unexpected error: {e}"

# MODIFICATION: Restored LocalisationWindow class from the user's older version
class LocalisationWindow(Toplevel):
    """
    A Toplevel window for handling the 4-corner canvas localisation process.
    """
    def __init__(self, master, ros_node):
        super().__init__(master)
        self.ros_node = ros_node
        self.master_app = master # Reference to the main app if needed
        self.title("Localisation")
        self.resizable(False, False) # Prevent resizing

        self.config(bg="#555555") # Slightly lighter grey for contrast

        self.calibration_step = 0 # 0: BL, 1: BR, 2: TR, 3: TL, 4: Done
        self.corner_order = ['BL', 'BR', 'TR', 'TL'] # Order of calibration

        main_frame = tk.Frame(self, padx=10, pady=10, bg="#555555")
        main_frame.pack(fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(main_frame, bg="#555555")
        button_frame.pack(pady=5)

        self.buttons = {}

        button_config = {
            "width": 12,
            "height": 2,
            "bg": "#303030",
            "fg": "white",
            "activebackground": "#454545",
            "relief": FLAT,
            "borderwidth": 0,
            "highlightthickness": 0
        }
        disabled_button_bg = "#404040"

        self.buttons['TL'] = tk.Button(button_frame, text="Top Left", **button_config,
                                       command=lambda: self.on_corner_button_click('TL'))
        self.buttons['TL'].grid(row=0, column=0, padx=5, pady=5)

        self.buttons['TR'] = tk.Button(button_frame, text="Top Right", **button_config,
                                       command=lambda: self.on_corner_button_click('TR'))
        self.buttons['TR'].grid(row=0, column=1, padx=5, pady=5)

        self.buttons['BL'] = tk.Button(button_frame, text="Bottom Left", **button_config,
                                       command=lambda: self.on_corner_button_click('BL'))
        self.buttons['BL'].grid(row=1, column=0, padx=5, pady=5)

        self.buttons['BR'] = tk.Button(button_frame, text="Bottom Right", **button_config,
                                       command=lambda: self.on_corner_button_click('BR'))
        self.buttons['BR'].grid(row=1, column=1, padx=5, pady=5)

        orientation_label = tk.Label(main_frame, text="▲\nRobot Position", font=("Arial", 10), bg="#555555", fg="white")
        orientation_label.pack(pady=(10, 5))

        self.loc_status_label = tk.Label(main_frame, text="Click Bottom Left", justify=tk.CENTER, height=2, bg="#555555", fg="white")
        self.loc_status_label.pack(pady=5, fill=tk.X)

        close_button = tk.Button(main_frame, text="Close", command=self.close_window, **button_config)
        close_button.pack(pady=10, side=tk.BOTTOM)

        self.disabled_button_bg = disabled_button_bg
        self.active_button_bg = button_config["bg"]

        self.update_button_states()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.transient(master)
        self.grab_set()
        self.lift()

        self.update_idletasks()
        master_x = master.winfo_rootx()
        master_y = master.winfo_rooty()
        master_width = master.winfo_width()
        master_height = master.winfo_height()
        win_width = self.winfo_width()
        win_height = self.winfo_height()
        x = master_x + (master_width - win_width) // 2
        y = master_y + (master_height - win_height) // 2
        self.geometry(f'+{x}+{y}')

    def update_button_states(self):
        """Enables/disables buttons based on the current calibration step."""
        if self.calibration_step >= len(self.corner_order):
            expected_corner = None
            self.loc_status_label.config(text="Calibration Complete!")
        else:
            expected_corner = self.corner_order[self.calibration_step]
            corner_name = self.buttons[expected_corner]['text']
            self.loc_status_label.config(text=f"Move robot to {corner_name}\nand click the button.")

        for corner_id, button in self.buttons.items():
            if self.calibration_step >= len(self.corner_order):
                button.config(state=DISABLED, bg=self.disabled_button_bg)
            else:
                if corner_id == expected_corner:
                    button.config(state=NORMAL, bg=self.active_button_bg)
                else:
                    button.config(state=DISABLED, bg=self.disabled_button_bg)

    def on_corner_button_click(self, corner_id):
        """Handles clicks on the corner buttons."""
        if self.calibration_step >= len(self.corner_order):
            print("Calibration already complete.")
            return

        expected_corner = self.corner_order[self.calibration_step]

        if corner_id == expected_corner:
            print(f"Correct button clicked: {corner_id}")
            success = self.ros_node.publish_save_position()
            if success:
                self.calibration_step += 1
                self.update_button_states()
            else:
                messagebox.showerror("Publish Error", "Failed to publish to /save_position topic. Please check ROS connections and try again.", parent=self)
        else:
            print(f"Incorrect button clicked: {corner_id}. Expected: {expected_corner}")
            expected_corner_name = self.buttons[expected_corner]['text']
            messagebox.showwarning("Incorrect Button", f"Please click the '{expected_corner_name}' button next.", parent=self)

    def close_window(self):
        """Closes the localisation window and informs the main app."""
        print("Localisation window close requested.")
        self.grab_release()
        if self.master_app:
            self.master_app.on_localisation_close()
        self.destroy() # MODIFICATION: Ensure window is destroyed


# --- Main GUI Application ---
class ActionViewerApp(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.settings_window = None
        self.localisation_window = None # This will hold the LocalisationWindow instance
        self.title("DalESelfEBot GUI")
        self.geometry("1040x640")
        self.minsize(800, 500)
        self.dark_grey, self.text_color, self.error_text_color = "#404040", "white", "red"
        self.config(bg=self.dark_grey)
        self.running, self.display_mode, self.last_processed_frame_available = True, "live", False
        self.top_frame = tk.Frame(self, bg=self.dark_grey)
        self.processed_display_frame = tk.Frame(self, bg=self.dark_grey)
        self.control_frame = tk.Frame(self, bg=self.dark_grey)

        for i, (text, label_attr) in enumerate([("Webcam Feed", "webcam_label"), ("Edge Feed", "edge_label")]):
            container = tk.Frame(self.top_frame, bd=0, relief=FLAT, bg=self.dark_grey)
            container.pack(side=tk.LEFT if i == 0 else tk.RIGHT, padx=5, pady=5, fill=tk.BOTH, expand=True)
            tk.Label(container, text=text, bg=self.dark_grey, fg=self.text_color).pack()
            setattr(self, label_attr, tk.Label(container, bg=self.dark_grey))
            getattr(self, label_attr).pack(fill=tk.BOTH, expand=True)

        processed_container = tk.Frame(self.processed_display_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        processed_container.pack(side=tk.TOP, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(processed_container, text="Processed Image", bg=self.dark_grey, fg=self.text_color).pack()
        self.processed_label = tk.Label(processed_container, bg=self.dark_grey)
        self.processed_label.pack(fill=tk.BOTH, expand=True)

        button_subframe = tk.Frame(self.control_frame, bg=self.dark_grey)
        button_subframe.pack(side=tk.LEFT, padx=5)
        btn_cfg = {"bg": "#303030", "fg": self.text_color, "activebackground": "#454545", "relief": FLAT, "borderwidth": 0, "highlightthickness": 0}
        self.capture_button = tk.Button(button_subframe, text="Capture Image", width=15, command=self.on_capture_button_click, **btn_cfg)
        self.capture_button.pack(side=tk.LEFT, padx=(0, 5))
        self.draw_button = tk.Button(button_subframe, text="Draw", width=10, command=self.on_draw_button_click, state=DISABLED, **btn_cfg)
        self.draw_button.pack(side=tk.LEFT, padx=5)
        self.settings_button = tk.Button(self.control_frame, text="Settings", width=10, command=self.open_settings_window, **btn_cfg)
        self.settings_button.pack(side=tk.RIGHT, padx=5)
        msg_bars_frame = tk.Frame(self.control_frame, bg=self.dark_grey)
        msg_bars_frame.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        self.status_label = tk.Label(msg_bars_frame, text="Initializing...", anchor='w', bg=self.dark_grey, fg=self.text_color)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.error_status_label = tk.Label(msg_bars_frame, text="", anchor='w', bg=self.dark_grey, fg=self.error_text_color)
        self.error_status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))

        self.control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, padx=5)
        self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.update_thread = threading.Thread(target=self.update_image_display, daemon=True)
        self.update_thread.start()
        print("GUI update thread started.")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def open_settings_window(self):
        print("Settings button clicked.")
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return
        self.settings_window = Toplevel(self)
        self.settings_window.title("Robot Settings")
        self.settings_window.geometry("350x250")
        self.settings_window.resizable(False, False)
        settings_bg = "#555555"
        button_config = {"width": 20, "bg": "#303030", "fg": "white", "activebackground": "#454545", "relief": FLAT, "borderwidth": 0, "highlightthickness": 0}
        self.settings_window.config(bg=settings_bg)
        settings_frame = tk.Frame(self.settings_window, padx=15, pady=15, bg=settings_bg)
        settings_frame.pack(fill=tk.BOTH, expand=True)

        localise_button = tk.Button(settings_frame, text="Localise Canvas", **button_config, command=self.on_localise_canvas_click)
        localise_button.pack(pady=8)
        set_ip_button = tk.Button(settings_frame, text="Set Robot IP Address", **button_config, command=self.on_set_robot_ip_click)
        set_ip_button.pack(pady=8)
        calibrate_robot_button = tk.Button(settings_frame, text="Calibrate Robot", **button_config, command=self.on_calibrate_robot_click)
        calibrate_robot_button.pack(pady=8)
        close_button = tk.Button(settings_frame, text="Close", command=self.on_settings_close, **button_config)
        close_button.pack(pady=12, side=tk.BOTTOM)
        self.settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
        self.settings_window.transient(self)
        self.settings_window.grab_set()
        self.settings_window.lift()
        self.settings_window.update_idletasks()
        master_x, master_y = self.winfo_rootx(), self.winfo_rooty()
        master_width, master_height = self.winfo_width(), self.winfo_height()
        win_width, win_height = self.settings_window.winfo_width(), self.settings_window.winfo_height()
        x, y = master_x + (master_width - win_width) // 2, master_y + (master_height - win_height) // 2
        self.settings_window.geometry(f'+{x}+{y}')

    def on_set_robot_ip_click(self):
        print("Set Robot IP Address button clicked.")
        new_ip = simpledialog.askstring("Set Robot IP",
                                        "Enter new Robot IP Address:",
                                        parent=self.settings_window)
        if new_ip is not None:
            new_ip = new_ip.strip()
            if not new_ip:
                messagebox.showwarning("Input Error", "IP address cannot be empty.", parent=self.settings_window)
                return
            success, message = self.ros_node.set_robot_ip_in_yaml(new_ip)
            if success:
                messagebox.showinfo("Success", message, parent=self.settings_window)
            else:
                messagebox.showerror("Error", f"Failed to set IP: {message}", parent=self.settings_window)
        else:
            print("Set Robot IP cancelled by user.")

    def on_settings_close(self):
        if self.settings_window:
            print("Settings window closed.")
            self.settings_window.grab_release()
            self.settings_window.destroy()
            self.settings_window = None

    # This method creates and shows the LocalisationWindow
    def open_localisation_window(self):
        if self.localisation_window is not None and self.localisation_window.winfo_exists():
            self.localisation_window.lift()
            return
        self.localisation_window = LocalisationWindow(self, self.ros_node)

    # This method is called by LocalisationWindow when it's closed
    def on_localisation_close(self):
        if self.localisation_window: # Check if it exists (it should if close_window was called from it)
            print("Localisation window closed by child.")
            # The child window now calls self.destroy(), so we just nullify the reference here.
            self.localisation_window = None

    # This is the callback for the "Localise Canvas" button
    def on_localise_canvas_click(self):
        print("Localise Canvas button clicked.")
        success_launch, msg_launch = self.ros_node.launch_canvas_localisation_node()
        if not success_launch:
            messagebox.showerror("Localisation Error", f"Failed to launch node: {msg_launch}", parent=self.settings_window)
            return
        # If launch is successful, open the localisation window
        self.open_localisation_window()
        messagebox.showinfo("Localisation", msg_launch + "\nLocalisation window opened.", parent=self.settings_window)


    def on_calibrate_robot_click(self):
        print("Calibrate Robot button clicked.")
        success, msg = self.ros_node.launch_robot_calibration_launchfile()
        messagebox.showinfo("Robot Calibration", msg, parent=self.settings_window) if success else messagebox.showerror("Robot Calibration Error", msg, parent=self.settings_window)

    def on_capture_button_click(self):
        if not self.running: return
        if self.display_mode == "live":
            print("Capture Image button clicked.")
            self.status_label.config(text="Sending processing request...")
            self.capture_button.config(state=DISABLED)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            if self.processed_label.winfo_exists():
                self.processed_label.configure(image=None)
                if hasattr(self.processed_label, 'image'): self.processed_label.image = None
            if not self.ros_node.send_process_image_goal() and not self.ros_node.processing_active:
                self.status_label.config(text="Failed to send request.")
                messagebox.showerror("Action Error", "Could not send processing request.")
                self.capture_button.config(state=NORMAL)
        elif self.display_mode == "processed":
            print("Take Another button clicked.")
            self.display_mode = "live"
            self.capture_button.config(text="Capture Image", state=NORMAL)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            with self.ros_node.processed_frame_lock: self.ros_node.latest_processed_frame = None
            if self.processed_label.winfo_exists():
                 self.processed_label.configure(image=None)
                 if hasattr(self.processed_label, 'image'): self.processed_label.image = None
            self.status_label.config(text="Ready for new capture.")

    def on_draw_button_click(self):
        if not self.running or not self.last_processed_frame_available:
            messagebox.showwarning("Publish Error", "No valid processed image to publish.", parent=self)
            self.status_label.config(text="Draw failed: No image.")
            return
        print("Draw button clicked.")
        self.status_label.config(text="Publishing to /pixel_map...")
        self.draw_button.config(state=DISABLED)
        self.capture_button.config(state=DISABLED)
        if self.ros_node.publish_processed_image():
            self.status_label.config(text="Published to /pixel_map.")
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
        else:
            self.status_label.config(text="Failed to publish.")
            messagebox.showerror("Publish Error", "Could not publish processed image.", parent=self)
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
            self.draw_button.config(state=NORMAL if self.last_processed_frame_available else DISABLED)

    def update_image_display(self):
        last_display_mode = None
        while self.running and rclpy.ok():
            if not self.winfo_exists(): break
            webcam_frame, edge_frame, processed_frame = (
                self.ros_node.get_latest_webcam_frame(),
                self.ros_node.get_latest_edge_frame(),
                self.ros_node.get_latest_processed_frame())
            is_processing, current_processed_frame_available = self.ros_node.processing_active, (processed_frame is not None)

            if self.error_status_label.winfo_exists():
                latest_error = self.ros_node.get_latest_error_message()
                err_text = f"ERR: {latest_error}" if latest_error else ""
                if self.error_status_label.cget("text") != err_text: self.error_status_label.config(text=err_text)

            if self.display_mode != last_display_mode:
                try:
                    print(f"Switching view to {'Live Feeds' if self.display_mode == 'live' else 'Processed Image'}")
                    self.processed_display_frame.pack_forget() if self.display_mode == "live" and self.processed_display_frame.winfo_ismapped() else None
                    self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5) if self.display_mode == "live" and not self.top_frame.winfo_ismapped() else None
                    self.top_frame.pack_forget() if self.display_mode == "processed" and self.top_frame.winfo_ismapped() else None
                    self.processed_display_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5) if self.display_mode == "processed" and not self.processed_display_frame.winfo_ismapped() else None
                    last_display_mode = self.display_mode
                except tk.TclError as e: self.ros_node.get_logger().warn(f"TclError: {e}"); last_display_mode = None
            try:
                if self.display_mode == "live":
                    for frame, label_widget in [(webcam_frame, self.webcam_label), (edge_frame, self.edge_label)]:
                        if frame is not None and label_widget.winfo_exists():
                            try:
                                color_conv = cv2.COLOR_BGR2RGB if frame.shape[-1] == 3 else cv2.COLOR_GRAY2RGB
                                img_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(frame, color_conv)))
                                label_widget.configure(image=img_tk); label_widget.image = img_tk
                            except Exception as e: self.ros_node.get_logger().error(f"Error updating {label_widget}: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                        elif label_widget.winfo_exists() and hasattr(label_widget, 'image') and label_widget.image is not None: pass
                elif self.display_mode == "processed":
                    if current_processed_frame_available and self.processed_label.winfo_exists():
                        try:
                            img_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2RGB)))
                            self.processed_label.configure(image=img_tk); self.processed_label.image = img_tk
                        except Exception as e: self.ros_node.get_logger().error(f"Error updating processed GUI: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                    elif not current_processed_frame_available and self.processed_label.winfo_exists() and hasattr(self.processed_label, 'image') and self.processed_label.image is not None:
                        self.processed_label.configure(image=None); self.processed_label.image = None
            except tk.TclError as e: self.ros_node.get_logger().warn(f"TclError image update: {e}")

            if not is_processing and self.ros_node._send_goal_future and self.display_mode == "live" and self.capture_button.cget('text') == "Capture Image":
                if current_processed_frame_available:
                    print("Processing complete, image available. Switching to processed view.")
                    self.display_mode, self.last_processed_frame_available = "processed", True
                    if self.capture_button.winfo_exists(): self.capture_button.config(text="Take Another", state=NORMAL)
                    if self.draw_button.winfo_exists(): self.draw_button.config(state=NORMAL)
                    if self.status_label.winfo_exists(): self.status_label.config(text="Processing complete. Image ready.")
                elif self.ros_node._get_result_future and self.ros_node._get_result_future.done():
                    print("Processing finished, but no valid image received.")
                    if self.status_label.winfo_exists(): self.status_label.config(text="Processing failed or no image returned.")
                    if self.capture_button.winfo_exists(): self.capture_button.config(state=NORMAL)
                self.ros_node._send_goal_future, self.ros_node._get_result_future = None, None
            
            if self.status_label.winfo_exists():
                current_status = self.status_label.cget("text")
                if is_processing:
                    if not current_status.startswith("Processing req"): self.status_label.config(text="Processing request...")
                    if self.capture_button.winfo_exists() and self.capture_button.cget('state')!=DISABLED: self.capture_button.config(state=DISABLED)
                    if self.draw_button.winfo_exists() and self.draw_button.cget('state')!=DISABLED: self.draw_button.config(state=DISABLED)
                else:
                    if self.display_mode == "live":
                        live_status = ("Waiting Webcam" if webcam_frame is None else "") + \
                                      (", " if webcam_frame is None and edge_frame is None else "") + \
                                      ("Waiting Edge" if edge_frame is None else "") or "Ready."
                        if not any(k in current_status for k in ["Fail","complete","Pub","new cap"]) or current_status in ["Wait","Ready.","Init"]:
                            if current_status != live_status: self.status_label.config(text=live_status)
                        if self.capture_button.winfo_exists() and self.capture_button.cget('state')==DISABLED and self.capture_button.cget('text')=="Capture Image": self.capture_button.config(state=NORMAL)
                        if self.draw_button.winfo_exists() and self.draw_button.cget('state')==NORMAL : self.draw_button.config(state=DISABLED)
                    elif self.display_mode == "processed":
                        if self.capture_button.winfo_exists() and self.capture_button.cget('state')==DISABLED and self.capture_button.cget('text')=="Take Another": self.capture_button.config(state=NORMAL)
                        if self.draw_button.winfo_exists():
                            draw_state = NORMAL if current_processed_frame_available else DISABLED
                            if self.draw_button.cget('state') != draw_state: self.draw_button.config(state=draw_state)
                        if not any(k in current_status for k in ["Pub","Fail to pub"]):
                            proc_status = "Processed image ready." if current_processed_frame_available else "No processed image available."
                            if not current_status.startswith("Processing complete") and current_status != proc_status : self.status_label.config(text=proc_status)
            self.last_processed_frame_available = current_processed_frame_available
            time.sleep(0.05)
        print("GUI update loop finished.")

    def on_closing(self):
        print("Main window on_closing called.")
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.running = False
            if hasattr(self, 'update_thread') and self.update_thread.is_alive():
                self.update_thread.join(timeout=0.5)
            for win_attr in ['localisation_window', 'settings_window']:
                win = getattr(self, win_attr, None)
                if win and win.winfo_exists(): win.destroy(); setattr(self, win_attr, None)
            print("Attempting to destroy main window..."); self.destroy(); print("Main window destroyed.")
        else: print("Quit cancelled.")

# Main execution block
def main(args=None):
    if Img is None or GoalStatus is None: print("Critical Error: Img/GoalStatus not found."); sys.exit(1)
    print("Initializing rclpy..."); rclpy.init(args=args)
    gui_node, app, ros_spin_thread, main_exception = None, None, None, None
    try:
        print("Creating GuiNode..."); gui_node = GuiNode(); print("GuiNode created.")
        def spin_ros():
            name = gui_node.get_name() if gui_node else "gui_node (pre-init)"
            print(f"ROS spin thread started for {name}.")
            try:
                if gui_node: rclpy.spin(gui_node)
                if rclpy.ok() and gui_node and gui_node.context.ok(): gui_node.get_logger().info("ROS spin normal.")
                else: print(f"ROS spin for {name} finished, context/node likely shutdown.")
            except Exception as e:
                log = f"Exception in ROS spin ({name}): {e}\n{traceback.format_exc()}"
                if rclpy.ok() and gui_node and gui_node.context.ok(): gui_node.get_logger().error(log)
                else: print(log)
            finally: print(f"ROS spin thread for {name} exiting.")
        ros_spin_thread = threading.Thread(target=spin_ros, daemon=True); ros_spin_thread.start()
        print("ROS spin thread created and started.")
        print("Creating ActionViewerApp..."); app = ActionViewerApp(ros_node=gui_node); print("ActionViewerApp created. Starting mainloop...")
        app.mainloop(); print("Tkinter mainloop finished.")
    except KeyboardInterrupt: print("\nKeyboardInterrupt received."); main_exception = "KeyboardInterrupt"
    except Exception as e: main_exception = e; print(f"Unhandled exception: {e}\n{traceback.format_exc()}")
    finally:
        print("Starting final cleanup...")
        if app and hasattr(app, 'running') and app.running: app.running = False
        if app and hasattr(app, 'tk') and app.tk:
            try:
                if app.winfo_exists(): print("Force destroying lingering Tk window..."); app.destroy()
            except tk.TclError: print("Tk window already destroyed.")
        if rclpy.ok():
            if gui_node:
                print("Terminating external processes...")
                for p_name, p_attr in [("canvas loc", "canvas_localisation_process"), ("robot calib", "robot_calibration_process")]:
                    proc = getattr(gui_node, p_attr, None)
                    if proc and proc.poll() is None:
                        print(f"Terminating {p_name}..."); proc.terminate()
                        try: proc.wait(timeout=1.0); print(f"{p_name} terminated.")
                        except subprocess.TimeoutExpired: print(f"{p_name} kill."); proc.kill()
                if gui_node.context.ok(): print("Destroying GUI node..."); gui_node.destroy_node(); print("GUI node destroyed.")
                else: print("GUI node already destroyed/context invalid.")
            print("Shutting down rclpy..."); rclpy.shutdown(); print("rclpy shutdown complete.")
        else: print("rclpy context already invalid.")
        if ros_spin_thread and ros_spin_thread.is_alive():
            print("Waiting for ROS spin thread..."); ros_spin_thread.join(timeout=2.0)
            print(f"ROS spin thread {'still alive.' if ros_spin_thread.is_alive() else 'joined.'}")
        print(f"Cleanup finished. Main exception: {main_exception}")
        sys.exit(0 if main_exception is None or main_exception == "KeyboardInterrupt" else 1)

if __name__ == '__main__':
    main()= self.create_subscription(
            Image, '/edge_image', self.edge_callback, 10)
        self.error_subscription = self.create_subscription(
            RosString, '/control_error', self.error_message_callback, 10)
        self.get_logger().info("Subscribed to /webcam_image, /edge_image, and /control_error")

        if Img is not None:
            self._action_client = ActionClient(self, Img, '/process_edge_image')
            self.get_logger().info("Action client created for /process_edge_image.")
        else:
            self._action_client = None
            self.get_logger().error("Img action type not available. Action client NOT created.")

    def webcam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.webcam_frame_lock:
                self.latest_webcam_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Webcam CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in webcam_callback: {e}\n{traceback.format_exc()}')

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.edge_frame_lock:
                self.latest_edge_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Edge CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in edge_callback: {e}\n{traceback.format_exc()}')

    def error_message_callback(self, msg):
        try:
            with self.error_message_lock:
                self.latest_error_message = msg.data
        except Exception as e:
            self.get_logger().error(f'Error in error_message_callback: {e}\n{traceback.format_exc()}')

    def get_latest_webcam_frame(self):
        with self.webcam_frame_lock:
            return self.latest_webcam_frame.copy() if self.latest_webcam_frame is not None else None

    def get_latest_edge_frame(self):
        with self.edge_frame_lock:
            return self.latest_edge_frame.copy() if self.latest_edge_frame is not None else None

    def get_latest_processed_frame(self):
        with self.processed_frame_lock:
            return self.latest_processed_frame.copy() if self.latest_processed_frame is not None else None

    def get_latest_error_message(self):
        with self.error_message_lock:
            return self.latest_error_message

    def send_process_image_goal(self):
        if self._action_client is None:
            self.get_logger().error("Action client not available.")
            return False
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Action server '/process_edge_image' not available yet.")
            return False
        with self.webcam_frame_lock:
            frame_to_send = self.latest_webcam_frame
            if frame_to_send is None:
                self.get_logger().warn("No webcam frame available to send.")
                return False
            frame_copy = frame_to_send.copy()
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
        except CvBridgeError as e:
             self.get_logger().error(f"CV Bridge error converting frame for action: {e}")
             return False
        except Exception as e:
             self.get_logger().error(f"Unexpected error converting frame for action: {e}\n{traceback.format_exc()}")
             return False
        goal_msg = Img.Goal()
        goal_msg.image = image_msg
        self.get_logger().info('Sending goal request...')
        self.processing_active = True
        with self.processed_frame_lock:
            self.latest_processed_frame = None
        if not self._action_client.server_is_ready():
             self.get_logger().warn("Action server became unavailable before sending goal.")
             self.processing_active = False
             return False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = None
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while getting goal handle: {e}\n{traceback.format_exc()}')
            self.processing_active = False
            return
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().info('Goal rejected or failed :(')
            self.processing_active = False
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        processed_cv_image = None
        if GoalStatus is None:
             self.get_logger().error("GoalStatus message type not available. Cannot check action result status.")
             self.processing_active = False
             return
        try:
            result_response = future.result()
            if result_response is None:
                self.get_logger().error("Future for get_result_async returned None.")
                self.processing_active = False
                return
            result = result_response.result
            status = result_response.status
            if status == GoalStatus.STATUS_SUCCEEDED and result and result.processed_image:
                self.get_logger().info(f'Result received (status: {status}).')
                processed_cv_image = self.bridge.imgmsg_to_cv2(result.processed_image, desired_encoding='mono8')
            else:
                if status != GoalStatus.STATUS_SUCCEEDED:
                     self.get_logger().warn(f'Action did not succeed. Status: {status}')
                elif not result or not result.processed_image:
                     self.get_logger().warn(f'Action succeeded but returned no processed image (status: {status}).')
                else:
                     self.get_logger().warn(f'Action finished with unexpected state (status: {status}).')
        except CvBridgeError as e:
             self.get_logger().error(f'CV Bridge error processing action result image: {e}')
        except Exception as e:
             self.get_logger().error(f'Error processing action result: {e}\n{traceback.format_exc()}')
        finally:
            with self.processed_frame_lock:
                self.latest_processed_frame = processed_cv_image
            self.processing_active = False
            self.get_logger().info(f'Processing finished. Processed frame is {"available" if processed_cv_image is not None else "not available"}.')

    def publish_processed_image(self):
        frame_to_publish = None
        with self.processed_frame_lock:
            if self.latest_processed_frame is not None:
                frame_to_publish = self.latest_processed_frame.copy()
        if frame_to_publish is None:
            self.get_logger().warn("No processed image available to publish.")
            return False
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_to_publish, encoding='mono8')
            self.pixel_map_publisher.publish(image_msg)
            self.get_logger().info(f"Published processed image to {self.pixel_map_publisher.topic_name}")
            return True
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error converting processed frame for publishing: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing processed frame: {e}\n{traceback.format_exc()}")
            return False

    def publish_save_position(self):
        try:
            msg = Empty()
            self.save_pos_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.save_pos_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish to /save_position: {e}\n{traceback.format_exc()}")
            return False

    def launch_canvas_localisation_node(self):
        if self.canvas_localisation_process and self.canvas_localisation_process.poll() is None:
            self.get_logger().warn("Canvas localisation node might already be running.")
            return False, "Canvas localisation node might already be running."
        command = ['ros2', 'run', 'ur3_localisation', 'localisation_node']
        try:
            current_env = os.environ.copy()
            self.canvas_localisation_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched localisation_node."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 run' command not found. Is ROS 2 environment sourced correctly?")
            self.canvas_localisation_process = None
            return False, "Error: 'ros2 run' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch localisation_node: {e}\n{traceback.format_exc()}")
            self.canvas_localisation_process = None
            return False, f"Failed to launch node: {e}"

    def launch_robot_calibration_launchfile(self):
        if self.robot_calibration_process and self.robot_calibration_process.poll() is None:
            self.get_logger().warn("Robot calibration launch file might already be running.")
            return False, "Robot calibration might already be running."
        target_file = "/home/jarred/git/DalESelfEBot/ur3_control/calibration/ur3e_calibration_1.yaml"
        if not os.path.exists(target_file):
             self.get_logger().error(f"Calibration target file not found: {target_file}")
             return False, f"Calibration target file not found: {target_file}"
        command = [
            'ros2', 'launch', 'ur_calibration', 'calibration_correction.launch.py',
            'robot_ip:=192.168.0.191',
            f'target_filename:={target_file}'
        ]
        try:
            current_env = os.environ.copy()
            self.robot_calibration_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched robot calibration."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 launch' command not found. Is ROS 2 environment sourced correctly?")
            self.robot_calibration_process = None
            return False, "Error: 'ros2 launch' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot calibration: {e}\n{traceback.format_exc()}")
            self.robot_calibration_process = None
            return False, f"Failed to launch calibration: {e}"

    def set_robot_ip_in_yaml(self, ip_address):
        try:
            data = {}
            if os.path.exists(self.params_file_path):
                with open(self.params_file_path, 'r') as f:
                    try:
                        loaded_data = yaml.safe_load(f)
                        if loaded_data is not None: data = loaded_data
                    except yaml.YAMLError as e:
                        self.get_logger().error(f"Error parsing YAML {self.params_file_path}: {e}")
                        return False, f"Error parsing existing YAML: {e}"
            if 'robot' not in data or not isinstance(data.get('robot'), dict): data['robot'] = {}
            data['robot']['ip_address'] = ip_address
            with open(self.params_file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Robot IP set to {ip_address} in {self.params_file_path}")
            return True, f"Robot IP updated to {ip_address}."
        except PermissionError:
            self.get_logger().error(f"Permission denied for {self.params_file_path}.")
            return False, "Permission denied for parameters file."
        except Exception as e:
            self.get_logger().error(f"Failed to set IP in YAML: {e}\n{traceback.format_exc()}")
            return False, f"Unexpected error: {e}"

# MODIFICATION: Restored LocalisationWindow class from the user's older version
class LocalisationWindow(Toplevel):
    """
    A Toplevel window for handling the 4-corner canvas localisation process.
    """
    def __init__(self, master, ros_node):
        super().__init__(master)
        self.ros_node = ros_node
        self.master_app = master # Reference to the main app if needed
        self.title("Localisation")
        self.resizable(False, False) # Prevent resizing

        self.config(bg="#555555") # Slightly lighter grey for contrast

        self.calibration_step = 0 # 0: BL, 1: BR, 2: TR, 3: TL, 4: Done
        self.corner_order = ['BL', 'BR', 'TR', 'TL'] # Order of calibration

        main_frame = tk.Frame(self, padx=10, pady=10, bg="#555555")
        main_frame.pack(fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(main_frame, bg="#555555")
        button_frame.pack(pady=5)

        self.buttons = {}

        button_config = {
            "width": 12,
            "height": 2,
            "bg": "#303030",
            "fg": "white",
            "activebackground": "#454545",
            "relief": FLAT,
            "borderwidth": 0,
            "highlightthickness": 0
        }
        disabled_button_bg = "#404040"

        self.buttons['TL'] = tk.Button(button_frame, text="Top Left", **button_config,
                                       command=lambda: self.on_corner_button_click('TL'))
        self.buttons['TL'].grid(row=0, column=0, padx=5, pady=5)

        self.buttons['TR'] = tk.Button(button_frame, text="Top Right", **button_config,
                                       command=lambda: self.on_corner_button_click('TR'))
        self.buttons['TR'].grid(row=0, column=1, padx=5, pady=5)

        self.buttons['BL'] = tk.Button(button_frame, text="Bottom Left", **button_config,
                                       command=lambda: self.on_corner_button_click('BL'))
        self.buttons['BL'].grid(row=1, column=0, padx=5, pady=5)

        self.buttons['BR'] = tk.Button(button_frame, text="Bottom Right", **button_config,
                                       command=lambda: self.on_corner_button_click('BR'))
        self.buttons['BR'].grid(row=1, column=1, padx=5, pady=5)

        orientation_label = tk.Label(main_frame, text="▲\nRobot Position", font=("Arial", 10), bg="#555555", fg="white")
        orientation_label.pack(pady=(10, 5))

        self.loc_status_label = tk.Label(main_frame, text="Click Bottom Left", justify=tk.CENTER, height=2, bg="#555555", fg="white")
        self.loc_status_label.pack(pady=5, fill=tk.X)

        close_button = tk.Button(main_frame, text="Close", command=self.close_window, **button_config)
        close_button.pack(pady=10, side=tk.BOTTOM)

        self.disabled_button_bg = disabled_button_bg
        self.active_button_bg = button_config["bg"]

        self.update_button_states()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.transient(master)
        self.grab_set()
        self.lift()

        self.update_idletasks()
        master_x = master.winfo_rootx()
        master_y = master.winfo_rooty()
        master_width = master.winfo_width()
        master_height = master.winfo_height()
        win_width = self.winfo_width()
        win_height = self.winfo_height()
        x = master_x + (master_width - win_width) // 2
        y = master_y + (master_height - win_height) // 2
        self.geometry(f'+{x}+{y}')

    def update_button_states(self):
        """Enables/disables buttons based on the current calibration step."""
        if self.calibration_step >= len(self.corner_order):
            expected_corner = None
            self.loc_status_label.config(text="Calibration Complete!")
        else:
            expected_corner = self.corner_order[self.calibration_step]
            corner_name = self.buttons[expected_corner]['text']
            self.loc_status_label.config(text=f"Move robot to {corner_name}\nand click the button.")

        for corner_id, button in self.buttons.items():
            if self.calibration_step >= len(self.corner_order):
                button.config(state=DISABLED, bg=self.disabled_button_bg)
            else:
                if corner_id == expected_corner:
                    button.config(state=NORMAL, bg=self.active_button_bg)
                else:
                    button.config(state=DISABLED, bg=self.disabled_button_bg)

    def on_corner_button_click(self, corner_id):
        """Handles clicks on the corner buttons."""
        if self.calibration_step >= len(self.corner_order):
            print("Calibration already complete.")
            return

        expected_corner = self.corner_order[self.calibration_step]

        if corner_id == expected_corner:
            print(f"Correct button clicked: {corner_id}")
            success = self.ros_node.publish_save_position()
            if success:
                self.calibration_step += 1
                self.update_button_states()
            else:
                messagebox.showerror("Publish Error", "Failed to publish to /save_position topic. Please check ROS connections and try again.", parent=self)
        else:
            print(f"Incorrect button clicked: {corner_id}. Expected: {expected_corner}")
            expected_corner_name = self.buttons[expected_corner]['text']
            messagebox.showwarning("Incorrect Button", f"Please click the '{expected_corner_name}' button next.", parent=self)

    def close_window(self):
        """Closes the localisation window and informs the main app."""
        print("Localisation window close requested.")
        self.grab_release()
        if self.master_app:
            self.master_app.on_localisation_close()
        self.destroy() # MODIFICATION: Ensure window is destroyed


# --- Main GUI Application ---
class ActionViewerApp(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.settings_window = None
        self.localisation_window = None # This will hold the LocalisationWindow instance
        self.title("DalESelfEBot GUI")
        self.geometry("1040x640")
        self.minsize(800, 500)
        self.dark_grey, self.text_color, self.error_text_color = "#404040", "white", "red"
        self.config(bg=self.dark_grey)
        self.running, self.display_mode, self.last_processed_frame_available = True, "live", False
        self.top_frame = tk.Frame(self, bg=self.dark_grey)
        self.processed_display_frame = tk.Frame(self, bg=self.dark_grey)
        self.control_frame = tk.Frame(self, bg=self.dark_grey)

        for i, (text, label_attr) in enumerate([("Webcam Feed", "webcam_label"), ("Edge Feed", "edge_label")]):
            container = tk.Frame(self.top_frame, bd=0, relief=FLAT, bg=self.dark_grey)
            container.pack(side=tk.LEFT if i == 0 else tk.RIGHT, padx=5, pady=5, fill=tk.BOTH, expand=True)
            tk.Label(container, text=text, bg=self.dark_grey, fg=self.text_color).pack()
            setattr(self, label_attr, tk.Label(container, bg=self.dark_grey))
            getattr(self, label_attr).pack(fill=tk.BOTH, expand=True)

        processed_container = tk.Frame(self.processed_display_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        processed_container.pack(side=tk.TOP, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(processed_container, text="Processed Image", bg=self.dark_grey, fg=self.text_color).pack()
        self.processed_label = tk.Label(processed_container, bg=self.dark_grey)
        self.processed_label.pack(fill=tk.BOTH, expand=True)

        button_subframe = tk.Frame(self.control_frame, bg=self.dark_grey)
        button_subframe.pack(side=tk.LEFT, padx=5)
        btn_cfg = {"bg": "#303030", "fg": self.text_color, "activebackground": "#454545", "relief": FLAT, "borderwidth": 0, "highlightthickness": 0}
        self.capture_button = tk.Button(button_subframe, text="Capture Image", width=15, command=self.on_capture_button_click, **btn_cfg)
        self.capture_button.pack(side=tk.LEFT, padx=(0, 5))
        self.draw_button = tk.Button(button_subframe, text="Draw", width=10, command=self.on_draw_button_click, state=DISABLED, **btn_cfg)
        self.draw_button.pack(side=tk.LEFT, padx=5)
        self.settings_button = tk.Button(self.control_frame, text="Settings", width=10, command=self.open_settings_window, **btn_cfg)
        self.settings_button.pack(side=tk.RIGHT, padx=5)
        msg_bars_frame = tk.Frame(self.control_frame, bg=self.dark_grey)
        msg_bars_frame.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        self.status_label = tk.Label(msg_bars_frame, text="Initializing...", anchor='w', bg=self.dark_grey, fg=self.text_color)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.error_status_label = tk.Label(msg_bars_frame, text="", anchor='w', bg=self.dark_grey, fg=self.error_text_color)
        self.error_status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))

        self.control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, padx=5)
        self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.update_thread = threading.Thread(target=self.update_image_display, daemon=True)
        self.update_thread.start()
        print("GUI update thread started.")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def open_settings_window(self):
        print("Settings button clicked.")
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return
        self.settings_window = Toplevel(self)
        self.settings_window.title("Robot Settings")
        self.settings_window.geometry("350x250")
        self.settings_window.resizable(False, False)
        settings_bg = "#555555"
        button_config = {"width": 20, "bg": "#303030", "fg": "white", "activebackground": "#454545", "relief": FLAT, "borderwidth": 0, "highlightthickness": 0}
        self.settings_window.config(bg=settings_bg)
        settings_frame = tk.Frame(self.settings_window, padx=15, pady=15, bg=settings_bg)
        settings_frame.pack(fill=tk.BOTH, expand=True)

        localise_button = tk.Button(settings_frame, text="Localise Canvas", **button_config, command=self.on_localise_canvas_click)
        localise_button.pack(pady=8)
        set_ip_button = tk.Button(settings_frame, text="Set Robot IP Address", **button_config, command=self.on_set_robot_ip_click)
        set_ip_button.pack(pady=8)
        calibrate_robot_button = tk.Button(settings_frame, text="Calibrate Robot", **button_config, command=self.on_calibrate_robot_click)
        calibrate_robot_button.pack(pady=8)
        close_button = tk.Button(settings_frame, text="Close", command=self.on_settings_close, **button_config)
        close_button.pack(pady=12, side=tk.BOTTOM)
        self.settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
        self.settings_window.transient(self)
        self.settings_window.grab_set()
        self.settings_window.lift()
        self.settings_window.update_idletasks()
        master_x, master_y = self.winfo_rootx(), self.winfo_rooty()
        master_width, master_height = self.winfo_width(), self.winfo_height()
        win_width, win_height = self.settings_window.winfo_width(), self.settings_window.winfo_height()
        x, y = master_x + (master_width - win_width) // 2, master_y + (master_height - win_height) // 2
        self.settings_window.geometry(f'+{x}+{y}')

    def on_set_robot_ip_click(self):
        print("Set Robot IP Address button clicked.")
        new_ip = simpledialog.askstring("Set Robot IP",
                                        "Enter new Robot IP Address:",
                                        parent=self.settings_window)
        if new_ip is not None:
            new_ip = new_ip.strip()
            if not new_ip:
                messagebox.showwarning("Input Error", "IP address cannot be empty.", parent=self.settings_window)
                return
            success, message = self.ros_node.set_robot_ip_in_yaml(new_ip)
            if success:
                messagebox.showinfo("Success", message, parent=self.settings_window)
            else:
                messagebox.showerror("Error", f"Failed to set IP: {message}", parent=self.settings_window)
        else:
            print("Set Robot IP cancelled by user.")

    def on_settings_close(self):
        if self.settings_window:
            print("Settings window closed.")
            self.settings_window.grab_release()
            self.settings_window.destroy()
            self.settings_window = None

    # This method creates and shows the LocalisationWindow
    def open_localisation_window(self):
        if self.localisation_window is not None and self.localisation_window.winfo_exists():
            self.localisation_window.lift()
            return
        self.localisation_window = LocalisationWindow(self, self.ros_node)

    # This method is called by LocalisationWindow when it's closed
    def on_localisation_close(self):
        if self.localisation_window: # Check if it exists (it should if close_window was called from it)
            print("Localisation window closed by child.")
            # The child window now calls self.destroy(), so we just nullify the reference here.
            self.localisation_window = None

    # This is the callback for the "Localise Canvas" button
    def on_localise_canvas_click(self):
        print("Localise Canvas button clicked.")
        success_launch, msg_launch = self.ros_node.launch_canvas_localisation_node()
        if not success_launch:
            messagebox.showerror("Localisation Error", f"Failed to launch node: {msg_launch}", parent=self.settings_window)
            return
        # If launch is successful, open the localisation window
        self.open_localisation_window()
        messagebox.showinfo("Localisation", msg_launch + "\nLocalisation window opened.", parent=self.settings_window)


    def on_calibrate_robot_click(self):
        print("Calibrate Robot button clicked.")
        success, msg = self.ros_node.launch_robot_calibration_launchfile()
        messagebox.showinfo("Robot Calibration", msg, parent=self.settings_window) if success else messagebox.showerror("Robot Calibration Error", msg, parent=self.settings_window)

    def on_capture_button_click(self):
        if not self.running: return
        if self.display_mode == "live":
            print("Capture Image button clicked.")
            self.status_label.config(text="Sending processing request...")
            self.capture_button.config(state=DISABLED)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            if self.processed_label.winfo_exists():
                self.processed_label.configure(image=None)
                if hasattr(self.processed_label, 'image'): self.processed_label.image = None
            if not self.ros_node.send_process_image_goal() and not self.ros_node.processing_active:
                self.status_label.config(text="Failed to send request.")
                messagebox.showerror("Action Error", "Could not send processing request.")
                self.capture_button.config(state=NORMAL)
        elif self.display_mode == "processed":
            print("Take Another button clicked.")
            self.display_mode = "live"
            self.capture_button.config(text="Capture Image", state=NORMAL)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            with self.ros_node.processed_frame_lock: self.ros_node.latest_processed_frame = None
            if self.processed_label.winfo_exists():
                 self.processed_label.configure(image=None)
                 if hasattr(self.processed_label, 'image'): self.processed_label.image = None
            self.status_label.config(text="Ready for new capture.")

    def on_draw_button_click(self):
        if not self.running or not self.last_processed_frame_available:
            messagebox.showwarning("Publish Error", "No valid processed image to publish.", parent=self)
            self.status_label.config(text="Draw failed: No image.")
            return
        print("Draw button clicked.")
        self.status_label.config(text="Publishing to /pixel_map...")
        self.draw_button.config(state=DISABLED)
        self.capture_button.config(state=DISABLED)
        if self.ros_node.publish_processed_image():
            self.status_label.config(text="Published to /pixel_map.")
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
        else:
            self.status_label.config(text="Failed to publish.")
            messagebox.showerror("Publish Error", "Could not publish processed image.", parent=self)
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
            self.draw_button.config(state=NORMAL if self.last_processed_frame_available else DISABLED)

    def update_image_display(self):
        last_display_mode = None
        while self.running and rclpy.ok():
            if not self.winfo_exists(): break
            webcam_frame, edge_frame, processed_frame = (
                self.ros_node.get_latest_webcam_frame(),
                self.ros_node.get_latest_edge_frame(),
                self.ros_node.get_latest_processed_frame())
            is_processing, current_processed_frame_available = self.ros_node.processing_active, (processed_frame is not None)

            if self.error_status_label.winfo_exists():
                latest_error = self.ros_node.get_latest_error_message()
                err_text = f"ERR: {latest_error}" if latest_error else ""
                if self.error_status_label.cget("text") != err_text: self.error_status_label.config(text=err_text)

            if self.display_mode != last_display_mode:
                try:
                    print(f"Switching view to {'Live Feeds' if self.display_mode == 'live' else 'Processed Image'}")
                    self.processed_display_frame.pack_forget() if self.display_mode == "live" and self.processed_display_frame.winfo_ismapped() else None
                    self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5) if self.display_mode == "live" and not self.top_frame.winfo_ismapped() else None
                    self.top_frame.pack_forget() if self.display_mode == "processed" and self.top_frame.winfo_ismapped() else None
                    self.processed_display_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5) if self.display_mode == "processed" and not self.processed_display_frame.winfo_ismapped() else None
                    last_display_mode = self.display_mode
                except tk.TclError as e: self.ros_node.get_logger().warn(f"TclError: {e}"); last_display_mode = None
            try:
                if self.display_mode == "live":
                    for frame, label_widget in [(webcam_frame, self.webcam_label), (edge_frame, self.edge_label)]:
                        if frame is not None and label_widget.winfo_exists():
                            try:
                                color_conv = cv2.COLOR_BGR2RGB if frame.shape[-1] == 3 else cv2.COLOR_GRAY2RGB
                                img_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(frame, color_conv)))
                                label_widget.configure(image=img_tk); label_widget.image = img_tk
                            except Exception as e: self.ros_node.get_logger().error(f"Error updating {label_widget}: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                        elif label_widget.winfo_exists() and hasattr(label_widget, 'image') and label_widget.image is not None: pass
                elif self.display_mode == "processed":
                    if current_processed_frame_available and self.processed_label.winfo_exists():
                        try:
                            img_tk = ImageTk.PhotoImage(image=PILImage.fromarray(cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2RGB)))
                            self.processed_label.configure(image=img_tk); self.processed_label.image = img_tk
                        except Exception as e: self.ros_node.get_logger().error(f"Error updating processed GUI: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                    elif not current_processed_frame_available and self.processed_label.winfo_exists() and hasattr(self.processed_label, 'image') and self.processed_label.image is not None:
                        self.processed_label.configure(image=None); self.processed_label.image = None
            except tk.TclError as e: self.ros_node.get_logger().warn(f"TclError image update: {e}")

            if not is_processing and self.ros_node._send_goal_future and self.display_mode == "live" and self.capture_button.cget('text') == "Capture Image":
                if current_processed_frame_available:
                    print("Processing complete, image available. Switching to processed view.")
                    self.display_mode, self.last_processed_frame_available = "processed", True
                    if self.capture_button.winfo_exists(): self.capture_button.config(text="Take Another", state=NORMAL)
                    if self.draw_button.winfo_exists(): self.draw_button.config(state=NORMAL)
                    if self.status_label.winfo_exists(): self.status_label.config(text="Processing complete. Image ready.")
                elif self.ros_node._get_result_future and self.ros_node._get_result_future.done():
                    print("Processing finished, but no valid image received.")
                    if self.status_label.winfo_exists(): self.status_label.config(text="Processing failed or no image returned.")
                    if self.capture_button.winfo_exists(): self.capture_button.config(state=NORMAL)
                self.ros_node._send_goal_future, self.ros_node._get_result_future = None, None
            
            if self.status_label.winfo_exists():
                current_status = self.status_label.cget("text")
                if is_processing:
                    if not current_status.startswith("Processing req"): self.status_label.config(text="Processing request...")
                    if self.capture_button.winfo_exists() and self.capture_button.cget('state')!=DISABLED: self.capture_button.config(state=DISABLED)
                    if self.draw_button.winfo_exists() and self.draw_button.cget('state')!=DISABLED: self.draw_button.config(state=DISABLED)
                else:
                    if self.display_mode == "live":
                        live_status = ("Waiting Webcam" if webcam_frame is None else "") + \
                                      (", " if webcam_frame is None and edge_frame is None else "") + \
                                      ("Waiting Edge" if edge_frame is None else "") or "Ready."
                        if not any(k in current_status for k in ["Fail","complete","Pub","new cap"]) or current_status in ["Wait","Ready.","Init"]:
                            if current_status != live_status: self.status_label.config(text=live_status)
                        if self.capture_button.winfo_exists() and self.capture_button.cget('state')==DISABLED and self.capture_button.cget('text')=="Capture Image": self.capture_button.config(state=NORMAL)
                        if self.draw_button.winfo_exists() and self.draw_button.cget('state')==NORMAL : self.draw_button.config(state=DISABLED)
                    elif self.display_mode == "processed":
                        if self.capture_button.winfo_exists() and self.capture_button.cget('state')==DISABLED and self.capture_button.cget('text')=="Take Another": self.capture_button.config(state=NORMAL)
                        if self.draw_button.winfo_exists():
                            draw_state = NORMAL if current_processed_frame_available else DISABLED
                            if self.draw_button.cget('state') != draw_state: self.draw_button.config(state=draw_state)
                        if not any(k in current_status for k in ["Pub","Fail to pub"]):
                            proc_status = "Processed image ready." if current_processed_frame_available else "No processed image available."
                            if not current_status.startswith("Processing complete") and current_status != proc_status : self.status_label.config(text=proc_status)
            self.last_processed_frame_available = current_processed_frame_available
            time.sleep(0.05)
        print("GUI update loop finished.")

    def on_closing(self):
        print("Main window on_closing called.")
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.running = False
            if hasattr(self, 'update_thread') and self.update_thread.is_alive():
                self.update_thread.join(timeout=0.5)
            for win_attr in ['localisation_window', 'settings_window']:
                win = getattr(self, win_attr, None)
                if win and win.winfo_exists(): win.destroy(); setattr(self, win_attr, None)
            print("Attempting to destroy main window..."); self.destroy(); print("Main window destroyed.")
        else: print("Quit cancelled.")

# Main execution block
def main(args=None):
    if Img is None or GoalStatus is None: print("Critical Error: Img/GoalStatus not found."); sys.exit(1)
    print("Initializing rclpy..."); rclpy.init(args=args)
    gui_node, app, ros_spin_thread, main_exception = None, None, None, None
    try:
        print("Creating GuiNode..."); gui_node = GuiNode(); print("GuiNode created.")
        def spin_ros():
            name = gui_node.get_name() if gui_node else "gui_node (pre-init)"
            print(f"ROS spin thread started for {name}.")
            try:
                if gui_node: rclpy.spin(gui_node)
                if rclpy.ok() and gui_node and gui_node.context.ok(): gui_node.get_logger().info("ROS spin normal.")
                else: print(f"ROS spin for {name} finished, context/node likely shutdown.")
            except Exception as e:
                log = f"Exception in ROS spin ({name}): {e}\n{traceback.format_exc()}"
                if rclpy.ok() and gui_node and gui_node.context.ok(): gui_node.get_logger().error(log)
                else: print(log)
            finally: print(f"ROS spin thread for {name} exiting.")
        ros_spin_thread = threading.Thread(target=spin_ros, daemon=True); ros_spin_thread.start()
        print("ROS spin thread created and started.")
        print("Creating ActionViewerApp..."); app = ActionViewerApp(ros_node=gui_node); print("ActionViewerApp created. Starting mainloop...")
        app.mainloop(); print("Tkinter mainloop finished.")
    except KeyboardInterrupt: print("\nKeyboardInterrupt received."); main_exception = "KeyboardInterrupt"
    except Exception as e: main_exception = e; print(f"Unhandled exception: {e}\n{traceback.format_exc()}")
    finally:
        print("Starting final cleanup...")
        if app and hasattr(app, 'running') and app.running: app.running = False
        if app and hasattr(app, 'tk') and app.tk:
            try:
                if app.winfo_exists(): print("Force destroying lingering Tk window..."); app.destroy()
            except tk.TclError: print("Tk window already destroyed.")
        if rclpy.ok():
            if gui_node:
                print("Terminating external processes...")
                for p_name, p_attr in [("canvas loc", "canvas_localisation_process"), ("robot calib", "robot_calibration_process")]:
                    proc = getattr(gui_node, p_attr, None)
                    if proc and proc.poll() is None:
                        print(f"Terminating {p_name}..."); proc.terminate()
                        try: proc.wait(timeout=1.0); print(f"{p_name} terminated.")
                        except subprocess.TimeoutExpired: print(f"{p_name} kill."); proc.kill()
                if gui_node.context.ok(): print("Destroying GUI node..."); gui_node.destroy_node(); print("GUI node destroyed.")
                else: print("GUI node already destroyed/context invalid.")
            print("Shutting down rclpy..."); rclpy.shutdown(); print("rclpy shutdown complete.")
        else: print("rclpy context already invalid.")
        if ros_spin_thread and ros_spin_thread.is_alive():
            print("Waiting for ROS spin thread..."); ros_spin_thread.join(timeout=2.0)
            print(f"ROS spin thread {'still alive.' if ros_spin_thread.is_alive() else 'joined.'}")
        print(f"Cleanup finished. Main exception: {main_exception}")
        sys.exit(0 if main_exception is None or main_exception == "KeyboardInterrupt" else 1)

if __name__ == '__main__':
    main()