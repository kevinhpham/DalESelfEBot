#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty # Import the Empty message type
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tkinter as tk
from tkinter import messagebox, Toplevel, DISABLED, NORMAL, FLAT # Import FLAT for relief
from PIL import Image as PILImage, ImageTk
import threading
import time
import numpy as np
import sys # Import sys for main function cleanup
import subprocess # Import subprocess to launch external nodes
import os # Import os for environment variables if needed
import traceback # START_FIX: Import traceback for logging exceptions

# Try to import the action definition
try:
    from img_prc_interface.action import Img
    # START_FIX: Import GoalStatus for checking action result status
    from action_msgs.msg import GoalStatus
    # END_FIX: Import GoalStatus
except ImportError:
    print("Error: Could not import Img action or GoalStatus.")
    print("Ensure the img_prc_interface package is built and sourced,")
    print("and that action_msgs is available.")
    Img = None # Set to None if import fails
    GoalStatus = None # Set to None if import fails
# END_FIX

# Define the ROS 2 Node for GUI interaction
class GuiNode(Node):
    """
    ROS 2 Node to subscribe to topics, interact with actions, and publish results/commands.
    """
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info('GUI Node started')
        self.bridge = CvBridge()

        # Attributes for various frames
        self.latest_webcam_frame = None
        self.webcam_frame_lock = threading.Lock()
        self.latest_edge_frame = None
        self.edge_frame_lock = threading.Lock()
        self.latest_processed_frame = None # This is the frame to publish
        self.processed_frame_lock = threading.Lock()
        self.processing_active = False
        self.canvas_localisation_process = None # Renamed from calibration_process
        self.robot_calibration_process = None # Process handle for robot calibration

        # --- Publishers ---
        self.pixel_map_publisher = self.create_publisher(Image, '/pixel_map', 10)
        self.save_pos_publisher = self.create_publisher(Empty, '/save_position', 10) # Publisher for calibration
        self.get_logger().info("Publishers created for /pixel_map and /save_position")
        # --- End Publishers ---

        # --- Subscribers ---
        self.webcam_subscription = self.create_subscription(
            Image, '/webcam_image', self.webcam_callback, 10)
        self.edge_subscription = self.create_subscription(
            Image, '/edge_image', self.edge_callback, 10)
        # --- End Subscribers ---

        # --- Action Client Setup ---
        if Img is not None:
            self._action_client = ActionClient(self, Img, '/process_edge_image')
            self.get_logger().info("Action client created for /process_edge_image.")
        else:
            self._action_client = None
            self.get_logger().error("Img action type not available. Action client NOT created.")
        # --- End Action Client Setup ---

    # --- Callbacks (webcam_callback, edge_callback) ---
    def webcam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.webcam_frame_lock:
                self.latest_webcam_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Webcam CV Bridge Error: {e}')
        except Exception as e:
            # START_FIX: Log exception with traceback
            self.get_logger().error(f'Error in webcam_callback: {e}\n{traceback.format_exc()}')
            # END_FIX

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.edge_frame_lock:
                self.latest_edge_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Edge CV Bridge Error: {e}')
        except Exception as e:
            # START_FIX: Log exception with traceback
            self.get_logger().error(f'Error in edge_callback: {e}\n{traceback.format_exc()}')
            # END_FIX
    # --- End Callbacks ---

    # --- Getters for Frames ---
    def get_latest_webcam_frame(self):
        with self.webcam_frame_lock:
            # Return a copy to prevent modification outside the lock
            return self.latest_webcam_frame.copy() if self.latest_webcam_frame is not None else None

    def get_latest_edge_frame(self):
        with self.edge_frame_lock:
            # Return a copy
            return self.latest_edge_frame.copy() if self.latest_edge_frame is not None else None

    def get_latest_processed_frame(self):
        with self.processed_frame_lock:
            # Return a copy
            return self.latest_processed_frame.copy() if self.latest_processed_frame is not None else None
    # --- End Getters ---

    # --- Action Client Methods (send_process_image_goal, etc.) ---
    def send_process_image_goal(self):
        if self._action_client is None:
            self.get_logger().error("Action client not available.")
            return False
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Action server '/process_edge_image' not available yet.")
            return False

        # Get frame within lock, then process outside lock
        with self.webcam_frame_lock:
            frame_to_send = self.latest_webcam_frame
            if frame_to_send is None:
                self.get_logger().warn("No webcam frame available to send.")
                return False
            # Make a copy to avoid race conditions if the frame updates during conversion
            frame_copy = frame_to_send.copy()

        try:
            # Convert the copied frame
            image_msg = self.bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
        except CvBridgeError as e:
             self.get_logger().error(f"CV Bridge error converting frame for action: {e}")
             return False
        except Exception as e:
             # START_FIX: Log exception with traceback
             self.get_logger().error(f"Unexpected error converting frame for action: {e}\n{traceback.format_exc()}")
             # END_FIX
             return False

        goal_msg = Img.Goal()
        goal_msg.image = image_msg

        self.get_logger().info('Sending goal request...')
        self.processing_active = True # Set flag BEFORE sending
        # Clear the previous processed frame immediately when starting a new request
        with self.processed_frame_lock:
            self.latest_processed_frame = None

        # Double-check server readiness right before sending
        if not self._action_client.server_is_ready():
             self.get_logger().warn("Action server became unavailable before sending goal.")
             self.processing_active = False # Reset flag
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
            # START_FIX: Log exception with traceback
            self.get_logger().error(f'Exception while getting goal handle: {e}\n{traceback.format_exc()}')
            # END_FIX
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
        # Placeholder for feedback processing if needed later
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f'Received feedback: {feedback.progress_status}')
        pass # Not used currently

    def get_result_callback(self, future):
        processed_cv_image = None # Initialize to None
        # Check if GoalStatus was imported successfully
        if GoalStatus is None:
             self.get_logger().error("GoalStatus message type not available. Cannot check action result status.")
             self.processing_active = False
             return

        try:
            result_response = future.result()
            if result_response is None:
                self.get_logger().error("Future for get_result_async returned None.")
                self.processing_active = False # Ensure flag is reset
                return

            result = result_response.result
            status = result_response.status # Get the status code

            # START_FIX: Use imported GoalStatus for comparison
            if status == GoalStatus.STATUS_SUCCEEDED and result and result.processed_image:
            # END_FIX
                self.get_logger().info(f'Result received (status: {status}).')
                # Convert the received image message to an OpenCV image
                processed_cv_image = self.bridge.imgmsg_to_cv2(result.processed_image, desired_encoding='mono8')
            else:
                # Log different messages based on status or lack of image
                # START_FIX: Use imported GoalStatus for comparison
                if status != GoalStatus.STATUS_SUCCEEDED:
                # END_FIX
                     self.get_logger().warn(f'Action did not succeed. Status: {status}')
                elif not result or not result.processed_image:
                     self.get_logger().warn(f'Action succeeded but returned no processed image (status: {status}).')
                else: # Should not happen if status is SUCCEEDED and result exists
                     self.get_logger().warn(f'Action finished with unexpected state (status: {status}).')

        except CvBridgeError as e:
             self.get_logger().error(f'CV Bridge error processing action result image: {e}')
             # Keep processed_cv_image as None
        except Exception as e:
             # START_FIX: Log exception with traceback, remove exc_info
             self.get_logger().error(f'Error processing action result: {e}\n{traceback.format_exc()}')
             # END_FIX
             # Keep processed_cv_image as None
        finally:
            # Store the successfully converted image (or None if conversion failed/no image received)
            with self.processed_frame_lock:
                self.latest_processed_frame = processed_cv_image
            # Always set processing_active to False when the result callback finishes
            self.processing_active = False
            self.get_logger().info(f'Processing finished. Processed frame is {"available" if processed_cv_image is not None else "not available"}.')
    # --- End Action Client Methods ---

    # --- Publishing Methods ---
    def publish_processed_image(self):
        """Publishes the latest valid processed image to /pixel_map."""
        frame_to_publish = None
        with self.processed_frame_lock:
            # Only proceed if there's a non-None frame
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
            # START_FIX: Log exception with traceback
            self.get_logger().error(f"Unexpected error publishing processed frame: {e}\n{traceback.format_exc()}")
            # END_FIX
            return False

    def publish_save_position(self):
        """Publishes an Empty message to /save_position."""
        try:
            msg = Empty()
            self.save_pos_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.save_pos_publisher.topic_name}")
            return True
        except Exception as e:
            # START_FIX: Log exception with traceback
            self.get_logger().error(f"Failed to publish to /save_position: {e}\n{traceback.format_exc()}")
            # END_FIX
            return False
    # --- End Publishing Methods ---

    # --- External Node Launch Methods ---
    def launch_canvas_localisation_node(self): # Renamed method
        """Launches the localisation_node in a separate process."""
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
            # START_FIX: Log exception with traceback
            self.get_logger().error(f"Failed to launch localisation_node: {e}\n{traceback.format_exc()}")
            # END_FIX
            self.canvas_localisation_process = None
            return False, f"Failed to launch node: {e}"

    def launch_robot_calibration_launchfile(self): # New method
        """Launches the UR calibration launch file."""
        if self.robot_calibration_process and self.robot_calibration_process.poll() is None:
            self.get_logger().warn("Robot calibration launch file might already be running.")
            return False, "Robot calibration might already be running."

        home_dir = os.path.expanduser("~")
        target_file = os.path.join(home_dir, "git/DalESelfEBot/ur3_control/calibration/ur3e_calibration_1.yaml")
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
            # START_FIX: Log exception with traceback
            self.get_logger().error(f"Failed to launch robot calibration: {e}\n{traceback.format_exc()}")
            # END_FIX
            self.robot_calibration_process = None
            return False, f"Failed to launch calibration: {e}"
    # --- End External Node Launch Methods ---

# --- Localisation Window ---
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

        # START_BG_COLOR: Set background for localisation window
        self.config(bg="#555555") # Slightly lighter grey for contrast
        # END_BG_COLOR

        self.calibration_step = 0 # 0: BL, 1: BR, 2: TR, 3: TL, 4: Done
        self.corner_order = ['BL', 'BR', 'TR', 'TL'] # Order of calibration

        # --- Create Widgets ---
        # START_BG_COLOR: Set background for frames and labels
        main_frame = tk.Frame(self, padx=10, pady=10, bg="#555555")
        main_frame.pack(fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(main_frame, bg="#555555")
        button_frame.pack(pady=5)

        self.buttons = {}

        # START_BUTTON_STYLE: Updated button configuration with highlightthickness
        button_config = {
            "width": 12,
            "height": 2,
            "bg": "#303030",  # Darker button background
            "fg": "white",
            "activebackground": "#454545", # Slightly lighter active background
            "relief": FLAT,  # Remove 3D effect
            "borderwidth": 0, # Remove border
            "highlightthickness": 0 # Remove focus highlight border
        }
        disabled_button_bg = "#404040" # Background for disabled buttons
        # END_BUTTON_STYLE

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

        # START_BUTTON_STYLE: Apply style to close button too
        close_button = tk.Button(main_frame, text="Close", command=self.close_window, **button_config)
        # END_BUTTON_STYLE

        # END_BG_COLOR
        close_button.pack(pady=10, side=tk.BOTTOM)
        # --- End Widgets ---

        # START_BUTTON_STYLE: Store disabled color for update_button_states
        self.disabled_button_bg = disabled_button_bg
        self.active_button_bg = button_config["bg"]
        # END_BUTTON_STYLE

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
                 # START_BUTTON_STYLE: Use defined disabled color
                button.config(state=DISABLED, bg=self.disabled_button_bg)
                 # END_BUTTON_STYLE
            else:
                if corner_id == expected_corner:
                     # START_BUTTON_STYLE: Use defined active color
                    button.config(state=NORMAL, bg=self.active_button_bg)
                     # END_BUTTON_STYLE
                else:
                     # START_BUTTON_STYLE: Use defined disabled color
                    button.config(state=DISABLED, bg=self.disabled_button_bg)
                     # END_BUTTON_STYLE


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
        # self.destroy() # Let master_app handle if needed


# --- Main GUI Application ---
class ActionViewerApp(tk.Tk):
    """
    Tkinter app displaying feeds, with Capture, Draw, and Settings buttons.
    Manages switching between live feeds and processed image display.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.settings_window = None
        self.localisation_window = None
        self.title("DalESelfEBot GUI")
        self.geometry("1040x640")
        self.minsize(800, 500)

        # START_BG_COLOR: Define colors
        self.dark_grey = "#404040"
        self.light_grey_widget = "#606060" # For buttons/frames if needed
        self.text_color = "white"
        # END_BG_COLOR

        # START_BG_COLOR: Set main window background
        self.config(bg=self.dark_grey)
        # END_BG_COLOR

        # --- App State ---
        self.running = True
        self.display_mode = "live"
        self.last_processed_frame_available = False

        # --- Frame Containers ---
        # START_BG_COLOR: Set background for main frames
        self.top_frame = tk.Frame(self, bg=self.dark_grey)
        self.processed_display_frame = tk.Frame(self, bg=self.dark_grey)
        self.control_frame = tk.Frame(self, bg=self.dark_grey)
        # END_BG_COLOR
        # --- End Frame Containers ---


        # --- Widgets within Top Frame (Live Feeds) ---
        # START_BG_COLOR: Set background/foreground for containers and labels
        # START_BUTTON_STYLE: Remove border from image containers
        webcam_frame_container = tk.Frame(self.top_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        # END_BUTTON_STYLE
        webcam_frame_container.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(webcam_frame_container, text="Webcam Feed", bg=self.dark_grey, fg=self.text_color).pack()
        # Image label itself shouldn't need bg set, it gets covered
        self.webcam_label = tk.Label(webcam_frame_container, bg=self.dark_grey)
        self.webcam_label.pack(fill=tk.BOTH, expand=True)

        # START_BUTTON_STYLE: Remove border from image containers
        edge_frame_container = tk.Frame(self.top_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        # END_BUTTON_STYLE
        edge_frame_container.pack(side=tk.RIGHT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(edge_frame_container, text="Edge Feed", bg=self.dark_grey, fg=self.text_color).pack()
        self.edge_label = tk.Label(edge_frame_container, bg=self.dark_grey)
        # END_BG_COLOR
        self.edge_label.pack(fill=tk.BOTH, expand=True)
        # --- End Widgets within Top Frame ---


        # --- Widgets within Processed Display Frame ---
        # START_BG_COLOR: Set background/foreground for container and label
        # START_BUTTON_STYLE: Remove border from image containers
        processed_frame_container = tk.Frame(self.processed_display_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        # END_BUTTON_STYLE
        processed_frame_container.pack(side=tk.TOP, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(processed_frame_container, text="Processed Image", bg=self.dark_grey, fg=self.text_color).pack()
        self.processed_label = tk.Label(processed_frame_container, bg=self.dark_grey)
        # END_BG_COLOR
        self.processed_label.pack(fill=tk.BOTH, expand=True)
        # --- End Widgets within Processed Display Frame ---


        # --- Widgets within Control Frame (Buttons & Status) ---
        # START_BG_COLOR: Set background/foreground for subframe and status label
        button_subframe = tk.Frame(self.control_frame, bg=self.dark_grey)
        button_subframe.pack(side=tk.LEFT, padx=5)

        # START_BUTTON_STYLE: Updated button configuration with highlightthickness
        button_config = {
            "bg": "#303030", # Darker button background
            "fg": self.text_color,
            "activebackground": "#454545", # Slightly lighter active background
            "relief": FLAT, # Remove 3D effect
            "borderwidth": 0, # Remove border
            "highlightthickness": 0 # Remove focus highlight border
        }
        # END_BUTTON_STYLE

        self.capture_button = tk.Button(button_subframe, text="Capture Image", width=15, command=self.on_capture_button_click, **button_config)
        self.capture_button.pack(side=tk.LEFT, padx=(0, 5))

        self.draw_button = tk.Button(button_subframe, text="Draw", width=10, command=self.on_draw_button_click, state=DISABLED, **button_config)
        self.draw_button.pack(side=tk.LEFT, padx=5)

        self.settings_button = tk.Button(self.control_frame, text="Settings", width=10, command=self.open_settings_window, **button_config)
        self.settings_button.pack(side=tk.RIGHT, padx=5)

        self.status_label = tk.Label(self.control_frame, text="Initializing...", anchor='w', bg=self.dark_grey, fg=self.text_color)
        # END_BG_COLOR
        self.status_label.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        # --- End Widgets within Control Frame ---


        # --- Initial Packing ---
        self.control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, padx=5)
        self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        # --- End Initial Packing ---


        # --- Start Background Tasks ---
        self.update_thread = threading.Thread(target=self.update_image_display, daemon=True)
        self.update_thread.start()
        print("GUI update thread started.")
        # --- End Background Tasks ---

        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    # --- Window Management ---
    def open_settings_window(self):
        """Opens the Toplevel window for settings."""
        print("Settings button clicked.")
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return

        self.settings_window = Toplevel(self)
        self.settings_window.title("Robot Settings")
        self.settings_window.geometry("350x200")
        self.settings_window.resizable(False, False)

        # START_BG_COLOR: Set background for settings window and contents
        settings_bg = "#555555" # Slightly lighter grey

        # START_BUTTON_STYLE: Updated button configuration for settings window with highlightthickness
        button_config = {
            "width": 20,
            "bg": "#303030", # Darker button background
            "fg": "white",
            "activebackground": "#454545", # Slightly lighter active background
            "relief": FLAT, # Remove 3D effect
            "borderwidth": 0, # Remove border
            "highlightthickness": 0 # Remove focus highlight border
        }
        # END_BUTTON_STYLE

        self.settings_window.config(bg=settings_bg)
        settings_frame = tk.Frame(self.settings_window, padx=15, pady=15, bg=settings_bg)
        settings_frame.pack(fill=tk.BOTH, expand=True)

        localise_button = tk.Button(settings_frame, text="Localise Canvas", **button_config, command=self.on_localise_canvas_click)
        localise_button.pack(pady=10)

        calibrate_robot_button = tk.Button(settings_frame, text="Calibrate Robot", **button_config, command=self.on_calibrate_robot_click)
        calibrate_robot_button.pack(pady=10)

        close_button = tk.Button(settings_frame, text="Close", command=self.on_settings_close, **button_config)
        # END_BG_COLOR
        close_button.pack(pady=15, side=tk.BOTTOM)

        self.settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
        self.settings_window.transient(self)
        self.settings_window.grab_set()
        self.settings_window.lift()

        self.settings_window.update_idletasks()
        master_x = self.winfo_rootx()
        master_y = self.winfo_rooty()
        master_width = self.winfo_width()
        master_height = self.winfo_height()
        win_width = self.settings_window.winfo_width()
        win_height = self.settings_window.winfo_height()
        x = master_x + (master_width - win_width) // 2
        y = master_y + (master_height - win_height) // 2
        self.settings_window.geometry(f'+{x}+{y}')


    def on_settings_close(self):
        """Callback when the settings window is closed."""
        if self.settings_window:
            print("Settings window closed.")
            self.settings_window.grab_release()
            self.settings_window.destroy()
            self.settings_window = None

    def open_localisation_window(self):
        """Opens the Toplevel window for localisation."""
        if self.localisation_window is not None and self.localisation_window.winfo_exists():
            self.localisation_window.lift()
            return
        # Pass main app colors to localisation window if needed, or set defaults there
        self.localisation_window = LocalisationWindow(self, self.ros_node)

    def on_localisation_close(self):
        """Callback when the localisation window is closed (called by LocalisationWindow)."""
        if self.localisation_window:
            print("Localisation window closed by child.")
            self.localisation_window = None
    # --- End Window Management ---

    # --- Button Callbacks ---
    def on_localise_canvas_click(self):
        """Callback for the Localise Canvas button *inside* the Settings window."""
        print("Localise Canvas button clicked.")
        success_launch, msg_launch = self.ros_node.launch_canvas_localisation_node()
        if not success_launch:
            messagebox.showerror("Localisation Error", f"Failed to launch node: {msg_launch}", parent=self.settings_window)
            return

        self.open_localisation_window()
        messagebox.showinfo("Localisation", msg_launch + "\nLocalisation window opened.", parent=self.settings_window)


    def on_calibrate_robot_click(self):
        """Callback for the Calibrate Robot button."""
        print("Calibrate Robot button clicked.")
        success_launch, msg_launch = self.ros_node.launch_robot_calibration_launchfile()
        if success_launch:
            messagebox.showinfo("Robot Calibration", msg_launch, parent=self.settings_window)
        else:
            messagebox.showerror("Robot Calibration Error", msg_launch, parent=self.settings_window)


    def on_capture_button_click(self):
        """Callback for the capture/take another button."""
        if not self.running: return

        if self.display_mode == "live":
            # --- Behaviour for "Capture Image" ---
            print("Capture Image button clicked.")
            self.status_label.config(text="Sending processing request...")
            self.capture_button.config(state=DISABLED)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False

            if self.processed_label.winfo_exists():
                self.processed_label.configure(image=None)
                self.processed_label.image = None

            success = self.ros_node.send_process_image_goal()

            if not success:
                if not self.ros_node.processing_active:
                    self.status_label.config(text="Failed to send request.")
                    messagebox.showerror("Action Error", "Could not send processing request. Check logs or server status.")
                    self.capture_button.config(state=NORMAL)
                else:
                    self.status_label.config(text="Request sent, waiting...")


        elif self.display_mode == "processed":
            # --- Behaviour for "Take Another" ---
            print("Take Another button clicked.")
            self.display_mode = "live"
            self.capture_button.config(text="Capture Image", state=NORMAL)
            self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False

            with self.ros_node.processed_frame_lock:
                 self.ros_node.latest_processed_frame = None

            if self.processed_label.winfo_exists():
                 self.processed_label.configure(image=None)
                 self.processed_label.image = None

            self.status_label.config(text="Ready for new capture.")


    def on_draw_button_click(self):
        """Callback for the Draw button."""
        if not self.running: return

        print("Draw button clicked.")
        if not self.last_processed_frame_available:
             messagebox.showwarning("Publish Error", "No valid processed image available to publish.", parent=self)
             self.status_label.config(text="Draw failed: No image.")
             return

        self.status_label.config(text="Publishing to /pixel_map...")
        self.draw_button.config(state=DISABLED)
        self.capture_button.config(state=DISABLED)

        success = self.ros_node.publish_processed_image()

        if success:
            self.status_label.config(text="Published to /pixel_map.")
            if self.display_mode == "processed" and self.capture_button.winfo_exists():
                 self.capture_button.config(state=NORMAL) # Re-enable "Take Another"
        else:
            self.status_label.config(text="Failed to publish.")
            messagebox.showerror("Publish Error", "Could not publish processed image. Check ROS logs.", parent=self)
            if self.display_mode == "processed" and self.capture_button.winfo_exists():
                 self.capture_button.config(state=NORMAL)
            if self.draw_button.winfo_exists():
                 self.draw_button.config(state=NORMAL if self.last_processed_frame_available else DISABLED)
    # --- End Button Callbacks ---


    def update_image_display(self):
        """Periodically updates image labels, button states, and manages frame visibility."""
        last_display_mode = None

        while self.running and rclpy.ok():
            if not self.winfo_exists(): break

            # --- Get Frames Safely ---
            webcam_frame = self.ros_node.get_latest_webcam_frame()
            edge_frame = self.ros_node.get_latest_edge_frame()
            processed_frame = self.ros_node.get_latest_processed_frame()

            is_processing = self.ros_node.processing_active
            current_processed_frame_available = (processed_frame is not None)
            # --- End Get Frames ---


            # --- Manage Frame Visibility (Pack/Unpack) ---
            if self.display_mode != last_display_mode:
                try:
                    if self.display_mode == "live":
                        print("Switching view to Live Feeds")
                        self.processed_display_frame.pack_forget()
                        self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
                    elif self.display_mode == "processed":
                        print("Switching view to Processed Image")
                        self.top_frame.pack_forget()
                        self.processed_display_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
                    last_display_mode = self.display_mode
                except tk.TclError as e:
                     self.ros_node.get_logger().warn(f"TclError during frame switching: {e}")
                     last_display_mode = None


            # --- Update Image Labels based on CURRENTLY VISIBLE frame ---
            try:
                if self.display_mode == "live":
                    # Update Webcam Feed
                    if webcam_frame is not None and self.webcam_label.winfo_exists():
                        try:
                            img_rgb = cv2.cvtColor(webcam_frame, cv2.COLOR_BGR2RGB)
                            pil_img = PILImage.fromarray(img_rgb)
                            img_tk = ImageTk.PhotoImage(image=pil_img)
                            self.webcam_label.configure(image=img_tk)
                            self.webcam_label.image = img_tk
                        except Exception as e:
                            self.ros_node.get_logger().error(f"Error updating webcam GUI: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                    elif self.webcam_label.winfo_exists():
                         pass # Leave last frame or clear: self.webcam_label.image = None

                    # Update Edge Feed
                    if edge_frame is not None and self.edge_label.winfo_exists():
                        try:
                            img_rgb_edge = cv2.cvtColor(edge_frame, cv2.COLOR_GRAY2RGB)
                            pil_img_edge = PILImage.fromarray(img_rgb_edge)
                            img_tk_edge = ImageTk.PhotoImage(image=pil_img_edge)
                            self.edge_label.configure(image=img_tk_edge)
                            self.edge_label.image = img_tk_edge
                        except Exception as e:
                             self.ros_node.get_logger().error(f"Error updating edge GUI: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                    elif self.edge_label.winfo_exists():
                         pass # Leave last frame or clear: self.edge_label.image = None

                elif self.display_mode == "processed":
                    # Update Processed Feed
                    if current_processed_frame_available and self.processed_label.winfo_exists():
                        try:
                            img_rgb_proc = cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2RGB)
                            pil_img_proc = PILImage.fromarray(img_rgb_proc)
                            img_tk_proc = ImageTk.PhotoImage(image=pil_img_proc)
                            self.processed_label.configure(image=img_tk_proc)
                            self.processed_label.image = img_tk_proc
                        except Exception as e:
                             self.ros_node.get_logger().error(f"Error updating processed GUI: {e}\n{traceback.format_exc()}", throttle_duration_sec=5.0)
                    elif not current_processed_frame_available and self.processed_label.winfo_exists():
                         self.processed_label.configure(image=None)
                         self.processed_label.image = None

            except tk.TclError as e:
                 self.ros_node.get_logger().warn(f"TclError during image label update: {e}")


            # --- State Transition: Check if processing finished ---
            if GoalStatus is not None and not is_processing and current_processed_frame_available and self.display_mode == "live" \
               and self.capture_button.cget('text') == "Capture Image":
                 print("Processing complete, switching to processed view.")
                 self.display_mode = "processed"
                 self.last_processed_frame_available = True
                 if self.capture_button.winfo_exists():
                     self.capture_button.config(text="Take Another", state=NORMAL)
                 if self.draw_button.winfo_exists():
                     self.draw_button.config(state=NORMAL)
                 if self.status_label.winfo_exists():
                     self.status_label.config(text="Processing complete. Image ready.")


            # --- Update Status Label and Button States (Refined Logic) ---
            if self.status_label.winfo_exists():
                current_status = self.status_label.cget("text")
                if is_processing:
                    if "Processing request..." not in current_status:
                         self.status_label.config(text="Processing request...")
                    if self.capture_button.winfo_exists() and self.capture_button.cget('state') == NORMAL:
                         self.capture_button.config(state=DISABLED)
                    if self.draw_button.winfo_exists() and self.draw_button.cget('state') == NORMAL:
                         self.draw_button.config(state=DISABLED)
                else:
                    if self.display_mode == "live":
                         if not any(msg in current_status for msg in ["complete", "Published", "Failed", "Ready for new capture"]):
                              status_parts = []
                              if webcam_frame is None: status_parts.append("Waiting Webcam")
                              if edge_frame is None: status_parts.append("Waiting Edge")
                              self.status_label.config(text=", ".join(status_parts) if status_parts else "Ready.")
                         if self.capture_button.winfo_exists() and self.capture_button.cget('state') == DISABLED:
                              self.capture_button.config(text="Capture Image", state=NORMAL)
                         if self.draw_button.winfo_exists() and self.draw_button.cget('state') == NORMAL:
                              self.draw_button.config(state=DISABLED)
                    elif self.display_mode == "processed":
                         if self.capture_button.winfo_exists() and self.capture_button.cget('state') == DISABLED:
                              self.capture_button.config(text="Take Another", state=NORMAL)
                         if self.draw_button.winfo_exists():
                              new_state = NORMAL if current_processed_frame_available else DISABLED
                              if self.draw_button.cget('state') != new_state:
                                   self.draw_button.config(state=new_state)

            self.last_processed_frame_available = current_processed_frame_available
            # --- End Update Status & Buttons ---

            time.sleep(0.05)

        print("GUI update loop finished.")


    def on_closing(self):
        """Handles main window closing."""
        print("Main window on_closing called.")
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.running = False
            if self.localisation_window and self.localisation_window.winfo_exists():
                 self.on_localisation_close()
            if self.settings_window and self.settings_window.winfo_exists():
                 self.on_settings_close()
            print("Attempting to destroy main window...")
            self.destroy()
            print("Main window destroyed.")
        else:
            print("Quit cancelled.")
    # --- End Closing Handler ---


# Main execution block
def main(args=None):
    if Img is None or GoalStatus is None:
        print("Critical Error: Img action or GoalStatus definition not found. Cannot run GUI.")
        print("Ensure 'img_prc_interface' package is built and sourced, and action_msgs is available.")
        sys.exit(1)

    print("Initializing rclpy...")
    rclpy.init(args=args)
    gui_node = None
    app = None
    ros_spin_thread = None
    main_exception = None

    try:
        print("Creating GuiNode...")
        gui_node = GuiNode()
        print("GuiNode created.")

        def spin_ros():
            node_name = gui_node.get_name() if gui_node else "gui_node (pre-init)"
            print(f"ROS spin thread started for {node_name}.")
            try:
                if gui_node:
                    rclpy.spin(gui_node)
                    if gui_node.context and gui_node.context.ok():
                         gui_node.get_logger().info("ROS spin finished normally.")
                    else:
                         print("ROS spin finished after context shutdown.")
                else:
                     print("Error: gui_node was None in spin_ros.")
            except Exception as e:
                log_msg = f"Exception in ROS spin thread: {e}\n{traceback.format_exc()}"
                if gui_node and gui_node.context and gui_node.context.ok():
                    gui_node.get_logger().error(log_msg)
                else:
                    print(f"Exception in ROS spin thread (context likely shutdown): {e}")
                    print(traceback.format_exc()) # Print traceback if logger failed
            finally:
                 print(f"ROS spin thread for {node_name} exiting.")

        ros_spin_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_spin_thread.start()
        print("ROS spin thread created and started.")

        print("Creating ActionViewerApp (Tkinter GUI)...")
        app = ActionViewerApp(ros_node=gui_node)
        print("ActionViewerApp created. Starting mainloop...")
        app.mainloop()
        print("Tkinter mainloop finished.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down.")
        main_exception = "KeyboardInterrupt"
        if app and app.winfo_exists():
            app.on_closing()
    except Exception as e:
        main_exception = e
        log_msg = f"Unhandled exception in main setup/mainloop: {e}\n{traceback.format_exc()}"
        if gui_node and rclpy.ok():
            gui_node.get_logger().fatal(log_msg)
        else:
            print(log_msg)
    finally:
        print("Starting final cleanup...")

        if app and hasattr(app, 'running'):
             if app.running:
                 print("Setting app.running to False.")
                 app.running = False
             if app.winfo_exists():
                  print("Force destroying lingering Tk window...")
                  app.destroy()

        if rclpy.ok():
             if gui_node:
                 print("Terminating external processes if running...")
                 if gui_node.canvas_localisation_process and gui_node.canvas_localisation_process.poll() is None:
                      print("Terminating canvas localisation process...")
                      gui_node.canvas_localisation_process.terminate()
                      try:
                           gui_node.canvas_localisation_process.wait(timeout=1.0)
                           print("Canvas localisation process terminated.")
                      except subprocess.TimeoutExpired:
                           print("Canvas localisation process kill required.")
                           gui_node.canvas_localisation_process.kill()
                 if gui_node.robot_calibration_process and gui_node.robot_calibration_process.poll() is None:
                      print("Terminating robot calibration process...")
                      gui_node.robot_calibration_process.terminate()
                      try:
                           gui_node.robot_calibration_process.wait(timeout=1.0)
                           print("Robot calibration process terminated.")
                      except subprocess.TimeoutExpired:
                           print("Robot calibration process kill required.")
                           gui_node.robot_calibration_process.kill()

                 print("Destroying GUI node...")
                 gui_node.destroy_node()
                 print("GUI node destroyed.")

             print("Shutting down rclpy...")
             rclpy.shutdown()
             print("rclpy shutdown complete.")
        else:
            print("rclpy context already invalid or node not created.")

        if ros_spin_thread and ros_spin_thread.is_alive():
             print("Waiting for ROS spin thread to join...")
             ros_spin_thread.join(timeout=2.0)
             if ros_spin_thread.is_alive():
                 print("Warning: ROS spin thread did not exit cleanly.")
             else:
                 print("ROS spin thread joined successfully.")
        else:
            print("ROS spin thread already finished or not started.")

        print(f"Cleanup finished. Main exception (if any): {main_exception}")
        sys.exit(0 if main_exception is None else 1)


if __name__ == '__main__':
    main()
