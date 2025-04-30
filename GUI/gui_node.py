#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty # Import the Empty message type
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tkinter as tk
from tkinter import messagebox, Toplevel, DISABLED, NORMAL # Import more constants
from PIL import Image as PILImage, ImageTk
import threading
import time
import numpy as np
import sys # Import sys for main function cleanup
import subprocess # Import subprocess to launch external nodes
import os # Import os for environment variables if needed

# Try to import the action definition
try:
    from img_prc_interface.action import Img
except ImportError:
    print("Error: Could not import Img action from img_prc_interface.action.")
    print("Ensure the img_prc_interface package is built and sourced.")
    Img = None # Set to None if import fails

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
            self.get_logger().error(f'Error in webcam_callback: {e}')

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.edge_frame_lock:
                self.latest_edge_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Edge CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in edge_callback: {e}')
    # --- End Callbacks ---

    # --- Getters for Frames ---
    def get_latest_webcam_frame(self):
        with self.webcam_frame_lock:
            return self.latest_webcam_frame

    def get_latest_edge_frame(self):
        with self.edge_frame_lock:
            return self.latest_edge_frame

    def get_latest_processed_frame(self):
        with self.processed_frame_lock:
            return self.latest_processed_frame
    # --- End Getters ---

    # --- Action Client Methods (send_process_image_goal, etc.) ---
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
        except Exception as e:
             self.get_logger().error(f"Error converting frame for action: {e}")
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
            self.get_logger().error(f'Exception while getting goal handle: {e}', exc_info=True)
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
        pass # Not used currently

    def get_result_callback(self, future):
        try:
            result_response = future.result()
            if result_response is None:
                self.get_logger().error("Future for get_result_async returned None.")
                return
            result = result_response.result
            status = result_response.status
            if result and result.processed_image:
                self.get_logger().info(f'Result received (status: {status}).')
                processed_cv_image = self.bridge.imgmsg_to_cv2(result.processed_image, desired_encoding='mono8')
                with self.processed_frame_lock:
                    self.latest_processed_frame = processed_cv_image
            else:
                self.get_logger().warn(f'Action finished with no processed image result (status: {status}).')
        except Exception as e:
             self.get_logger().error(f'Error processing action result: {e}', exc_info=True)
        finally:
            self.processing_active = False
    # --- End Action Client Methods ---

    # --- Publishing Methods ---
    def publish_processed_image(self):
        with self.processed_frame_lock:
            frame_to_publish = self.latest_processed_frame.copy() if self.latest_processed_frame is not None else None
        if frame_to_publish is None:
            self.get_logger().warn("No processed image available to publish.")
            return False
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_to_publish, encoding='mono8')
            self.pixel_map_publisher.publish(image_msg)
            self.get_logger().info(f"Published processed image to {self.pixel_map_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing processed frame: {e}")
            return False

    def publish_save_position(self):
        """Publishes an Empty message to /save_position."""
        try:
            msg = Empty()
            self.save_pos_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.save_pos_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish to /save_position: {e}")
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
            return False, "Error: 'ros2 run' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch localisation_node: {e}", exc_info=True)
            return False, f"Failed to launch node: {e}"

    def launch_robot_calibration_launchfile(self): # New method
        """Launches the UR calibration launch file."""
        if self.robot_calibration_process and self.robot_calibration_process.poll() is None:
            self.get_logger().warn("Robot calibration launch file might already be running.")
            return False, "Robot calibration might already be running."

        # Construct the command arguments carefully
        # NOTE: Adjust the target_filename path if needed for the user running the GUI
        target_file = "/home/jarred/git/DalESelfEBot/ur3_control/calibration/ur3e_calibration_1.yaml" # Make sure this path is correct
        command = [
            'ros2', 'launch', 'ur_calibration', 'calibration_correction.launch.py',
            'robot_ip:=192.168.0.191',
            f'target_filename:="{target_file}"' # Ensure quotes are handled correctly if path has spaces
        ]
        try:
            current_env = os.environ.copy()
            self.robot_calibration_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched robot calibration."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 launch' command not found. Is ROS 2 environment sourced correctly?")
            return False, "Error: 'ros2 launch' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot calibration: {e}", exc_info=True)
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
        self.geometry("300x300") # Increased height slightly for new label

        self.calibration_step = 0 # 0: BL, 1: BR, 2: TR, 3: TL, 4: Done
        self.corner_order = ['BL', 'BR', 'TR', 'TL'] # Order of calibration

        # --- Create Widgets ---
        main_frame = tk.Frame(self, padx=10, pady=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Use grid layout for buttons
        button_frame = tk.Frame(main_frame)
        button_frame.pack(pady=5) # Reduced padding above buttons

        self.buttons = {} # Dictionary to hold buttons

        # Top Row
        self.buttons['TL'] = tk.Button(button_frame, text="Top Left", width=10,
                                       command=lambda: self.on_corner_button_click('TL'))
        self.buttons['TL'].grid(row=0, column=0, padx=5, pady=5)

        self.buttons['TR'] = tk.Button(button_frame, text="Top Right", width=10,
                                       command=lambda: self.on_corner_button_click('TR'))
        self.buttons['TR'].grid(row=0, column=1, padx=5, pady=5)

        # Bottom Row
        self.buttons['BL'] = tk.Button(button_frame, text="Bottom Left", width=10,
                                       command=lambda: self.on_corner_button_click('BL'))
        self.buttons['BL'].grid(row=1, column=0, padx=5, pady=5)

        self.buttons['BR'] = tk.Button(button_frame, text="Bottom Right", width=10,
                                       command=lambda: self.on_corner_button_click('BR'))
        self.buttons['BR'].grid(row=1, column=1, padx=5, pady=5)

        # --- Robot Orientation Indicator (New) ---
        # Assuming robot faces the 'bottom' edge of the button grid
        orientation_label = tk.Label(main_frame, text="▲\nRobot", font=("Arial", 10))
        orientation_label.pack(pady=(5, 5)) # Padding top/bottom
        # --- End Robot Orientation Indicator ---

        # Status Label
        self.loc_status_label = tk.Label(main_frame, text="Click Bottom Left")
        self.loc_status_label.pack(pady=5)

        # Close Button
        close_button = tk.Button(main_frame, text="Close", command=self.close_window)
        close_button.pack(pady=10, side=tk.BOTTOM)
        # --- End Widgets ---

        # Set initial button states
        self.update_button_states()

        # Handle window closing via 'X' button
        self.protocol("WM_DELETE_WINDOW", self.close_window)

        # Make window appear on top
        self.lift()
        self.focus_force() # Grab focus

    def update_button_states(self):
        """Enables/disables buttons based on the current calibration step."""
        if self.calibration_step >= len(self.corner_order): # Calibration done
            expected_corner = None
            self.loc_status_label.config(text="Calibration Complete!")
        else:
            expected_corner = self.corner_order[self.calibration_step]
            # Make status message clearer
            corner_name = self.buttons[expected_corner]['text']
            self.loc_status_label.config(text=f"Move robot to {corner_name}\nand click button.")

        for corner_id, button in self.buttons.items():
            if corner_id == expected_corner:
                button.config(state=NORMAL)
            else:
                # Keep previously clicked buttons disabled
                button_step_index = self.corner_order.index(corner_id)
                if button_step_index < self.calibration_step:
                     button.config(state=DISABLED) # Keep disabled if already done
                else:
                     # Disable buttons not yet reached
                     button.config(state=DISABLED if corner_id != expected_corner else NORMAL)


    def on_corner_button_click(self, corner_id):
        """Handles clicks on the corner buttons."""
        if self.calibration_step >= len(self.corner_order):
            print("Calibration already complete.")
            return

        expected_corner = self.corner_order[self.calibration_step]

        if corner_id == expected_corner:
            print(f"Correct button clicked: {corner_id}")
            # Publish the /save_position message
            success = self.ros_node.publish_save_position()
            if success:
                # Move to next step
                self.calibration_step += 1
                self.update_button_states() # Update button enable states and label
            else:
                messagebox.showerror("Publish Error", "Failed to publish to /save_position topic.")
        else:
            print(f"Incorrect button clicked: {corner_id}. Expected: {expected_corner}")
            # Make warning message clearer
            expected_corner_name = self.buttons[expected_corner]['text']
            messagebox.showwarning("Incorrect Button", f"Please click the '{expected_corner_name}' button next.")

    def close_window(self):
        """Closes the localisation window and informs the main app."""
        print("Localisation window close requested.")
        # Call the main app's cleanup method for this window
        if self.master_app:
            self.master_app.on_localisation_close()
        # Destroy self (Toplevel window)
        # No need to call self.destroy() here as master_app.on_localisation_close should handle it


# --- Main GUI Application ---
class ActionViewerApp(tk.Tk):
    """
    Tkinter app displaying feeds, with Capture, Draw, and Settings buttons.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.settings_window = None # To keep track of the settings window instance
        self.localisation_window = None # To keep track of the localisation window
        self.title("DalESelfEBot GUI") # Simplified title
        self.geometry("1040x840")

        # --- Top Frame for Live Feeds ---
        top_frame = tk.Frame(self)
        top_frame.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)

        webcam_frame = tk.Frame(top_frame, bd=2, relief=tk.SUNKEN)
        webcam_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(webcam_frame, text="Webcam Feed").pack()
        self.webcam_label = tk.Label(webcam_frame)
        self.webcam_label.pack(fill=tk.BOTH, expand=True)

        edge_frame = tk.Frame(top_frame, bd=2, relief=tk.SUNKEN)
        edge_frame.pack(side=tk.RIGHT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(edge_frame, text="Edge Feed").pack()
        self.edge_label = tk.Label(edge_frame)
        self.edge_label.pack(fill=tk.BOTH, expand=True)

        # --- Bottom Frame for Processed Image and Controls ---
        bottom_frame = tk.Frame(self)
        bottom_frame.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)

        processed_frame = tk.Frame(bottom_frame, bd=2, relief=tk.SUNKEN)
        processed_frame.pack(side=tk.TOP, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(processed_frame, text="Processed Image (Result)").pack()
        self.processed_label = tk.Label(processed_frame)
        self.processed_label.pack(fill=tk.BOTH, expand=True)

        # Frame for Buttons and Status
        control_frame = tk.Frame(bottom_frame)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        # Capture Button
        self.capture_button = tk.Button(control_frame, text="Capture & Process", command=self.on_capture_button_click)
        self.capture_button.pack(side=tk.LEFT, padx=10)

        # Draw Button
        self.draw_button = tk.Button(control_frame, text="Draw", command=self.on_draw_button_click, state=DISABLED)
        self.draw_button.pack(side=tk.LEFT, padx=10)

        # Settings Button
        self.settings_button = tk.Button(control_frame, text="Settings", command=self.open_settings_window)
        self.settings_button.pack(side=tk.LEFT, padx=10)

        # Status Label
        self.status_label = tk.Label(control_frame, text="Waiting for feeds...")
        self.status_label.pack(side=tk.LEFT, padx=10)

        # --- App State ---
        self.running = True
        self.last_processed_frame_available = False

        # Start the update loop
        self.update_thread = threading.Thread(target=self.update_image_display, daemon=True)
        self.update_thread.start()
        print("GUI update thread started.")

        # Handle window closing event
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    # --- Window Management ---
    def open_settings_window(self):
        """Opens the Toplevel window for settings."""
        print("Settings button clicked.")
        if self.settings_window is not None and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return
        # Create settings window (if not already open)
        self.settings_window = Toplevel(self)
        self.settings_window.title("Robot Settings")
        self.settings_window.geometry("400x300") # Adjusted size for new button
        settings_frame = tk.Frame(self.settings_window, padx=10, pady=10)
        settings_frame.pack(fill=tk.BOTH, expand=True)

        # Localise Canvas Button (Renamed)
        localise_button = tk.Button(settings_frame, text="Localise Canvas", command=self.on_localise_canvas_click) # Renamed command
        localise_button.pack(pady=10)

        # Calibrate Robot Button (New)
        calibrate_robot_button = tk.Button(settings_frame, text="Calibrate Robot", command=self.on_calibrate_robot_click) # New command
        calibrate_robot_button.pack(pady=10)

        placeholder_label = tk.Label(settings_frame, text="(Other settings here...)")
        placeholder_label.pack(pady=10)
        close_button = tk.Button(settings_frame, text="Close", command=self.on_settings_close)
        close_button.pack(pady=10, side=tk.BOTTOM)
        self.settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
        self.settings_window.lift() # Bring to front

    def on_settings_close(self):
        """Callback when the settings window is closed."""
        if self.settings_window:
            print("Settings window closed.")
            self.settings_window.destroy()
            self.settings_window = None

    def open_localisation_window(self):
        """Opens the Toplevel window for localisation."""
        if self.localisation_window is not None and self.localisation_window.winfo_exists():
            self.localisation_window.lift()
            return
        # Create localisation window
        self.localisation_window = LocalisationWindow(self, self.ros_node)
        self.localisation_window.lift()

    def on_localisation_close(self):
        """Callback when the localisation window is closed."""
        if self.localisation_window:
            print("Localisation window closed.")
            self.localisation_window.destroy()
            self.localisation_window = None
    # --- End Window Management ---

    # --- Button Callbacks ---
    def on_localise_canvas_click(self): # Renamed callback
        """Callback for the Localise Canvas button *inside* the Settings window."""
        print("Localise Canvas button clicked.")
        # 1. Launch Jarred's node
        success_launch, msg_launch = self.ros_node.launch_canvas_localisation_node() # Use renamed method
        if not success_launch:
            messagebox.showerror("Localisation Error", f"Failed to launch node: {msg_launch}")
            return # Don't open the localisation window if launch fails

        # 2. Open the localisation button window
        self.open_localisation_window()
        messagebox.showinfo("Localisation", msg_launch + "\nLocalisation window opened.")

    def on_calibrate_robot_click(self): # New callback
        """Callback for the Calibrate Robot button."""
        print("Calibrate Robot button clicked.")
        success_launch, msg_launch = self.ros_node.launch_robot_calibration_launchfile()
        if success_launch:
            messagebox.showinfo("Robot Calibration", msg_launch)
        else:
            messagebox.showerror("Robot Calibration Error", msg_launch)

    def on_capture_button_click(self):
        """Callback for the capture button."""
        print("Capture button clicked.")
        self.status_label.config(text="Sending request...")
        self.capture_button.config(state=DISABLED)
        self.draw_button.config(state=DISABLED)
        self.last_processed_frame_available = False
        if self.processed_label.winfo_exists():
             self.processed_label.configure(image=None)

        success = self.ros_node.send_process_image_goal()
        if not success:
            if not self.ros_node.processing_active:
                 self.status_label.config(text="Failed to send request.")
            messagebox.showerror("Action Error", "Could not send processing request. Check logs or server status.")
            self.capture_button.config(state=NORMAL)

    def on_draw_button_click(self):
        """Callback for the Draw button."""
        print("Draw button clicked.")
        self.status_label.config(text="Publishing to /pixel_map...")
        success = self.ros_node.publish_processed_image()
        if success:
            self.status_label.config(text="Published to /pixel_map.")
        else:
            self.status_label.config(text="Failed to publish.")
            messagebox.showerror("Publish Error", "Could not publish processed image. Was one captured?")
    # --- End Button Callbacks ---

    # --- Display Update ---
    def update_image_display(self):
        """Periodically updates the image labels and button states."""
        while self.running and rclpy.ok():
            # --- Get Frames ---
            with self.ros_node.webcam_frame_lock:
                webcam_frame = self.ros_node.latest_webcam_frame.copy() if self.ros_node.latest_webcam_frame is not None else None
            with self.ros_node.edge_frame_lock:
                edge_frame = self.ros_node.latest_edge_frame.copy() if self.ros_node.latest_edge_frame is not None else None
            with self.ros_node.processed_frame_lock:
                processed_frame = self.ros_node.latest_processed_frame.copy() if self.ros_node.latest_processed_frame is not None else None

            is_processing = self.ros_node.processing_active
            status_parts = []
            self.last_processed_frame_available = (processed_frame is not None)
            # --- End Get Frames ---

            # --- Update GUI Elements ---
            if not self.winfo_exists(): break

            # Update Webcam Feed
            webcam_update_success = False
            if webcam_frame is not None:
                try:
                    img_rgb = cv2.cvtColor(webcam_frame, cv2.COLOR_BGR2RGB)
                    pil_img = PILImage.fromarray(img_rgb)
                    img_tk = ImageTk.PhotoImage(image=pil_img)
                    self.webcam_label.configure(image=img_tk)
                    self.webcam_label.image = img_tk
                    webcam_update_success = True
                except Exception as e:
                    if self.webcam_label.winfo_exists():
                        self.ros_node.get_logger().error(f"Error updating webcam GUI: {e}", throttle_duration_sec=5.0)
                        status_parts.append("Webcam Err")
            if not webcam_update_success: status_parts.append("Wait Webcam")

            # Update Edge Feed
            edge_update_success = False
            if edge_frame is not None:
                try:
                    img_rgb_edge = cv2.cvtColor(edge_frame, cv2.COLOR_GRAY2RGB)
                    pil_img_edge = PILImage.fromarray(img_rgb_edge)
                    img_tk_edge = ImageTk.PhotoImage(image=pil_img_edge)
                    self.edge_label.configure(image=img_tk_edge)
                    self.edge_label.image = img_tk_edge
                    edge_update_success = True
                except Exception as e:
                     if self.edge_label.winfo_exists():
                        self.ros_node.get_logger().error(f"Error updating edge GUI: {e}", throttle_duration_sec=5.0)
                        status_parts.append("Edge Err")
            if not edge_update_success: status_parts.append("Wait Edge")

            # Update Processed Feed
            processed_update_success = False
            if processed_frame is not None:
                try:
                    img_rgb_proc = cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2RGB)
                    pil_img_proc = PILImage.fromarray(img_rgb_proc)
                    img_tk_proc = ImageTk.PhotoImage(image=pil_img_proc)
                    self.processed_label.configure(image=img_tk_proc)
                    self.processed_label.image = img_tk_proc
                    processed_update_success = True
                except Exception as e:
                     if self.processed_label.winfo_exists():
                        self.ros_node.get_logger().error(f"Error updating processed GUI: {e}", throttle_duration_sec=5.0)
                        status_parts.append("Proc Err")
            elif self.processed_label.winfo_exists():
                    if processed_frame is None:
                        self.processed_label.configure(image=None)

            # Update Status Label and Button States
            if self.status_label.winfo_exists():
                if is_processing:
                    self.status_label.config(text="Processing request...")
                    if self.capture_button.winfo_exists(): self.capture_button.config(state=DISABLED)
                    if self.draw_button.winfo_exists(): self.draw_button.config(state=DISABLED)
                else:
                    current_status = self.status_label.cget("text")
                    if not any(msg in current_status for msg in ["Processing", "Publishing", "Published", "Failed"]):
                        if not status_parts: self.status_label.config(text="Ready.")
                        else:
                            wait_msgs = [s for s in status_parts if "Wait" in s]
                            err_msgs = [s for s in status_parts if "Err" in s]
                            display_status = ""
                            if len(wait_msgs) == 2: display_status = "Waiting for feeds"
                            elif wait_msgs: display_status = wait_msgs[0]
                            if err_msgs: display_status += (", " if display_status else "") + ", ".join(err_msgs)
                            self.status_label.config(text=display_status if display_status else "Status Unknown")
                    if self.capture_button.winfo_exists(): self.capture_button.config(state=NORMAL)
                    if self.draw_button.winfo_exists():
                        if self.last_processed_frame_available: self.draw_button.config(state=NORMAL)
                        else: self.draw_button.config(state=DISABLED)
            # --- End Update GUI Elements ---

            # --- Loop Timing ---
            time.sleep(0.05)
            # --- End Loop Timing ---

        print("GUI update loop finished.")

    # --- Closing Handler ---
    def on_closing(self):
        """Handles main window closing."""
        print("Main window on_closing called.")
        self.running = False
        # Close secondary windows if they exist
        self.on_localisation_close()
        self.on_settings_close()
        print("Attempting to destroy main window...")
        self.after(50, self.destroy)
    # --- End Closing Handler ---

# Main execution block
def main(args=None):
    if Img is None:
        print("Exiting: Img action definition not found.")
        sys.exit(1)

    rclpy.init(args=args)
    gui_node = None
    app = None
    ros_spin_thread = None
    main_exception = None

    try:
        gui_node = GuiNode()
        print("GuiNode created.")

        def spin_ros():
            print("ROS spin thread started.")
            try:
                rclpy.spin(gui_node)
                if gui_node and gui_node.context.ok():
                     gui_node.get_logger().info("ROS spin finished normally.")
            except Exception as e:
                if gui_node and gui_node.context.ok():
                    gui_node.get_logger().error(f"Exception in ROS spin thread: {e}", exc_info=True)
                else:
                    print(f"Exception in ROS spin thread after context shutdown: {e}")
            finally:
                 print("ROS spin thread exiting.")

        ros_spin_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_spin_thread.start()
        print("ROS spin thread created and started.")

        app = ActionViewerApp(ros_node=gui_node)
        print("ActionViewerApp created. Starting mainloop...")
        app.mainloop()
        print("Tkinter mainloop finished.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down.")
        main_exception = "KeyboardInterrupt"
    except Exception as e:
        main_exception = e
        print(f"Unhandled exception in main setup/mainloop: {e}")
        if gui_node:
            gui_node.get_logger().fatal(f"Unhandled exception in main setup/mainloop: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before node creation: {e}")
    finally:
        print("Starting final cleanup...")
        if app and app.running:
             print("Setting app.running to False.")
             app.running = False

        if rclpy.ok():
             if gui_node:
                 print("Destroying GUI node...")
                 # Terminate calibration processes if running
                 if gui_node.canvas_localisation_process and gui_node.canvas_localisation_process.poll() is None:
                      print("Terminating canvas localisation process...")
                      gui_node.canvas_localisation_process.terminate()
                      try: gui_node.canvas_localisation_process.wait(timeout=1.0)
                      except subprocess.TimeoutExpired: gui_node.canvas_localisation_process.kill()
                 if gui_node.robot_calibration_process and gui_node.robot_calibration_process.poll() is None:
                      print("Terminating robot calibration process...")
                      gui_node.robot_calibration_process.terminate()
                      try: gui_node.robot_calibration_process.wait(timeout=1.0)
                      except subprocess.TimeoutExpired: gui_node.robot_calibration_process.kill()

                 gui_node.destroy_node()
             print("Shutting down rclpy...")
             rclpy.shutdown()
             print("rclpy shutdown called.")
        else:
            print("rclpy context already invalid.")

        if ros_spin_thread and ros_spin_thread.is_alive():
             print("Waiting for ROS spin thread to join...")
             ros_spin_thread.join(timeout=2.0)
             if ros_spin_thread.is_alive(): print("ROS spin thread did not exit cleanly.")
             else: print("ROS spin thread joined.")

        print(f"Cleanup finished. Main exception (if any): {main_exception}")
        sys.exit(0 if main_exception is None else 1)


if __name__ == '__main__':
    main()
