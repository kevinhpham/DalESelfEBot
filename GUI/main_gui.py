import sys
import cv2 # OpenCV for camera interaction
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QSizePolicy)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer # Keep QTimer for local preview if used

# --- ROS2 Imports ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty # Import the Empty message type
from threading import Thread
import signal
# Add other ROS imports later if needed (e.g., custom services, CvBridge)
# --- End ROS2 Imports ---

class MainWindow(QMainWindow):
    # Add signals here later if needed for thread-safe GUI updates from ROS callbacks
    # update_ui_signal = pyqtSignal(object, object)

    def __init__(self, node): # Accept node instance
        super().__init__()
        self.node = node # Store the node instance

        self.setWindowTitle("Robot Arm Drawing Control")
        self.setGeometry(100, 100, 1000, 750)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # --- Video Feed Section (Container Approach with Fixed Height) ---
        self.video_container = QWidget()
        self.video_container.setFixedHeight(480) # Fixed height for stability
        self.video_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.main_layout.addWidget(self.video_container)

        container_layout = QVBoxLayout(self.video_container)
        container_layout.setContentsMargins(0, 0, 0, 0)

        self.video_feed_label = QLabel("Connecting to Webcam...")
        self.video_feed_label.setAlignment(Qt.AlignCenter)
        self.video_feed_label.setStyleSheet("border: 1px solid black;")
        container_layout.addWidget(self.video_feed_label)

        # --- Control Buttons & Status ---
        controls_layout = QHBoxLayout()
        self.take_photo_button = QPushButton("Take Photo")
        self.status_label = QLabel("Status: Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        controls_layout.addWidget(self.take_photo_button)
        controls_layout.addWidget(self.status_label)
        self.main_layout.addLayout(controls_layout)

        # --- Image Display Section ---
        image_display_layout = QHBoxLayout()
        self.captured_photo_label = QLabel("Captured Photo")
        self.captured_photo_label.setAlignment(Qt.AlignCenter)
        self.captured_photo_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.captured_photo_label.setMinimumSize(320, 240)
        self.captured_photo_label.setStyleSheet("border: 1px solid gray;")
        image_display_layout.addWidget(self.captured_photo_label)

        self.processed_photo_label = QLabel("Processed Photo (Line Drawing)")
        self.processed_photo_label.setAlignment(Qt.AlignCenter)
        self.processed_photo_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.processed_photo_label.setMinimumSize(320, 240)
        self.processed_photo_label.setStyleSheet("border: 1px solid gray;")
        image_display_layout.addWidget(self.processed_photo_label)
        self.main_layout.addLayout(image_display_layout)

        # --- Action Buttons Section ---
        action_buttons_layout = QHBoxLayout()
        self.process_photo_button = QPushButton("Process Photo")
        self.start_drawing_button = QPushButton("Start Drawing")
        action_buttons_layout.addWidget(self.process_photo_button)
        action_buttons_layout.addWidget(self.start_drawing_button)
        self.main_layout.addLayout(action_buttons_layout)

        # --- Settings Button ---
        settings_layout = QHBoxLayout()
        settings_layout.addStretch()
        self.settings_button = QPushButton("Settings")
        settings_layout.addWidget(self.settings_button)
        self.main_layout.addLayout(settings_layout)

        # --- ROS2 Publisher/Client Setup ---
        # Publisher for the start drawing button
        self.continue_publisher = self.node.create_publisher(
            Empty, # Message type
            '/continue_execution', # Topic name
            10 # QoS history depth
        )
        self.node.get_logger().info(f"Publisher created for topic '/continue_execution'")

        # Placeholders for other clients (add later)
        self.capture_client = None
        self.process_client = None
        self.bridge = None # CvBridge placeholder
        # self.update_ui_signal.connect(self.update_image_labels) # Connect signal later

        # --- Button Connections ---
        self.take_photo_button.clicked.connect(self.take_photo)
        self.process_photo_button.clicked.connect(self.process_photo)
        self.start_drawing_button.clicked.connect(self.start_drawing) # Connects to the updated method
        self.settings_button.clicked.connect(self.open_settings)

        # --- Webcam Setup ---
        # Keep this for local preview if needed
        self.video_capture = None
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_frame)
        self.initialize_camera() # Comment out if not using local preview


    def initialize_camera(self):
        """Tries to initialize the webcam FOR LOCAL PREVIEW ONLY."""
        index_to_try = 0
        self.video_capture = cv2.VideoCapture(index_to_try)
        if not self.video_capture.isOpened():
            self.video_feed_label.setText(f"Error: Cannot open webcam index {index_to_try} for preview")
            print(f"Error: Cannot open webcam index {index_to_try} for preview")
            self.video_capture = None
            return
        self.camera_timer.start(33)
        print("Local preview webcam initialized successfully.")


    def update_frame(self):
        """Reads a frame FOR LOCAL PREVIEW ONLY and displays it."""
        if not self.video_capture or not self.video_capture.isOpened():
            return
        ret, frame = self.video_capture.read()
        if ret:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            qt_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.video_container.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.video_feed_label.setPixmap(qt_pixmap)


    # === Button Callback Placeholders ===

    def take_photo(self):
        """ Placeholder: Called when 'Take Photo' button is clicked. """
        self.status_label.setText("Status: 'Take Photo' clicked (ROS2 service call needed)")
        print("Take Photo button clicked - Placeholder for ROS2 service call")
        # --- KEV'S ROS2 CLIENT CALL LOGIC FOR CAPTURE GOES HERE ---
        # Needs self.capture_client initialized, async call, callback, signal/slot update


    def process_photo(self):
        """ Placeholder: Called when 'Process Photo' button is clicked. """
        self.status_label.setText("Status: 'Process Photo' clicked (Logic TBD)")
        print("Process Photo button clicked - Placeholder")
        # --- KEV'S ROS2 CLIENT CALL LOGIC FOR PROCESSING (if needed) GOES HERE ---


    def start_drawing(self):
        """
        Called when 'Start Drawing' button is clicked.
        Publishes an Empty message to the /continue_execution topic.
        """
        self.status_label.setText("Status: Sending 'Continue Execution' signal...")
        print("Start Drawing button clicked - Publishing to /continue_execution")

        if self.continue_publisher is not None:
            msg = Empty()
            try:
                self.continue_publisher.publish(msg)
                self.node.get_logger().info('Published Empty message to /continue_execution')
                self.status_label.setText("Status: 'Continue' signal sent.")
            except Exception as e:
                self.node.get_logger().error(f'Failed to publish to /continue_execution: {e}')
                self.status_label.setText("Status: Error sending signal.")
        else:
            self.node.get_logger().error('Continue publisher is not initialized!')
            self.status_label.setText("Status: Error - Publisher not ready.")


    def open_settings(self):
        """ Placeholder: Called when 'Settings' button is clicked. """
        self.status_label.setText("Status: 'Settings' clicked (GUI logic needed)")
        print("Settings button clicked - Placeholder for settings window")
        # --- SETTINGS WINDOW GUI LOGIC GOES HERE ---


    # === ROS2 Response Handling Placeholders ===
    # Define callback methods here later (e.g., capture_response_callback)
    # Remember they are called from the ROS thread and need signals/slots for GUI updates.
    # Define slot method(s) here later for thread-safe GUI updates
    # def update_image_labels(self, captured_pixmap, processed_pixmap): ...


    def closeEvent(self, event):
        """Clean up resources when the window closes."""
        print("Close event triggered...")
        # Stop internal camera timer if used
        if hasattr(self, 'camera_timer') and self.camera_timer.isActive():
             self.camera_timer.stop()
        if hasattr(self, 'video_capture') and self.video_capture and self.video_capture.isOpened():
             self.video_capture.release()
             print("Local preview webcam released.")
        # Node shutdown is handled after app.exec_ finishes
        event.accept()

    def shutdown_ros_node(self):
        """Destroys the ROS2 node."""
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info("Destroying ROS2 node...")
            self.node.destroy_node()


# === ROS2 Spinning Function ===
def spin_ros_node(node):
    """ Target function for the ROS2 spinning thread """
    print("Starting ROS2 spin thread")
    try:
        rclpy.spin(node)
    except (rclpy.exceptions.RCLError, KeyboardInterrupt) as e:
         # Handle cases where spin is interrupted externally or due to shutdown
         print(f"rclpy.spin stopped: {e}")
    except Exception as e:
        if node and rclpy.ok(): # Log error only if node is still valid
             node.get_logger().error(f"rclpy.spin failed unexpectedly: {e}")
    finally:
        print("ROS2 spin thread finished.")


# --- Application Execution ---
if __name__ == "__main__":
    rclpy.init() # Initialize ROS2
    ros_node = None # Initialize variable
    ros_thread = None # Initialize variable
    app = None
    main_window = None
    app_exit_code = 1 # Default error code

    try:
        ros_node = rclpy.create_node('robot_drawing_gui_node_main') # Use unique name temporarily
        
        # Start ROS2 spinning in a separate thread
        ros_thread = Thread(target=spin_ros_node, args=(ros_node,), daemon=True)
        ros_thread.start()

        app = QApplication(sys.argv)
        # Pass the node to the main window instance
        main_window = MainWindow(node=ros_node) 
        main_window.show()

        # Allow Ctrl+C to be caught by QApplication        
        signal.signal(signal.SIGINT, signal.SIG_DFL) 

        app_exit_code = app.exec_() # Start Qt event loop

    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down.")
        app_exit_code = 0
    except Exception as e:
        print(f"An error occurred during application execution: {e}")
        if ros_node: # Use logger if node exists
             ros_node.get_logger().fatal(f"GUI Error: {e}")
        app_exit_code = 1
    finally:
        print("Application exiting...")
        # Cleanly destroy the node if it exists
        if main_window is not None:
            main_window.shutdown_ros_node()
        elif ros_node is not None: # If window failed but node exists
            ros_node.destroy_node()

        # Shutdown ROS2 context
        if rclpy.ok():
            print("Shutting down rclpy...")
            rclpy.shutdown()
        
        # Ensure thread finishes - though daemon=True might make join unnecessary
        # if ros_thread is not None and ros_thread.is_alive():
        #     print("Waiting for ROS thread to join...")
        #     ros_thread.join(timeout=1.0) # Wait briefly

        print(f"Exiting with code {app_exit_code}")
        sys.exit(app_exit_code)