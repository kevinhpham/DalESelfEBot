import sys
import cv2 # OpenCV for camera interaction
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QSizePolicy)
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QTimer

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot Arm Drawing Control")
        # Set an initial size, you can adjust this v
        self.setGeometry(100, 100, 1000, 750) 

        # --- Main Layout ---
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget) # Main vertical layout

        # --- Video Feed Section ---
        self.video_feed_label = QLabel("Connecting to Webcam...")
        self.video_feed_label.setAlignment(Qt.AlignCenter)
        # Make the video label expandable
        self.video_feed_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) 
        self.video_feed_label.setStyleSheet("border: 1px solid black;") # Add border for visibility
        self.main_layout.addWidget(self.video_feed_label)

        # --- Control Buttons & Status ---
        controls_layout = QHBoxLayout() # Horizontal layout for buttons/status
        self.take_photo_button = QPushButton("Take Photo")
        self.status_label = QLabel("Status: Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        # Make status label take up available space
        # Modified line:
        self.video_feed_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred) 

        controls_layout.addWidget(self.take_photo_button)
        controls_layout.addWidget(self.status_label)
        self.main_layout.addLayout(controls_layout)

        # --- Image Display Section ---
        image_display_layout = QHBoxLayout() # Horizontal layout for images

        # Placeholder for Captured Photo
        self.captured_photo_label = QLabel("Captured Photo")
        self.captured_photo_label.setAlignment(Qt.AlignCenter)
        self.captured_photo_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored) # Let layout handle size
        self.captured_photo_label.setMinimumSize(320, 240) # Ensure a minimum size
        self.captured_photo_label.setStyleSheet("border: 1px solid gray;")
        image_display_layout.addWidget(self.captured_photo_label)

        # Placeholder for Processed Photo
        self.processed_photo_label = QLabel("Processed Photo (Line Drawing)")
        self.processed_photo_label.setAlignment(Qt.AlignCenter)
        self.processed_photo_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored) # Let layout handle size
        self.processed_photo_label.setMinimumSize(320, 240) # Ensure a minimum size
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
        # Place settings button in its own layout for better spacing control if needed
        settings_layout = QHBoxLayout()
        settings_layout.addStretch() # Push button to the right
        self.settings_button = QPushButton("Settings")
        settings_layout.addWidget(self.settings_button)
        self.main_layout.addLayout(settings_layout)


        # --- Button Connections (Placeholder Functions) ---
        self.take_photo_button.clicked.connect(self.take_photo)
        self.process_photo_button.clicked.connect(self.process_photo)
        self.start_drawing_button.clicked.connect(self.start_drawing)
        self.settings_button.clicked.connect(self.open_settings)

        # --- Webcam Setup ---
        self.video_capture = None
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_frame)
        self.initialize_camera()


    def initialize_camera(self):
        """Tries to initialize the webcam."""
        # 0 is usually the default built-in webcam. 
        # If you have multiple cameras, you might need to change this index (1, 2, etc.)
        index_to_try = 0 
        self.video_capture = cv2.VideoCapture(index_to_try) 

        if not self.video_capture.isOpened():
            self.video_feed_label.setText(f"Error: Cannot open webcam index {index_to_try}")
            print(f"Error: Cannot open webcam index {index_to_try}")
            self.video_capture = None # Ensure it's None if failed
            return

        # Start the timer to fetch frames
        # Update rate (milliseconds), e.g., 33ms ~ 30fps
        self.camera_timer.start(33) 
        print("Webcam initialized successfully.")


    def update_frame(self):
        """Reads a frame from the webcam and displays it."""
        if not self.video_capture or not self.video_capture.isOpened():
            return # Don't do anything if camera isn't working

        ret, frame = self.video_capture.read()
        if ret:
            # OpenCV reads frames in BGR format, PyQt needs RGB
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Get frame dimensions
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Convert to QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Convert QImage to QPixmap to display in QLabel
            # Scale the image to fit the label while maintaining aspect ratio
            qt_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.video_feed_label.size(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation # Use smooth scaling
            )
            
            # Set the pixmap on the label
            self.video_feed_label.setPixmap(qt_pixmap)


    # --- Placeholder Button Functions ---
    def take_photo(self):
        self.status_label.setText("Status: Taking Photo...")
        print("Take Photo button clicked (functionality not implemented yet)")
        # Later: Capture current frame, display in captured_photo_label

    def process_photo(self):
        self.status_label.setText("Status: Processing Photo...")
        print("Process Photo button clicked (functionality not implemented yet)")
        # Later: Process image from captured_photo_label, display in processed_photo_label

    def start_drawing(self):
        self.status_label.setText("Status: Starting Drawing...")
        print("Start Drawing button clicked (functionality not implemented yet)")
        # Later: Send drawing path (from processed photo) to ROS2 backend

    def open_settings(self):
        self.status_label.setText("Status: Opening Settings...")
        print("Settings button clicked (functionality not implemented yet)")
        # Later: Open a new QDialog or QWidget for settings


    def closeEvent(self, event):
        """Ensure camera is released when the window closes."""
        print("Closing application...")
        if self.video_capture and self.video_capture.isOpened():
            self.video_capture.release()
            print("Webcam released.")
        self.camera_timer.stop()
        event.accept() # Accept the close event


# --- Application Execution ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())