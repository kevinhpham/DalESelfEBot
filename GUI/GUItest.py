import PyQt5.QtWidgets as qtw
import PyQt5.QtGui as qtg
from PyQt5.QtCore import Qt

class MainWindow(qtw.QWidget):
    def __init__(self):
        super().__init__()
        # Add a title
        self.setWindowTitle('Dal-E Self-E BOT (test GUI)')

        # Set the layout
        self.setLayout(qtw.QVBoxLayout())

        # Create a label
        my_label = qtw.QLabel('Welcome to Dal-E Self-E BOT')
        # Change the font size of the label
        my_label.setFont(qtg.QFont("Comic Sans MS", 69))
        # Add the label to the layout
        self.layout().addWidget(my_label)

        # Create an entry box
        my_entry = qtw.QLineEdit()
        my_entry.setObjectName("name_field")
        my_entry.setText("Enter your name")
        self.layout().addWidget(my_entry)

        # Create a button
        my_button = qtw.QPushButton('Take Photo', clicked = lambda: press_take())
        self.layout().addWidget(my_button)

        # Create a button
        my_button = qtw.QPushButton('Process Photo', clicked = lambda: press_process())
        self.layout().addWidget(my_button)

        # Create a button
        my_button = qtw.QPushButton('Start Drawing', clicked = lambda: press_draw())
        self.layout().addWidget(my_button)

        # Add a placeholder for webcam feed
        webcam_placeholder = qtw.QLabel('Webcam Feed Placeholder')
        webcam_placeholder.setFixedSize(640, 480)  # Set a fixed size for the webcam feed
        webcam_placeholder.setStyleSheet("background-color: black; color: white;")
        webcam_placeholder.setAlignment(Qt.AlignCenter)
        self.layout().addWidget(webcam_placeholder)

        def press_take():
            # add name to lable
            my_label.setText(f"{my_entry.text()} Photo taken!")
            # Clear the entry box
            my_entry.setText("")

        def press_process():
            # add name to lable
            my_label.setText(f"{my_entry.text()} Photo processing!")
            # Clear the entry box
            my_entry.setText("")

        def press_draw():
            # add name to lable
            my_label.setText(f"{my_entry.text()} Photo drawing!")
            # Clear the entry box
            my_entry.setText("")

        # Add a status bar
        status_bar = qtw.QStatusBar()
        status_bar.showMessage("Ready")
        self.layout().addWidget(status_bar)
        
        # Show the app
        self.show()

app = qtw.QApplication([])
mw = MainWindow()

# Run the application's main loop
app.exec_()


"""
        self.setWindowTitle('Hello World')
        self.setLayout(qtw.QVBoxLayout())
        self.label = qtw.QLabel('Hello World')
        self.layout().addWidget(self.label)
        self.button = qtw.QPushButton('Press me!', clicked = self.clicked)
        self.layout().addWidget(self.button)
        self.show()"
"""