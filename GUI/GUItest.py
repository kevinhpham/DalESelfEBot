import PyQt5.QtWidgets as qtw
import PyQt5.QtGui as qtg

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
        my_button = qtw.QPushButton('Take a selfie!', clicked = lambda: press_it())
        self.layout().addWidget(my_button)

        def press_it():
            # add name to lable
            my_label.setText(f"{my_entry.text()} had sex with a dog")
            # Clear the entry box
            my_entry.setText("")
        
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