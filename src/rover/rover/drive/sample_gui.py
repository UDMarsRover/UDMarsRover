import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QSlider, QTableWidget, QTableWidgetItem
from PyQt6.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Sample GUI")

        # Create the main layout
        layout = QVBoxLayout()

        # Create a slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(50)
        self.slider.valueChanged.connect(self.slider_value_changed)
        layout.addWidget(self.slider)

        # Create a table
        self.table = QTableWidget(5, 3)  # 5 rows, 3 columns
        self.table.setHorizontalHeaderLabels(["Column 1", "Column 2", "Column 3"])
        for i in range(5):
            for j in range(3):
                self.table.setItem(i, j, QTableWidgetItem(f"Item {i+1},{j+1}"))
        layout.addWidget(self.table)

        # Set the central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def slider_value_changed(self, value):
        print(f"Slider value: {value}")

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec())