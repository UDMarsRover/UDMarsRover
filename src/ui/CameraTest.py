import cv2
import sys
from PyQt6.QtCore import QUrl
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QApplication, QWidget, QBoxLayout, QMainWindow
from PyQt6.QtWebEngineWidgets import QWebEngineView
from Capture import WebRTCVideoCapture as webRTC
# from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QLineEdit


# Now unneccessary template camera class
class Camera(QWidget):
    def __init__(self):
        super().__init__()


        self.cap = cv2.VideoCapture("192.168.0.114:8889/cam")
        self.cap = cv2.VideoCapture(0)

        self.image_label = QLabel()
        self.layout = QBoxLayout(QBoxLayout.Direction.TopToBottom)
        self.setLayout(self.layout)

        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update)
        # self.timer.start(30)


    def update(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(frame, frame.shape[1], frame.shape[0], QImage.Format.Format_RGB888)

            #Scales the image so it fits better
            self.new_width = 600
            self.new_height = 450
            scaled_image = image.scaled(self.new_width, self.new_height)

            # Convert the QImage to a QPixmap so PyQt6 can read it
            pixmap = QPixmap.fromImage(scaled_image)
            
            
            
            self.image_label.setPixmap(pixmap)
            
            self.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    cam = Camera()

    url = QUrl("https://www.wikipedia.org/")      #QUrl("192.168.0.114:8889/cam")
    browser = QWebEngineView()
    browser.setUrl(url)

    window = QMainWindow()

    central_widget = QWidget()  # The main content widget
    window.setCentralWidget(browser)
    window.layout = QBoxLayout(QBoxLayout.Direction.TopToBottom)
    
    window.layout.addWidget(cam)
    
    window.show()

    app.exec()