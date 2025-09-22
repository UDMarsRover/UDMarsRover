from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QPixmap, QImage
from PIL import Image, ImageDraw
import json

'''
PyQt6 class (extends QWidget) for the UI minimap
'''
class Map(QWidget):
    def __init__(self):
        super().__init__()

        self.image_label = QLabel()

        # Creating buttons
        self.pin_window = QPushButton("Open Pins")
        self.pin_window.clicked.connect(self.open_pin_window)
        self.pin_save = QPushButton("Save Pins")
        self.pin_save.clicked.connect(self.save_pins)

        # Adding everything to the layout
        self.layout = QGridLayout()
        self.layout.addWidget(self.image_label, 0,0, 3,3)
        self.layout.addWidget(self.pin_window, 3,1)
        self.layout.addWidget(self.pin_save, 3,2)

        self.setLayout(self.layout)
        
        #Initializing pins, loading them from a json file
        self.pins = []
        self.load_pins()

        self.set_image()

        self.show()

        #Sets up 
        self.timer = QTimer()
        self.timer.timeout.connect(self.set_image)
        self.timer.start(5000)
    
    '''
    Sets the image 
    '''

    def set_image(self):
        #This loads the image (change the path to where it is in the run computer)
        path = r"C:\Users\weave\Documents\Programming\PythonScripts\KLTestImage.jpg"
        #This is the path to the map with the pins overlaid
        pin_path = r"C:\Users\weave\Documents\Programming\PythonScripts\KLTestImage_withPins.jpg"

        image = Image.open(path)
        draw = ImageDraw.Draw(image)

        #Place pins at the stored locations
        pin_radius = 10
        for pin in self.pins:
            x = self.dms_to_decimal(pin.lat)
            y = self.dms_to_decimal(pin.lon)
            draw.circle(self.to_pixels(x,y), pin_radius, "red")
        #Get Rover Lat and Lon (Decimal) and add rover icon here

        ICON_WIDTH = 40
        ICON_HEIGHT = 40

        icon = Image.open("rover_icon.png")

        icon.resize((ICON_WIDTH,ICON_HEIGHT))
        #Add rotation here
        position = self.to_pixels(self.dms_to_decimal("39°44'16.79\"N") - ICON_WIDTH/2, self.dms_to_decimal("84°10'35.00\"W") - ICON_HEIGHT/2)
        icon.rotate(angle=45, center=(ICON_HEIGHT/2, ICON_WIDTH/2))
        image.paste(icon, position)

        image.save(pin_path)
        image = QImage(pin_path)

        # Resize to the window size
        self.new_width = 600
        self.new_height = 450
        scaled_image = image.scaled(self.new_width, self.new_height)

        # Finalizing setting the image
        pixmap = QPixmap.fromImage(scaled_image)
        self.image_label.setPixmap(pixmap)
        self.show()
    

    '''
    Converts pixel coordinates to latitude and longitude
    Params: latitude, longitude (IN DECIMAL)
    Return: integer touple (x pixel, y pixel)
    '''
    def to_pixels(self, lat, lon):
        #The coordinates for the upper left and bottom righthand corners of the map
        self.coordsUL = self.dms_to_decimal("39°44'13.39\"N"), self.dms_to_decimal("84°10'42.79\"W")
        self.coordsBR = self.dms_to_decimal("39°44'19.77\"N"), self.dms_to_decimal("84°10'28.04\"W")

        #This assumes a 1080p resolution
        self.widthP = 1920
        self.heightP = 1080

        #sets up a ratio of how far in the x direction the pin is in relation to the screen
        #Then, mutliplying by the screen width and height gives you the pixel coordinates
        self.yPixel = self.heightP * (lat - self.coordsUL[0]) / (self.coordsBR[0] - self.coordsUL[0])
        self.xPixel = self.widthP * (self.coordsUL[1]- lon) / (self.coordsUL[1] - self.coordsBR[1])

        return int(self.xPixel), int(self.yPixel)
    
    '''
    Places a pin on the rover's current location. (Still needs to be connected to gps coords)
    '''
    def place_pin_current_location(self):
        #add lat and long coords input
        self.lat = "39°44'16.79\"N"
        self.lon = "84°10'35.00\"W"

        self.pins.append(Pin(self.lat, self.lon, "Placeholder"))
        self.set_image()
        self.update_pins()
        #just pins with numerical labels?

    '''
    Uses the input from the pinwindow to place a pin at a given 
    '''
    def place_picked_pin(self):
        #How are we going to type the degree character?
        input = self.pin_select.toPlainText()
        i = input.find(",")
        x = float(input[:i])
        y = float(input[i+2:])
        self.pins.append( Pin(x, y, "Placeholder") )
        self.update_pins()

    def open_pin_window(self):
        self.window = QWidget()
        self.window.local_pin = QPushButton("Place Pin On Current Location")
        self.window.local_pin.clicked.connect(self.place_pin_current_location)
        self.window.pick_pin = QPushButton("Place Pin By Coordinate")
        self.window.label = QLabel("(With a comma and a space in between lat and lon. ', ')")
        self.window.pick_pin.clicked.connect(self.place_picked_pin)
        self.window.pin_select = QTextEdit()

        self.window.layout = QGridLayout()


        self.window.text_out = QTextEdit()

        self.window.text_out.setReadOnly(True)

        self.window.layout.addWidget(self.window.text_out,0,0, 3,1)
        self.window.layout.addWidget(self.window.local_pin, 1,1)
        self.window.layout.addWidget(self.window.pick_pin, 1,2)
        self.window.layout.addWidget(self.window.label, 0,2)
        self.window.layout.addWidget(self.window.pin_select, 2,2)

        self.window.setLayout(self.window.layout)

        self.update_pins()
        
        self.window.show()

    def update_pins(self):
        self.window.text_out.clear()
        for pin in self.pins:
            self.window.text_out.append("(" + pin.lat + ", " + pin.lon + ")")

    def save_pins(self):
        self.pins_dicts = [pin.to_dict() for pin in self.pins]
        with open("pin_data.json", 'w') as f:
            json.dump(self.pins_dicts, f, indent=4)

    def load_pins(self):
        try:
            with open('pin_data.json', 'r') as f:
                self.pins_dicts = json.load(f)
                for data in self.pins_dicts:     
                    self.pins.append(Pin.from_dict(Pin, data))
        except Exception as e:
            print("There was an error reading the file: " + str(e))

    '''
    Converts DMS coordinates to decimal
    Params: dms (String), the dms lat/lon coordinate
    Returns: double decimal degrees
    '''
    def dms_to_decimal(self, dms_str):
        # Split the string by degree, minute, and second symbols
        dms_str.strip()
        direction = dms_str[-1]
        dms_str = dms_str[:-1]  # Remove the direction character (E, W, N, S)
        
        # Separate degrees, minutes, and seconds
        degrees, minutes, seconds = 0, 0, 0
        if '°' in dms_str:
            degrees = float(dms_str.split('°')[0])
            minutes_seconds = dms_str.split('°')[1]
            if "'" in minutes_seconds:
                minutes = float(minutes_seconds.split("'")[0])
                if '"' in minutes_seconds:
                    seconds = float(minutes_seconds.split("'")[1].replace('"', ''))
        
        # Convert to decimal
        decimal_degrees = degrees + (minutes / 60) + (seconds / 3600)
        
        # Apply negative sign for W and S directions
        if direction in ['W', 'S']:
            decimal_degrees = -decimal_degrees
        
        return decimal_degrees
    
    '''
    Converts decimal to dms coordinates
    Params: decimal coordinate
    Returns: String, dms lat/lon coordinate

    '''
    def decimal_to_dms(deg, is_latitude):
        direction = "N" if deg >= 0 else "S" if is_latitude else "E" if deg >= 0 else "W"
        
        deg = abs(deg)
        degrees = int(deg)
        minutes_float = (deg - degrees) * 60
        minutes = int(minutes_float)
        seconds = (minutes_float - minutes) * 60
        
        return f"{degrees}°{minutes}'{seconds:.2f}\" {direction}"

'''
A pin object for storing and retrieving from JSON
(Latitude and Longitude are stored in DMS, not decimal)
'''
class Pin():
    def __init__(self, x, y, t):
        self.lat = x
        self.lon = y
        self.timeStamp = t
    def to_dict(self):
        return {"Lattitude" : self.lat, "Longitude":self.lon, "Time": self.timeStamp}

    def from_dict(cls, data):
        return cls(data["Lattitude"], data["Longitude"], data["Time"])
    
    def toString(self):
        return str(self.lat) + " " + str(self.lon) + " " + str(self.timeStamp)