import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2
import time
import sys # Added for graceful exit

# Initialize the Picamera2 object
picam2 = Picamera2()
# Configure for a smaller, faster resolution for detection
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2) # Allow camera to warm up

# Define the ArUco detector
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
aruco_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, aruco_params)

print("Starting Headless ArUco detection. Press Ctrl+C to stop.")

try:
    while True:
        # Get frame from the camera
        frame = picam2.capture_array()
        # Convert to grayscale (detection works best on grayscale)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- ArUco Detection ---
        corners, ids, rejected = detector.detectMarkers(gray_frame)

        # Print ID and corners if markers are found
        if ids is not None and len(ids) > 0:
            print(f"\n--- Marker Found ---")
            for i in range(len(ids)):
                marker_id = ids[i][0]
                # Extract the pixel coordinates of the four corners (top-left, top-right, bottom-right, bottom-left)
                corner_points = corners[i][0].astype(int)
                
                print(f"ID: {marker_id}")
                print(f"Corners (Pixel Coords):\n{corner_points}")

        # Add a small delay to avoid overwhelming the CPU and the terminal output
        time.sleep(0.1) 

except KeyboardInterrupt:
    # This block handles Ctrl+C gracefully
    print("\nArUco detection stopped by user.")
    
finally:
    picam2.close()
    sys.exit(0)