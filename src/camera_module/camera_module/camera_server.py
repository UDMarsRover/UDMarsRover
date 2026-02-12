from picamera2 import Picamera2
import cv2
import cv2.aruco as aruco
from flask import Flask, Response, request
import threading
import time
import numpy as np
from rclpy.node import Node

app = Flask(__name__)

# Resolution presets
OUTPUT_RESOLUTIONS = {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "4k": (3840, 2160)
}

ZOOM_LEVELS = [1.0, 1.5, 2.0, 3.0, 4.0]

# Camera native resolutions
RPI_CAM_3_WIDE_RES = (4608, 2592)
RPI_HQ_CAM_RES = (4056, 3040)

INPUT_RESOLUTION_SCALE = 0.5

DEFAULT_OUTPUT_SETTINGS = {
    "resolution": "720p",
    "zoom": 1.0
}

# Shared camera data structure
initial_output_res = OUTPUT_RESOLUTIONS[DEFAULT_OUTPUT_SETTINGS["resolution"]]
initial_frame = np.zeros((initial_output_res[1], initial_output_res[0], 3), dtype=np.uint8)
#dict storing camera info for each of the cameras, incudling most recent frame, lock, ect
latest_camera_data = {
    0: {
        "frame": initial_frame,
        "lock": threading.Lock(),
        "output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "picam2": None,
    },
    1: {
        "frame": initial_frame,
        "lock": threading.Lock(),
        "output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "picam2": None,
    }
}
"""
This function aims to read the morse code apart of the competition. 
@param camera_id is the id of the picam being used 
@returns 
"""
def read_morse_from_camera(camera_id):
    #TODO Need to figure out the camera fps (dots/dashes could and probably will last multiple frames)
    morse_result = ""
    #pattern will be fed into current_morse_pattern, then checked against db of patterns 
    #once a pattern is detected, it will be appended to the result, and the array will clear 
    current_morse_pattern = []
    #a single, quick flash of light 
    DOT = 0 
    # - is a light flash around 3x longer than a dot 
    DASH = 1
    #A pause indicates a new letter 
    PAUSE = 2

    #Create a dictionary to compare current_morse_pattern to 
    MORSE_ALPHABET = {
        (DOT, DASH): "A", 
        (DASH, DOT, DOT, DOT): "B", 
        (DASH, DOT, DASH, DOT): "C",
        (DASH, DOT, DOT): "D", 
        (DOT): "E", 
        (DOT, DOT, DASH, DOT): "F",
        (DASH, DASH, DOT): "G",
        (DOT, DOT, DOT, DOT): "H",
        (DOT, DOT): "I", 
        (DOT, DASH, DASH, DASH): "J",
        (DASH, DOT, DASH): "K",
        (DOT, DASH, DOT, DOT): "L",
        (DASH, DASH): "M",
        (DASH, DOT): "N",
        (DASH, DASH, DASH): "O",
        (DOT, DASH, DASH, DOT): "P",
        (DASH, DASH, DOT, DASH): "Q",
        (DOT, DASH, DOT): "R",
        (DOT, DOT, DOT): "S",
        (DASH): "T",
        (DOT, DOT, DASH): "U",
        (DOT, DOT, DOT, DASH): "V",
        (DOT, DASH, DASH): "W",
        (DASH, DOT, DOT, DASH): "X",
        (DASH, DOT, DASH, DASH): "Y",
        (DASH, DASH, DOT, DOT): "Z",
        #-----------NUMBERS------------
        (DASH, DASH, DASH, DASH, DASH): "0",
        (DOT, DASH, DASH, DASH, DASH): "1",
        (DOT, DOT, DASH, DASH, DASH): "2",
        (DOT, DOT, DOT, DASH, DASH): "3",
        (DOT, DOT, DOT, DOT, DASH): "4",
        (DOT, DOT, DOT, DOT, DOT): "5",
        (DASH, DOT, DOT, DOT, DOT): "6",
        (DASH, DASH, DOT, DOT, DOT): "7",
        (DASH, DASH, DASH, DOT, DOT): "8",
        (DASH, DASH, DASH, DASH, DOT): "9"
    }
    while True:
        with latest_camera_data[camera_id]["lock"]:
            frame = latest_camera_data[camera_id]["frame"]
        
        if frame is not None:
            # 1. Analyze the 'frame' for a bright light (OpenCV)

            # 2. Determine if it's a Dot or Dash based on duration
            # 3. If a full letter is found, return/print it
            pass
        
def capture_and_process_frames(camera_id):
    print(f"Starting capture thread for camera {camera_id}...")
    picam2 = None
    try:
        if camera_id == 0:
            base_resolution = RPI_CAM_3_WIDE_RES
        elif camera_id == 1:
            base_resolution = RPI_HQ_CAM_RES
        else:
            print(f"Unknown camera ID {camera_id}. Exiting thread.")
            return

        scaled_width = int(base_resolution[0] * INPUT_RESOLUTION_SCALE)
        scaled_height = int(base_resolution[1] * INPUT_RESOLUTION_SCALE)
        sensor_capture_resolution = (scaled_width, scaled_height)

        picam2 = Picamera2(camera_id)
        config = picam2.create_preview_configuration(
            main={"size": sensor_capture_resolution, "format": "XRGB8888"}
        )
        picam2.configure(config)
        picam2.start()

        latest_camera_data[camera_id]["picam2"] = picam2
        sensor_width, sensor_height = sensor_capture_resolution

        print(f"Camera {camera_id} sensor configured to {sensor_width}x{sensor_height}. Starting capture loop.")

        while True:
            full_frame = picam2.capture_array()

            with latest_camera_data[camera_id]["lock"]:
                output_res_str = latest_camera_data[camera_id]["output_res_str"]
                zoom_level = latest_camera_data[camera_id]["zoom_level"]

            # Output width presets
            OUTPUT_WIDTHS = {
                "480p": 640,
                "720p": 1280,
                "1080p": 1920,
                "4k": 3840
            }
            output_width = OUTPUT_WIDTHS[output_res_str]
            native_aspect = sensor_width / sensor_height
            output_height = int(output_width / native_aspect)

            # Calculate crop size based on zoom and native aspect ratio
            crop_w = int(sensor_width / zoom_level)
            crop_h = int(sensor_height / zoom_level)
            crop_aspect = crop_w / crop_h

            # Adjust crop to match native aspect ratio
            if abs(crop_aspect - native_aspect) > 0.01:
                # Crop width or height to match native aspect
                if crop_aspect > native_aspect:
                    new_crop_w = int(crop_h * native_aspect)
                    new_crop_h = crop_h
                else:
                    new_crop_w = crop_w
                    new_crop_h = int(crop_w / native_aspect)
            else:
                new_crop_w = crop_w
                new_crop_h = crop_h

            start_x = (sensor_width - new_crop_w) // 2
            start_y = (sensor_height - new_crop_h) // 2

            cropped_frame = full_frame[start_y:start_y + new_crop_h,
                                       start_x:start_x + new_crop_w]

            processed_frame = cv2.resize(cropped_frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
            processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)

            with latest_camera_data[camera_id]["lock"]:
                latest_camera_data[camera_id]["frame"] = processed_frame

            time.sleep(1.0 / 24.0)

    except Exception as e:
        print(f"Capture thread for camera {camera_id} error: {e}")
        latest_camera_data[camera_id]["picam2"] = None
        time.sleep(5)
    finally:
        if picam2 and picam2.started:
            picam2.stop()
        print(f"Camera {camera_id}: Thread exiting.")
        latest_camera_data[camera_id]["picam2"] = None

def generate_frames(camera_id, aruco_enabled=False):
    # Initialize ArUco detector only once per generator
    if aruco_enabled:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()

    while True:
        with latest_camera_data[camera_id]["lock"]:
            frame = latest_camera_data[camera_id]["frame"].copy()
        if frame is None:
            time.sleep(0.1)
            continue

        if aruco_enabled:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None and len(ids) > 0:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)

        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ret:
            time.sleep(0.1)
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(1.0 / 15.0)


@app.route('/stream/<int:camera_id>')
def stream_feed(camera_id):
    #check if camera data is being found and functional 
    if camera_id not in latest_camera_data:
        #return https 400 
        return "Invalid camera ID. Use 0 or 1.", 400
    if latest_camera_data[camera_id]["picam2"] is None:
        return f"Camera {camera_id} is not available.", 503
    #request current resolution and zoom
    requested_resolution = request.args.get('resolution', DEFAULT_OUTPUT_SETTINGS["resolution"]).lower()
    requested_zoom_str = request.args.get('zoom', str(DEFAULT_OUTPUT_SETTINGS["zoom"]))

    aruco_enabled = request.args.get('aruco', 'false').lower() == 'true'

    if requested_resolution not in OUTPUT_RESOLUTIONS:
        return f"Invalid resolution. Choose from: {', '.join(OUTPUT_RESOLUTIONS.keys())}", 400


    try:
        requested_zoom = float(requested_zoom_str)
        zoom_min = min(ZOOM_LEVELS)
        zoom_max = max(ZOOM_LEVELS)
        if not (zoom_min <= requested_zoom <= zoom_max):
            return f"Invalid zoom level. Must be between {zoom_min} and {zoom_max}.", 400
    except ValueError:
        return f"Invalid zoom value: '{requested_zoom_str}'. Must be a number.", 400

    with latest_camera_data[camera_id]["lock"]:
        latest_camera_data[camera_id]["output_res_str"] = requested_resolution
        latest_camera_data[camera_id]["zoom_level"] = requested_zoom
        print(f"Camera {camera_id} settings updated: Resolution={requested_resolution}, Zoom={requested_zoom}, ArUco={aruco_enabled}")

    return Response(generate_frames(camera_id, aruco_enabled),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def root():
    res_options = ''.join([f'<option value="{r}">{r}</option>' for r in OUTPUT_RESOLUTIONS.keys()])
    zoom_min = min(ZOOM_LEVELS)
    zoom_max = max(ZOOM_LEVELS)
    zoom_step = 0.1
    return f"""
    <html>
    <head>
        <title>Dual Camera Stream</title>
        <style>
            body {{ font-family: Arial, sans-serif; background: #222; color: #eee; }}
            .controls {{ margin-bottom: 20px; }}
            .video-container {{ display: flex; justify-content: center; align-items: center; }}
            #stream {{ border: 2px solid #444; border-radius: 8px; background: #111; }}
            label, select, input {{ margin-right: 10px; }}
            .switch-btn {{ margin-left: 10px; padding: 5px 10px; }}
        </style>
    </head>
    <body>
        <h1>Dual Camera Stream</h1>
        <div class="controls">
            <label for="camera">Camera:</label>
            <select id="camera">
                <option value="0">Camera 0</option>
                <option value="1">Camera 1</option>
            </select>
            <label for="resolution">Resolution:</label>
            <select id="resolution">{res_options}</select>
            <label for="zoom">Zoom:</label>
            <input type="range" id="zoom" min="{zoom_min}" max="{zoom_max}" step="{zoom_step}" value="1.0" />
            <span id="zoom-value">1.0</span>
            <button class="switch-btn" onclick="switchCamera()">Switch Camera</button>
        </div>
        <div class="video-container">
            <img id="stream" src="/stream/0?resolution=720p&zoom=1.0" width="640" height="480" />
        </div>
        <script>
            var cameraSel = document.getElementById('camera');
            var resSel = document.getElementById('resolution');
            var zoomSlider = document.getElementById('zoom');
            var zoomVal = document.getElementById('zoom-value');
            var streamImg = document.getElementById('stream');
            var currentCamera = 0;

            function updateStream() {{
                var cam = cameraSel.value;
                var res = resSel.value;
                var zoom = parseFloat(zoomSlider.value).toFixed(1);
                zoomVal.textContent = zoom;
                streamImg.src = '/stream/' + cam + '?resolution=' + res + '&zoom=' + zoom;
                currentCamera = cam;
            }}

            cameraSel.addEventListener('change', updateStream);
            resSel.addEventListener('change', updateStream);
            zoomSlider.addEventListener('input', updateStream);

            function switchCamera() {{
                cameraSel.value = currentCamera == 0 ? '1' : '0';
                updateStream();
            }}
        </script>
    </body>
    </html>
    """

def main():
    camera_threads = []
    for i in range(len(latest_camera_data)):
        t = threading.Thread(target=capture_and_process_frames, args=(i,))
        t.daemon = True
        camera_threads.append(t)
        t.start()

    try:
        print("Starting Flask app...")
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Shutting down...")
        for cam_id in latest_camera_data:
            if latest_camera_data[cam_id]["picam2"]:
                latest_camera_data[cam_id]["picam2"].stop()
        print("All cameras stopped.")

if __name__ == '__main__':
    main()
