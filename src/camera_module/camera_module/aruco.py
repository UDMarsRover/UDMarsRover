import cv2

def main():
    # Initialize the webcam. '0' is usually the default camera.
    cap = cv2.VideoCapture('/dev/video0')

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Can't receive frame (stream end?). Exiting ...")
                break

            # Display the resulting frame
            cv2.imshow('Webcam Feed', frame)

            # Press 'q' on the keyboard to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()