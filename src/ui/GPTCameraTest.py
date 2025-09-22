import cv2
from Capture import WebRTCVideoCapture as webRTC

sdp_offer = "192.168.0.114:8889/cam"

cap = webRTC(sdp_offer)
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue

    cv2.imshow("WebRTC Video", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()