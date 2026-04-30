import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import asyncio
import threading
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription

class MediaMtxWebRTCSubscriber(Node):
    def __init__(self, whep_url):
        super().__init__('webrtc_subscriber')
        self.publisher_ = self.create_publisher(Image, 'cameras/arm/raw', 10)
        self.bridge = CvBridge()
        self.whep_url = whep_url
        
        self.get_logger().info(f"Connecting to WebRTC WHEP endpoint: {self.whep_url}")
        
        # Start the asyncio loop in a separate thread so it doesn't block ROS2 spin
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._start_asyncio_loop, daemon=True)
        self.thread.start()

    def _start_asyncio_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run_webrtc())

    async def run_webrtc(self):
        pc = RTCPeerConnection()
        pc.addTransceiver("video", direction="recvonly")

        @pc.on("track")
        def on_track(track):
            self.get_logger().info(f"WebRTC track received: {track.kind}")
            if track.kind == "video":
                asyncio.ensure_future(self.consume_track(track))

        # 1. Create the WebRTC Offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # 2. Send the Offer to MediaMTX via WHEP
        async with aiohttp.ClientSession() as session:
            async with session.post(
                self.whep_url,
                data=pc.localDescription.sdp,
                headers={"Content-Type": "application/sdp"}
            ) as response:
                if response.status not in [200, 201]:
                    self.get_logger().error(f"WHEP request failed with status: {response.status}")
                    return
                answer_sdp = await response.text()
                
        # 3. Set the Answer to establish the connection
        answer = RTCSessionDescription(sdp=answer_sdp, type="answer")
        await pc.setRemoteDescription(answer)

        # Keep the connection alive while the ROS2 node is running
        while rclpy.ok():
            await asyncio.sleep(1)
            
        await pc.close()

    async def consume_track(self, track):
        while rclpy.ok():
            try:
                # Receive the frame from MediaMTX
                frame = await track.recv()
                
                # Convert the PyAV frame to a standard OpenCV Numpy array
                img = frame.to_ndarray(format="bgr24")
                
                # Convert the OpenCV image to a ROS2 message and publish
                msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "webrtc_camera"
                
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error processing WebRTC frame: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    
    # MediaMTX WHEP read endpoint format: http://<ip>:8889/<stream_name>/whep
    # Assuming MediaMTX is on the host and Docker is using --network host
    whep_url = "http://192.168.8.101:8889/cam0/whep" 
    
    node = MediaMtxWebRTCSubscriber(whep_url)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
