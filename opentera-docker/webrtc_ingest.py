import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import asyncio
import threading
import aiohttp
from aiortc import RTCPeerConnection, RTCSessionDescription

class MediaMtxWebRTCSubscriber(Node):
    def __init__(self):
        super().__init__('webrtc_subscriber')
        
        # 1. Declare parameters with safe fallback defaults
        self.declare_parameter('whep_url', 'http://192.168.8.101:8889/cam/whep')
        self.declare_parameter('ros_topic', 'cameras/hires/raw')
        
        # 2. Retrieve the runtime values
        self.whep_url = self.get_parameter('whep_url').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        
        # 3. Dynamically set up the publisher using the parameter value
        self.publisher_ = self.create_publisher(Image, self.ros_topic, 10)
        self.compressed_pub_ = self.create_publisher(CompressedImage, f"{self.ros_topic}/compressed", 10)

        self.bridge = CvBridge()
        
        self.get_logger().info(f"Target ROS Topic: {self.ros_topic}")
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

        # Create the WebRTC Offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # Send the Offer to MediaMTX via WHEP
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
                
        # Set the Answer to establish the connection
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

                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = "jpeg"

                success, encode_msg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if success: 
                    comp_msg.data = encode_msg.tobytes()
                    self.compressed_pub_.publish(comp_msg)

            except Exception as e:
                self.get_logger().error(f"Error processing WebRTC frame: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    
    # The parameters are now resolved entirely inside the class constructor
    node = MediaMtxWebRTCSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()