import asyncio
import threading
import numpy as np
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack

class WebRTCVideoCapture:
    def __init__(self, sdp_offer: str, offer_type: str = "offer"):
        self.frame = None
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._start, args=(sdp_offer, offer_type))
        self.thread.start()

    def _start(self, sdp_offer, offer_type):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._run(sdp_offer, offer_type))

    async def _run(self, sdp_offer, offer_type):
        pc = RTCPeerConnection()

        @pc.on("track")
        def on_track(track):
            if track.kind == "video":
                asyncio.ensure_future(self._receive_video(track))

        offer = RTCSessionDescription(sdp=sdp_offer, type=offer_type)
        await pc.setRemoteDescription(offer)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # print("\n📥 Your SDP Answer (send this back to signaling peer):\n")
        # print(pc.localDescription.sdp)

        # Keep running
        await asyncio.Event().wait()

    async def _receive_video(self, track: MediaStreamTrack):
        while True:
            frame = await track.recv()
            img = frame.to_ndarray(format="bgr24")
            self.frame = img

    def read(self):
        return True, self.frame.copy() if self.frame is not None else None

    def isOpened(self):
        return self.frame is not None

    def release(self):
        self.loop.stop()
        self.thread.join()
