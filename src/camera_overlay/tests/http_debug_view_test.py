#!/usr/bin/python3
"""Synthetic RGB-D over ROS and HTTP only. No camera, model, or motor hardware."""
import os
import json
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


class HttpViewTest(unittest.TestCase):
    def test_headless_click_measures_the_displayed_pair_not_newest_depth(self):
        rclpy.init(args=[])
        publisher=Node("synthetic_overlay_camera")
        pubs=[publisher.create_publisher(message,topic,qos_profile_sensor_data) for message,topic in (
            (Image,"/camera/camera/color/image_raw"),
            (Image,"/camera/camera/aligned_depth_to_color/image_raw"),
            (CameraInfo,"/camera/camera/color/camera_info"))]
        stop=threading.Event()
        depth_mm=[1000]
        def publish():
            frame=0
            while not stop.is_set():
                stamp=publisher.get_clock().now().to_msg()
                color=Image(width=640,height=480,encoding="bgr8",step=1920)
                pixels=np.full((480,640,3),90,dtype=np.uint8)
                pixels[400:410,400:410]=frame%200+20
                color.data=pixels.tobytes()
                depth=Image(width=640,height=480,encoding="16UC1",step=1280)
                depth.data=np.full((480,640),depth_mm[0],dtype=np.uint16).tobytes()
                info=CameraInfo(width=640,height=480)
                info.k=[600.,0.,320.,0.,600.,240.,0.,0.,1.]
                for message,pub in zip((color,depth,info),pubs):
                    message.header.stamp=stamp
                    message.header.frame_id="synthetic_color_optical_frame"
                    pub.publish(message)
                frame+=1
                stop.wait(.07)
        thread=threading.Thread(target=publish,daemon=True)
        with socket.socket() as socket_:
            socket_.bind(("127.0.0.1",0))
            port=socket_.getsockname()[1]
        executable=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[3]/"install/camera_overlay/lib/camera_overlay/click_depth_node"
        env=dict(os.environ)
        env.pop("DISPLAY",None)
        env.pop("WAYLAND_DISPLAY",None)
        proc=subprocess.Popen([str(executable),"--ros-args","-p","show_debug_view:=false",
            "-p","stream_debug_view:=true","-p",f"stream_debug_port:={port}"],
            env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        stream=None
        thread.start()
        try:
            deadline=time.monotonic()+10
            while True:
                try:
                    stream=urllib.request.urlopen(f"http://127.0.0.1:{port}/stream.mjpg",timeout=3)
                    break
                except OSError:
                    if time.monotonic()>deadline:
                        self.fail("test server did not start")
                    time.sleep(.05)
            frame_id=None
            while time.monotonic()<deadline:
                headers={}
                while True:
                    line=stream.readline()
                    if not line: self.fail("stream disconnected")
                    if line==b"\r\n": break
                    if b":" in line:
                        key,value=line.decode().split(":",1)
                        headers[key.lower()]=value.strip()
                data=stream.read(int(headers["content-length"]))
                stream.read(2)
                image=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
                if image is not None and image[420,420,0]>50:
                    frame_id=headers["x-frame-id"]
                    break
            self.assertIsNotNone(frame_id,"no synthetic RGB frame arrived")
            self.assertEqual(image.shape,(480,640,3))
            # New frames now have different depth. The click still refers to the
            # older displayed RGB-D snapshot retained by the adapter.
            depth_mm[0]=3000
            time.sleep(.3)
            body=f"click {frame_id} 0 320 240".encode()
            request=urllib.request.Request(f"http://127.0.0.1:{port}/input",data=body,method="POST")
            with urllib.request.urlopen(request,timeout=3) as response:
                self.assertEqual(response.status,202)
            logs=b""
            deadline=time.monotonic()+4
            while time.monotonic()<deadline and b"median Z=1.000000" not in logs:
                if select.select([proc.stdout],[],[],.1)[0]:
                    logs+=os.read(proc.stdout.fileno(),65536)
            self.assertIn(b"median Z=1.000000",logs,logs.decode(errors="replace"))
            self.assertNotIn(b"median Z=3.000000",logs)
            def stats():
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/stats",timeout=3) as response:
                    return json.load(response)
            time.sleep(.2)
            encoded=stats()["encoded_frames"]
            time.sleep(.25)
            self.assertEqual(stats()["encoded_frames"],encoded,"frozen image was repeatedly encoded")
            stream.close()
            stream=None
            self.assertIsNone(proc.poll(),"viewer disconnect terminated node")
            # A frozen image must also appear after a new browser connects,
            # without forcing repeated encoding of the same screen.
            stream=urllib.request.urlopen(f"http://127.0.0.1:{port}/stream.mjpg",timeout=3)
            header=b""
            while not header.endswith(b"\r\n\r\n"):
                header+=stream.read(1)
            self.assertIn(b"X-Frame-Id:",header)
        finally:
            if stream is not None: stream.close()
            stop.set()
            thread.join(timeout=2)
            if proc.poll() is None:
                proc.terminate()
            try: proc.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()
                self.fail("test node shutdown hung")
            publisher.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    # The binary argument belongs to this harness, not unittest.
    unittest.main(argv=[sys.argv[0]])
