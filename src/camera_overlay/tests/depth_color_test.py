#!/usr/bin/python3
"""Metric colour mapping + synthetic ROS camera controls. No camera or motor hardware."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import Image

SCRIPT = Path(sys.argv.pop(1)) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'scripts/depth_color_viewer.py'
spec = importlib.util.spec_from_file_location('depth_colour', SCRIPT)
v = importlib.util.module_from_spec(spec);spec.loader.exec_module(v)


class ColourTest(unittest.TestCase):
    def test_fixed_metric_colours_and_invalid_black(self):
        a = np.array([[.1,2.5,0,np.nan,np.inf,-1,.05,10]],np.float32)
        c = v.colorize(a,.1,2.5)
        np.testing.assert_array_equal(c[0,0], [0,0,255])
        np.testing.assert_array_equal(c[0,1], [255,0,0])
        np.testing.assert_array_equal(c[0,2:6], np.zeros((4,3),np.uint8))
        np.testing.assert_array_equal(c[0,6], c[0,0]);np.testing.assert_array_equal(c[0,7],c[0,1])
        np.testing.assert_array_equal(v.colorize(np.array([[1.2]],np.float32),.1,2.5)[0,0],
                                      v.colorize(np.array([[.3,1.2,7]],np.float32),.1,2.5)[0,1])
        with self.assertRaises(ValueError):v.colorize(a,2,1)

    def test_control_uses_descriptor_ranges_and_manual_ae_gate(self):
        camera={v.NAMES[0]:{'value':False,'read_only':False},
                v.NAMES[1]:{'value':5000,'read_only':False,'min':100,'max':10000,'step':100}}
        self.assertEqual(v.validate_control({v.NAMES[1]:6000},camera),{v.NAMES[1]:6000})
        for body in [{v.NAMES[1]:6050},{v.NAMES[1]:0},{v.NAMES[1]:True},{'rgb_camera.exposure':100}]:
            with self.assertRaises(ValueError):v.validate_control(body,camera)
        camera[v.NAMES[0]]['value']=True
        with self.assertRaisesRegex(ValueError,'disable automatic'):v.validate_control({v.NAMES[1]:6000},camera)
        self.assertEqual(v.validate_control({v.NAMES[0]:False,v.NAMES[1]:6000},camera)[v.NAMES[1]],6000)

    def test_roi_excludes_invalid_and_reports_z(self):
        depth=np.ones((25,25),np.float32);depth[2,2]=0
        text=v.DepthViewer.measure(depth,2,2)
        self.assertIn('Z=invalid',text);self.assertIn('median=1.000 m',text)

    def test_http_controls_synthetic_camera_and_depth(self):
        # Override inherited robot-domain values before initializing any ROS entity.
        os.environ['ROS_DOMAIN_ID']='90';os.environ['ROS_LOCALHOST_ONLY']='1'
        rclpy.init()
        node=Node('camera',namespace='/depth_test')
        for name,value in zip(v.NAMES,[False,5000,16]):
            desc=ParameterDescriptor(description='synthetic camera control')
            if type(value) is int:
                desc.integer_range=[IntegerRange(from_value=100 if name==v.NAMES[1] else 0,
                                                to_value=10000 if name==v.NAMES[1] else 128,
                                                step=100 if name==v.NAMES[1] else 1)]
            node.declare_parameter(name,value,desc)
        calls=[]
        def validate(params):
            calls.append([(p.name,p.value) for p in params])
            reject=any(p.name==v.NAMES[1] and p.value==7000 for p in params)
            return SetParametersResult(successful=not reject,reason='synthetic sensor rejected 7000' if reject else '')
        node.add_on_set_parameters_callback(validate)
        pub=node.create_publisher(Image,'/depth_test/image',qos_profile_sensor_data)
        source=[1000,'16UC1']
        def publish():
            is_mm=source[1]=='16UC1';arr=np.full((48,64),source[0] if is_mm else source[0]/1000,np.uint16 if is_mm else np.float32)
            arr[0,0]=0
            msg=Image(width=64,height=48,encoding=source[1],step=64*(2 if is_mm else 4),data=arr.tobytes())
            msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg)
        timer=node.create_timer(.1,publish)
        executor=SingleThreadedExecutor();executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        proc=subprocess.Popen([sys.executable,'-s',str(SCRIPT),'--ros-args','-p',f'stream_debug_port:={port}',
            '-p','camera_node:=/depth_test/camera','-p','depth_topic:=/depth_test/image','-p','show_debug_view:=false'],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        base=f'http://127.0.0.1:{port}'
        def request(path,body=None):
            req=urllib.request.Request(base+path,data=json.dumps(body).encode() if body is not None else None,
                                       headers={'Content-Type':'application/json'})
            return urllib.request.urlopen(req,timeout=2)
        def status():
            with request('/status') as r:return json.load(r)
        def wait(predicate,reason,timeout=10):
            until=time.monotonic()+timeout
            while time.monotonic()<until:
                try:
                    if predicate():return
                except (OSError,KeyError):pass
                self.assertIsNone(proc.poll(),'viewer stopped unexpectedly')
                time.sleep(.08)
            self.fail(reason)
        try:
            wait(lambda:status().get('encoding')=='16UC1' and all('value' in status()['camera'].get(k,{}) for k in v.NAMES),'viewer never connected')
            self.assertEqual(calls,[],'viewer changed exposure without input')
            with request('/frame.jpg') as response:
                frame=int(response.headers['X-Frame-Id']);jpg=response.read()
            image=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(image.shape,(48,64,3))
            with request('/point',{'frame_id':frame,'x':20,'y':20}):pass
            wait(lambda:'Z=1.000 m' in status()['point'],'point measurement unavailable')
            # Ensure there is no outstanding periodic read when submitting controls.
            def submit(body):
                for _ in range(50):
                    if not status()['busy']:
                        with request('/control',body):pass
                        return
                    time.sleep(.03)
                self.fail('camera remained busy')
            submit({v.NAMES[1]:6000,v.NAMES[2]:20})
            wait(lambda:node.get_parameter(v.NAMES[1]).value==6000,'manual exposure did not reach fake sensor')
            wait(lambda:status()['camera'][v.NAMES[1]]['value']==6000,'readback not updated')
            submit({v.NAMES[1]:7000})
            wait(lambda:'synthetic sensor rejected' in status()['message'],'sensor rejection not shown')
            self.assertEqual(node.get_parameter(v.NAMES[1]).value,6000)
            submit({v.NAMES[0]:True})
            wait(lambda:status()['camera'][v.NAMES[0]]['value'] is True,'AE did not enable')
            with self.assertRaises(urllib.error.HTTPError) as caught:request('/control',{v.NAMES[1]:6000})
            self.assertEqual(caught.exception.code,400)
            source[1]='32FC1'
            wait(lambda:status()['encoding']=='32FC1','32FC1 depth unsupported')
            self.assertIn('Z=1.000 m',status()['point'])
            with request('/point',{'frame_id':1,'x':20,'y':20}):pass
            wait(lambda:'expired' in status()['message'],'stale frame input not rejected')
            with request('/') as r:self.assertIn(b'Apply manual values',r.read())
        finally:
            proc.terminate()
            try:output=proc.communicate(timeout=5)[0]
            except subprocess.TimeoutExpired:proc.kill();output=proc.communicate()[0]
            executor.shutdown(timeout_sec=2);thread.join(timeout=2)
            node.destroy_node();rclpy.try_shutdown()
            if proc.returncode not in (0,-15):print(output.decode(errors='replace'))


if __name__=='__main__':unittest.main()
