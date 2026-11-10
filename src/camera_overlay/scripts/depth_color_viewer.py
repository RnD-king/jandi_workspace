#!/usr/bin/python3 -s
"""Depth colour view and explicit camera-only controls; never opens a device.
HTTP workers only exchange snapshots/queued input. ROS services run on the node thread.
Exposure changes are temporary. Edit the driver rs_launch.py for startup values.
"""
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.srv import DescribeParameters, GetParameters, SetParameters
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

NAMES = ('depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.gain')


def colorize(depth, minimum, maximum):
    """Fixed metric scale: valid out-of-range values saturate, invalid pixels stay black."""
    if not math.isfinite(minimum) or not math.isfinite(maximum) or not 0 < minimum < maximum:
        raise ValueError('distance range requires 0 < min < max')
    valid = np.isfinite(depth) & (depth > 0)
    ratio = np.zeros(depth.shape, dtype=np.float32)
    ratio[valid] = np.clip((depth[valid] - minimum) / (maximum - minimum), 0, 1)
    hsv = np.zeros((*depth.shape, 3), dtype=np.uint8)
    hsv[:, :, 0] = np.rint(120 * ratio).astype(np.uint8)
    hsv[:, :, 1] = 255
    hsv[:, :, 2] = valid.astype(np.uint8) * 255
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def validate_control(body, camera):
    if not isinstance(body, dict) or not body or set(body) - set(NAMES):
        raise ValueError('only depth AE / exposure / gain may be changed')
    if NAMES[0] in body and type(body[NAMES[0]]) is not bool:
        raise ValueError('AE must be boolean')
    for name, value in body.items():
        item = camera.get(name, {})
        if 'value' not in item or item.get('read_only', True):
            raise ValueError(name + ': unavailable or read-only')
        if name != NAMES[0]:
            if type(value) is not int:
                raise ValueError(name + ': integer required')
            if body.get(NAMES[0], camera.get(NAMES[0], {}).get('value', True)):
                raise ValueError('disable automatic exposure before changing manual values')
            if 'min' not in item or not item['min'] <= value <= item['max']:
                raise ValueError(name + ': outside camera range')
            if item.get('step', 0) and (value - item['min']) % item['step']:
                raise ValueError(name + ': outside camera step')
    return body


PAGE = r'''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Depth exposure test</title><style>
body{background:#181818;color:#eee;font:16px sans-serif;max-width:1000px;margin:20px auto;padding:0 12px}
#point{white-space:pre-line}img{max-width:100%;border:1px solid #666;cursor:crosshair}button,input{font:inherit;margin:6px}pre{white-space:pre-wrap}
.scale{height:16px;background:linear-gradient(to right,red,yellow,lime,cyan,blue)}
</style><h2>Depth exposure test</h2><div id="legend"></div><div class="scale"></div>
<p>Black: invalid depth. Click: distance and 21×21 ROI valid ratio. Right click: clear.</p>
<img id="view" alt="Waiting for depth"><p id="point"></p>
<label><input type="checkbox" id="ae" disabled>Depth auto exposure</label>
<div>Exposure <input id="exposure" type="range" disabled><input id="exposureValue" type="number" disabled></div>
<div>Gain <input id="gain" type="range" disabled><input id="gainValue" type="number" disabled></div>
<button id="apply" disabled>Apply manual values</button><button id="refresh">Read camera</button>
<p>Changes apply to the running camera only. Startup values: realsense2_camera/launch/rs_launch.py</p>
<pre id="status"></pre><script>
const $=id=>document.getElementById(id);let frame=0,objectURL=null,busy=false,dirty=false;
async function post(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 const d=await r.json();if(!r.ok)throw Error(d.error);return d;}
async function control(body){try{await post('/control',body);dirty=false}catch(e){$('status').textContent=e.message}}
$('ae').onchange=()=>control({'depth_module.enable_auto_exposure':$('ae').checked});
for(const id of ['exposure','gain']){const input=$(id),value=$(id+'Value');input.oninput=()=>{value.value=input.value;dirty=true};value.oninput=()=>{input.value=value.value;dirty=true}}
$('apply').onclick=()=>control({'depth_module.exposure':Number($('exposureValue').value),'depth_module.gain':Number($('gainValue').value)});
$('refresh').onclick=()=>{dirty=false;post('/refresh',{}).catch(e=>$('status').textContent=e.message)};
$('view').onclick=async e=>{const box=e.target.getBoundingClientRect();try{await post('/point',{frame_id:frame,x:Math.floor((e.clientX-box.left)*e.target.naturalWidth/box.width),y:Math.floor((e.clientY-box.top)*e.target.naturalHeight/box.height)})}catch(e){$('status').textContent=e.message}};
$('view').oncontextmenu=e=>{e.preventDefault();post('/point',{clear:true})};
async function image(){try{const r=await fetch('/frame.jpg',{cache:'no-store'});if(r.ok){const id=Number(r.headers.get('X-Frame-Id')),url=URL.createObjectURL(await r.blob());
 await new Promise(resolve=>{$('view').onload=resolve;$('view').src=url});frame=id;if(objectURL)URL.revokeObjectURL(objectURL);objectURL=url}}catch(e){}setTimeout(image,100)}
async function status(){try{const r=await fetch('/status',{cache:'no-store'}),s=await r.json(),c=s.camera,ae=c['depth_module.enable_auto_exposure'];
 $('status').textContent=s.message+'\n'+s.topic+' | '+s.encoding+' | frame age: '+(s.frame_age_sec===null?'no image':s.frame_age_sec+' s');
 $('legend').textContent=s.min_depth_m+' m (red) → '+s.max_depth_m+' m (blue), fixed scale';$('point').textContent=s.point;
 $('ae').disabled=s.busy||!ae||ae.read_only;if(ae)$('ae').checked=ae.value;
 for(const id of ['exposure','gain']){const item=c['depth_module.'+id],disabled=s.busy||!item||item.read_only||!ae||ae.value;
 for(const element of [$(id),$(id+'Value')]){element.disabled=disabled;if(item){element.min=item.min;element.max=item.max;element.step=item.step||1;if(!dirty)element.value=item.value}}}
 $('apply').disabled=s.busy||!ae||ae.value||!c['depth_module.exposure']||!c['depth_module.gain'];
 }catch(e){$('status').textContent='Viewer disconnected: '+e.message}setTimeout(status,500)}image();status();
</script>'''


class WebState:
    def __init__(self):
        self.lock = threading.Lock()
        self.inputs = queue.Queue(maxsize=32)
        self.jpeg = b''
        self.frame = 0
        self.status = {}


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, state):
        self.state = state
        self.stopping = threading.Event()
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def reply(self, status, data, content='application/json', frame=None):
        if content == 'application/json':data = json.dumps(data, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if frame is not None:self.send_header('X-Frame-Id', str(frame))
        self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):pass

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/':return self.reply(200, PAGE.encode(), 'text/html; charset=utf-8')
        if path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control','no-store');self.end_headers()
            previous=None
            try:
                while not self.server.stopping.is_set():
                    with self.server.state.lock:
                        jpeg,frame=self.server.state.jpeg,self.server.state.frame
                    if jpeg and frame!=previous:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                        self.wfile.flush();previous=frame
                    self.server.stopping.wait(.1)
            except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
            return
        with self.server.state.lock:
            status = dict(self.server.state.status)
            jpeg, frame = self.server.state.jpeg, self.server.state.frame
        if path == '/status':return self.reply(200, status)
        if path == '/frame.jpg':return self.reply(200 if jpeg else 503, jpeg, 'image/jpeg', frame)
        self.reply(404, {'error':'not found'})

    def do_POST(self):
        try:
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                return self.reply(403, {'error':'origin mismatch'})
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:raise ValueError('invalid request length')
            body = json.loads(self.rfile.read(length))
            path = urlsplit(self.path).path
            if path not in ('/control', '/point', '/refresh'):raise ValueError('unknown input')
            if not isinstance(body, dict):raise ValueError('object required')
            if path == '/control':
                with self.server.state.lock:camera = self.server.state.status.get('camera', {})
                validate_control(body, camera)
            self.server.state.inputs.put_nowait((path, body))
            self.reply(202, {'queued':True})
        except (ValueError, TypeError, queue.Full) as error:
            self.reply(400, {'error':str(error)})


class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_color_viewer')
        self.topic = self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw').value
        self.camera_node = self.declare_parameter('camera_node', '/camera/camera').value.rstrip('/')
        self.minimum = float(self.declare_parameter('min_depth_m', 0.1).value)
        self.maximum = float(self.declare_parameter('max_depth_m', 2.5).value)
        colorize(np.zeros((1, 1), np.float32), self.minimum, self.maximum)
        self.local = self.declare_parameter('show_debug_view', False).value
        streaming = self.declare_parameter('stream_debug_view', True).value
        port = self.declare_parameter('stream_debug_port', 8081).value
        if not self.topic or not self.camera_node.startswith('/') or not 1 <= port <= 65535:
            raise ValueError('invalid topic, absolute camera node, or port')
        if self.local and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('show_debug_view requires a desktop display')
        self.state = WebState()
        self.bridge = CvBridge()
        self.encoding = ''
        self.depth = None
        self.sequence = time.monotonic_ns() % (2**52)
        self.last_image = None
        self.frames = OrderedDict()
        self.point = None
        self.point_text = 'No point selected'
        self.camera = {}
        self.message = 'Waiting for camera parameter services'
        self.pending = None
        self.deferred = None
        self.read_at = 0.0
        self.refresh = True
        self.camera_clients = {
            'get':self.create_client(GetParameters, self.camera_node+'/get_parameters'),
            'describe':self.create_client(DescribeParameters, self.camera_node+'/describe_parameters'),
            'set':self.create_client(SetParameters, self.camera_node+'/set_parameters'),
        }
        self.sub = self.create_subscription(Image, self.topic, self.on_image, qos_profile_sensor_data)
        self.server = None
        if self.local:
            cv2.namedWindow('Depth colour', cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback('Depth colour', self.local_point)
        self.publish_status()
        if streaming:
            self.server = Server(('0.0.0.0', port), self.state)
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            self.get_logger().info(f'Depth viewer: http://<HOST_IP>:{port}')
        self.timer = self.create_timer(0.05, self.tick)

    def on_image(self, message):
        if message.encoding not in ('16UC1', '32FC1'):
            self.message = 'Unsupported depth encoding: '+message.encoding
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
            if raw.ndim != 2:raise ValueError('depth image must be two dimensional')
            self.depth = raw.astype(np.float32) * (0.001 if message.encoding == '16UC1' else 1.0)
            self.encoding = message.encoding
            self.last_image = time.monotonic()
            self.sequence += 1
            self.frames[self.sequence] = self.depth
            while len(self.frames) > 8:self.frames.popitem(last=False)
        except (ValueError, RuntimeError, cv2.error) as error:
            self.message = 'Depth conversion failed: '+str(error)

    def local_point(self, event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:self.select_point({'frame_id':self.sequence, 'x':x, 'y':y})
        elif event == cv2.EVENT_RBUTTONDOWN:self.select_point({'clear':True})

    def select_point(self, body):
        if body.get('clear'):
            self.point = None;self.point_text = 'No point selected';return
        frame, x, y = body.get('frame_id'), body.get('x'), body.get('y')
        if type(frame) is not int or type(x) is not int or type(y) is not int:
            raise ValueError('integer frame_id/x/y required')
        depth = self.frames.get(frame)
        if depth is None:raise ValueError('displayed frame expired; click the current image')
        height, width = depth.shape
        if not 0 <= x < width or not 0 <= y < height:raise ValueError('click outside depth image')
        self.point = (x,y)
        self.point_text = self.measure(depth, x, y) + ' [clicked frame]'

    @staticmethod
    def measure(depth, x, y):
        roi = depth[max(0,y-10):y+11,max(0,x-10):x+11]
        valid = roi[np.isfinite(roi) & (roi > 0)]
        z = float(depth[y,x])
        centre = f'{z:.3f} m' if math.isfinite(z) and z > 0 else 'invalid'
        median = f'{float(np.median(valid)):.3f} m' if valid.size else 'unavailable'
        return f'({x},{y}) Z={centre} | ROI median={median} | valid={100*valid.size/roi.size:.1f}%'

    def begin(self, kind, request):
        client = self.camera_clients[kind]
        if not client.service_is_ready():
            self.message = 'Camera service unavailable: '+kind
            self.read_at = time.monotonic()+1
            return
        self.pending = (kind, client.call_async(request), time.monotonic())

    def finish_pending(self):
        if not self.pending:return
        kind, future, started = self.pending
        if not future.done():
            if time.monotonic()-started < 3:return
            self.camera_clients[kind].remove_pending_request(future)
            future.cancel();self.pending = None
            self.camera = {};self.message = 'Camera request timed out; read again before applying'
            return
        self.pending = None
        try:
            response = future.result()
            if kind == 'get':
                for name, value in zip(NAMES, response.values):
                    if value.type not in (1,2):
                        self.camera.pop(name,None);continue
                    self.camera.setdefault(name, {})['value'] = value.bool_value if value.type == 1 else value.integer_value
                self.read_at = time.monotonic()+0.8
                self.refresh = False
                if not self.camera:self.message = 'Depth parameters unavailable on selected camera node'
            elif kind == 'describe':
                for name, desc in zip(NAMES, response.descriptors):
                    if desc.type not in (1,2):continue
                    item = self.camera.setdefault(name, {})
                    item['read_only'] = desc.read_only
                    if desc.integer_range:
                        span = desc.integer_range[0]
                        item.update(min=span.from_value, max=span.to_value, step=span.step)
                self.message = 'Camera connected. Exposure controls affect the running camera only.'
            elif kind == 'set':
                failures = [result.reason for result in response.results if not result.successful]
                self.message = 'Camera rejected: '+'; '.join(failures) if failures else 'Camera accepted update; reading parameters'
                self.refresh = True
        except Exception as error:
            self.camera = {};self.message = 'Camera request failed: '+str(error)

    def tick(self):
        self.finish_pending()
        for _ in range(8):
            if self.deferred:
                if self.pending:break
                path,body=self.deferred;self.deferred=None
            else:
                try:path, body = self.state.inputs.get_nowait()
                except queue.Empty:break
            try:
                if path == '/point':self.select_point(body)
                elif path == '/refresh':self.refresh = True
                else:
                    if self.pending:
                        self.deferred=(path,body);break
                    validate_control(body, self.camera)
                    request = SetParameters.Request()
                    request.parameters = [Parameter(name, value=body[name]).to_parameter_msg() for name in NAMES if name in body]
                    self.begin('set', request)
            except (ValueError, TypeError) as error:self.message = str(error)
        if self.pending is None and time.monotonic() >= self.read_at:
            if not self.camera:
                request = DescribeParameters.Request();request.names = list(NAMES)
                self.begin('describe', request)
            else:
                request = GetParameters.Request();request.names = list(NAMES)
                self.begin('get', request)
        elif self.pending is None and self.refresh:
            request = GetParameters.Request();request.names = list(NAMES)
            self.begin('get', request)
        if self.depth is not None:
            display = colorize(self.depth, self.minimum, self.maximum)
            if self.point:
                x,y = self.point
                if y < self.depth.shape[0] and x < self.depth.shape[1]:
                    cv2.drawMarker(display, (x,y), (255,255,255), cv2.MARKER_CROSS, 15)
                    live = self.measure(self.depth, x,y)
                else:live = 'Selected point outside new image size'
            else:live = 'Click image to measure depth'
            if self.local:
                cv2.putText(display, live, (8,20), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255),1)
                cv2.imshow('Depth colour',display)
            ok, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with self.state.lock:self.state.jpeg = jpeg.tobytes();self.state.frame = self.sequence
            self.live_text = live
        if self.local and cv2.waitKey(1) & 255 in (27,ord('q')):rclpy.try_shutdown()
        self.publish_status()

    def publish_status(self):
        status = {'camera':{k:dict(v) for k,v in self.camera.items()}, 'busy':self.pending is not None or self.deferred is not None,
                  'topic':self.topic,'encoding':self.encoding,'message':self.message,
                  'min_depth_m':self.minimum,'max_depth_m':self.maximum,
                  'frame_age_sec':round(time.monotonic()-self.last_image,2) if self.last_image is not None else None,
                  'point':self.point_text+'\n'+getattr(self,'live_text','')}
        with self.state.lock:self.state.status = status

    def destroy_node(self):
        if self.server:
            self.server.stopping.set()
            self.server.shutdown();self.server.server_close()
        if self.local:cv2.destroyAllWindows()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = DepthViewer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':main()
