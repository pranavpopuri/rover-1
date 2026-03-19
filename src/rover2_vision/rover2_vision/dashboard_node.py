#!/usr/bin/env python3
"""
Web dashboard for rover monitoring.
Shows camera feed, detections, follower state, and motor commands.

Open http://<robot-ip>:8080 in a browser.
"""

import json
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from io import BytesIO

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge


# Shared state between ROS node and HTTP server
state = {
    'frame_jpeg': None,
    'detections': {},
    'cmd_vel': {'linear_x': 0.0, 'angular_z': 0.0},
    'follower_state': 'UNKNOWN',
    'last_detection_time': 0.0,
    'fps': 0.0,
    'inference_fps': 0.0,
}
state_lock = threading.Lock()

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<title>Rover2 Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #1a1a2e; color: #eee; font-family: monospace; padding: 12px; }
  h1 { color: #0ff; font-size: 1.3em; margin-bottom: 10px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 900px; }
  .panel { background: #16213e; border: 1px solid #0f3460; border-radius: 6px; padding: 10px; }
  .panel h2 { color: #e94560; font-size: 0.95em; margin-bottom: 8px; }
  .cam-panel { grid-column: 1 / -1; text-align: center; }
  #cam { max-width: 100%; border-radius: 4px; background: #000; }
  .stat { display: flex; justify-content: space-between; padding: 4px 0;
          border-bottom: 1px solid #0f3460; }
  .stat:last-child { border-bottom: none; }
  .label { color: #888; }
  .val { color: #0ff; font-weight: bold; }
  .state-SEARCHING { color: #f0ad4e; }
  .state-APPROACHING { color: #5bc0de; }
  .state-ARRIVED { color: #5cb85c; }
  .state-UNKNOWN { color: #888; }
  .bar-wrap { background: #0f3460; border-radius: 3px; height: 14px; margin-top: 3px; }
  .bar { height: 100%; border-radius: 3px; transition: width 0.2s; }
  .bar-fwd { background: #5bc0de; }
  .bar-turn { background: #e94560; }
  .det-box { background: #1a1a2e; padding: 6px; border-radius: 4px; margin-top: 4px; font-size: 0.85em; }
  @media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<h1>ROVER2 DASHBOARD</h1>
<div class="grid">
  <div class="panel cam-panel">
    <h2>CAMERA FEED</h2>
    <img id="cam" src="/frame" alt="camera">
  </div>
  <div class="panel">
    <h2>FOLLOWER STATE</h2>
    <div class="stat">
      <span class="label">State</span>
      <span class="val" id="fstate">--</span>
    </div>
    <div class="stat">
      <span class="label">Target</span>
      <span class="val" id="target_class">--</span>
    </div>
    <div class="stat">
      <span class="label">Detected</span>
      <span class="val" id="det_count">--</span>
    </div>
    <div class="stat">
      <span class="label">Confidence</span>
      <span class="val" id="confidence">--</span>
    </div>
    <div class="stat">
      <span class="label">BBox Height</span>
      <span class="val" id="bbox_h">--</span>
    </div>
    <div class="stat">
      <span class="label">Inference FPS</span>
      <span class="val" id="inf_fps">--</span>
    </div>
  </div>
  <div class="panel">
    <h2>MOTOR COMMANDS</h2>
    <div class="stat">
      <span class="label">Linear X</span>
      <span class="val" id="lin_x">0.00 m/s</span>
    </div>
    <div class="bar-wrap"><div class="bar bar-fwd" id="bar_fwd" style="width:0%"></div></div>
    <div class="stat" style="margin-top:8px">
      <span class="label">Angular Z</span>
      <span class="val" id="ang_z">0.00 rad/s</span>
    </div>
    <div style="display:flex;gap:2px;margin-top:3px">
      <div class="bar-wrap" style="flex:1"><div class="bar bar-turn" id="bar_left" style="width:0%;float:right"></div></div>
      <div style="width:2px;background:#0f3460"></div>
      <div class="bar-wrap" style="flex:1"><div class="bar bar-turn" id="bar_right" style="width:0%"></div></div>
    </div>
    <div class="det-box" id="det_detail" style="margin-top:10px">No detection</div>
  </div>
</div>
<script>
function refresh() {
  document.getElementById('cam').src = '/frame?' + Date.now();
  fetch('/state').then(r => r.json()).then(s => {
    const fs = document.getElementById('fstate');
    fs.textContent = s.follower_state;
    fs.className = 'val state-' + s.follower_state;

    document.getElementById('target_class').textContent = s.target_class || '--';

    const det = s.detections;
    const hasDet = det && det.detections && det.detections.length > 0;
    document.getElementById('det_count').textContent = hasDet ? det.detections.length : '0';

    if (hasDet) {
      const d = det.detections[0];
      document.getElementById('confidence').textContent = (d.confidence * 100).toFixed(0) + '%';
      document.getElementById('bbox_h').textContent = d.bbox_h + 'px / ' + s.stop_bbox_height + 'px';
      document.getElementById('det_detail').textContent =
        d.class_name + ' @ (' + d.bbox_center_x + ',' + d.bbox_center_y + ') ' +
        d.bbox_w + 'x' + d.bbox_h + 'px';
    } else {
      document.getElementById('confidence').textContent = '--';
      document.getElementById('bbox_h').textContent = '--';
      document.getElementById('det_detail').textContent = 'No detection';
    }

    document.getElementById('inf_fps').textContent = s.inference_fps.toFixed(1);

    const lx = s.cmd_vel.linear_x;
    const az = s.cmd_vel.angular_z;
    document.getElementById('lin_x').textContent = lx.toFixed(3) + ' m/s';
    document.getElementById('ang_z').textContent = az.toFixed(3) + ' rad/s';
    document.getElementById('bar_fwd').style.width = Math.min(100, Math.abs(lx) / 0.15 * 100) + '%';
    document.getElementById('bar_left').style.width = (az > 0 ? Math.min(100, az / 0.5 * 100) : 0) + '%';
    document.getElementById('bar_right').style.width = (az < 0 ? Math.min(100, -az / 0.5 * 100) : 0) + '%';
  });
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Silence request logs

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

        elif self.path.startswith('/frame'):
            with state_lock:
                jpeg = state['frame_jpeg']
            if jpeg is None:
                # Send a 1x1 black pixel
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.end_headers()
                blank = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(blank, 'No image', (80, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                _, buf = cv2.imencode('.jpg', blank)
                self.wfile.write(buf.tobytes())
            else:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(jpeg)

        elif self.path == '/state':
            with state_lock:
                payload = {
                    'follower_state': state['follower_state'],
                    'detections': state['detections'],
                    'cmd_vel': state['cmd_vel'],
                    'target_class': state['detections'].get('target_class', ''),
                    'inference_fps': state['inference_fps'],
                    'stop_bbox_height': state.get('stop_bbox_height', 300),
                }
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode())

        else:
            self.send_response(404)
            self.end_headers()


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        self.bridge = CvBridge()
        self.last_det_times = []

        # Subscribe to all relevant topics
        self.create_subscription(Image, '/image_raw', self.image_cb, 1)
        self.create_subscription(String, '/detections', self.detection_cb, 10)
        self.create_subscription(
            TwistStamped, '/diff_drive_controller/cmd_vel', self.cmd_cb, 10)

        # Start HTTP server in a thread
        self.httpd = HTTPServer(('0.0.0.0', self.port), DashboardHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(f'Dashboard running at http://0.0.0.0:{self.port}')

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        # Draw detection overlay
        with state_lock:
            det_data = state['detections']

        if det_data and det_data.get('detections'):
            for d in det_data['detections']:
                x1 = d['bbox_x']
                y1 = d['bbox_y']
                x2 = x1 + d['bbox_w']
                y2 = y1 + d['bbox_h']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{d['class_name']} {d['confidence']:.0%}"
                cv2.putText(frame, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Draw center crosshair
                cx, cy = d['bbox_center_x'], d['bbox_center_y']
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        # Draw image center line
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (50, 50, 50), 1)

        # Encode to JPEG
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with state_lock:
            state['frame_jpeg'] = buf.tobytes()

    def detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        now = time.time()
        self.last_det_times.append(now)
        # Keep last 2 seconds of timestamps for FPS calculation
        self.last_det_times = [t for t in self.last_det_times if now - t < 2.0]
        fps = len(self.last_det_times) / 2.0 if len(self.last_det_times) > 1 else 0.0

        with state_lock:
            state['detections'] = data
            state['inference_fps'] = fps

            # Infer follower state from detection presence
            if data.get('detections'):
                state['last_detection_time'] = now

    def cmd_cb(self, msg):
        with state_lock:
            state['cmd_vel'] = {
                'linear_x': msg.twist.linear.x,
                'angular_z': msg.twist.angular.z,
            }
            # Infer follower state from commands
            lx = msg.twist.linear.x
            az = msg.twist.angular.z
            has_det = (time.time() - state['last_detection_time']) < 2.0
            if not has_det:
                state['follower_state'] = 'SEARCHING'
            elif abs(lx) > 0.001 or abs(az) > 0.001:
                state['follower_state'] = 'APPROACHING'
            else:
                state['follower_state'] = 'ARRIVED'


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()
