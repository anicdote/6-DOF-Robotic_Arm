#!/usr/bin/env python3
"""Local browser dashboard for the camera-guided pick/place arm."""

from __future__ import annotations

import json
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    "joint1_base_yaw",
    "joint2_shoulder_pitch",
    "joint3_elbow_pitch",
    "joint4_wrist_pitch",
]


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PNP Arm Dashboard</title>
  <style>
    :root { color-scheme: dark; --bg:#0b1017; --panel:#141c27; --line:#263244;
      --text:#edf3fa; --muted:#91a0b5; --accent:#42d3a4; --danger:#ff6174; }
    * { box-sizing:border-box; }
    body { margin:0; font-family:Inter,system-ui,sans-serif; color:var(--text);
      background:radial-gradient(circle at 15% 0,#18283b 0,var(--bg) 42%); }
    header { display:flex; justify-content:space-between; align-items:center;
      padding:22px 28px; border-bottom:1px solid var(--line); background:#0d141ed9; }
    h1 { margin:0; font-size:20px; letter-spacing:.04em; }
    .sub { color:var(--muted); font-size:12px; margin-top:5px; }
    .badge { padding:8px 13px; border:1px solid #35506a; border-radius:999px;
      background:#132333; font:700 12px ui-monospace,monospace; color:#8fdcff; }
    main { max-width:1280px; margin:auto; padding:24px; display:grid;
      grid-template-columns:minmax(0,1.7fr) minmax(300px,.8fr); gap:20px; }
    .card { background:linear-gradient(150deg,#172231,#101721); border:1px solid var(--line);
      border-radius:16px; box-shadow:0 16px 45px #0006; overflow:hidden; }
    .card h2 { font-size:13px; letter-spacing:.12em; text-transform:uppercase;
      color:#aebbd0; margin:0; padding:16px 18px; border-bottom:1px solid var(--line); }
    .camera { position:relative; min-height:380px; display:grid; place-items:center; background:#05080c; }
    .camera img { width:100%; display:block; min-height:380px; object-fit:contain; }
    .camera-label { position:absolute; left:14px; bottom:12px; padding:6px 9px;
      background:#05080cbb; border:1px solid #344154; border-radius:7px;
      color:#b8c5d6; font:11px ui-monospace,monospace; }
    .side { display:grid; gap:20px; align-content:start; }
    .body { padding:17px; }
    .colours { display:grid; grid-template-columns:repeat(3,1fr); gap:9px; }
    button { border:1px solid #344154; color:var(--text); background:#1c2735;
      border-radius:10px; padding:12px 10px; font-weight:750; cursor:pointer; transition:.15s; }
    button:hover { transform:translateY(-1px); border-color:#6e849e; }
    .colour.active { outline:2px solid #dbe8f5; outline-offset:2px; }
    .red { background:#6e2630; }.green { background:#176044; }.blue { background:#164f77; }
    .actions { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:16px; }
    .start { grid-column:1/-1; background:#147c5b; border-color:#32b98d; font-size:15px; }
    .stop { background:#772b38; border-color:#b84c60; }.reset { background:#29394d; }
    .telemetry { display:grid; gap:10px; }
    .row { display:flex; justify-content:space-between; gap:14px; padding:9px 0;
      border-bottom:1px solid #243041; color:var(--muted); font-size:13px; }
    .row:last-child { border:0; }.value { color:var(--text); font-family:ui-monospace,monospace; }
    .joints { display:grid; grid-template-columns:1fr 1fr; gap:9px; margin-top:12px; }
    .joint { background:#0e151e; border:1px solid #263244; border-radius:9px; padding:10px; }
    .joint small { color:var(--muted); display:block; }.joint strong { font:14px ui-monospace,monospace; }
    #notice { min-height:20px; color:var(--accent); font-size:12px; margin-top:10px; }
    .manual { display:grid; gap:10px; }.manual label { display:grid; grid-template-columns:82px 1fr 58px;
      align-items:center; gap:8px; color:var(--muted); font-size:12px; }
    input[type=range] { accent-color:var(--accent); width:100%; }
    .manual output { color:var(--text); font:11px ui-monospace,monospace; text-align:right; }
    @media(max-width:850px){ main{grid-template-columns:1fr}.camera,.camera img{min-height:250px} }
  </style>
</head>
<body>
  <header><div><h1>CAMERA-GUIDED PICK &amp; PLACE</h1><div class="sub">ROS 2 operator dashboard</div></div>
    <div id="status" class="badge">CONNECTING</div></header>
  <main>
    <section class="card"><h2>Live detection camera</h2><div class="camera">
      <img id="camera" alt="Waiting for camera frames"><div id="frame" class="camera-label">NO FRAME</div>
    </div></section>
    <aside class="side">
      <section class="card"><h2>Operation</h2><div class="body">
        <div class="colours">
          <button class="colour red" data-colour="red">RED</button>
          <button class="colour green" data-colour="green">GREEN</button>
          <button class="colour blue" data-colour="blue">BLUE</button>
        </div>
        <div class="actions"><button class="start" data-command="START">START AUTONOMOUS PICK</button>
          <button class="stop" data-command="STOP">STOP / HOLD</button>
          <button class="reset" data-command="RESET">RESET ARM + CUBE</button></div>
        <div id="notice"></div>
      </div></section>
      <section class="card"><h2>Telemetry</h2><div class="body telemetry">
        <div class="row"><span>Target colour</span><span id="target" class="value">—</span></div>
        <div class="row"><span>Camera age</span><span id="age" class="value">—</span></div>
        <div class="joints" id="joints"></div>
      </div></section>
      <section class="card"><h2>Manual joint control</h2><div class="body manual">
        <label>Base <input id="j0" type="range" min="-3.14" max="3.14" step="0.01" disabled><output></output></label>
        <label>Shoulder <input id="j1" type="range" min="-2.20" max="1.20" step="0.01" disabled><output></output></label>
        <label>Elbow <input id="j2" type="range" min="-2.60" max="2.60" step="0.01" disabled><output></output></label>
        <label>Wrist <input id="j3" type="range" min="-2.60" max="2.60" step="0.01" disabled><output></output></label>
        <label>Gripper <input id="gripper" type="range" min="0" max="0.30" step="0.01" value="0" disabled><output></output></label>
        <button id="sendManual" disabled>MOVE ARM / CANCEL AUTO</button>
        <div class="sub">Move a slider to jog the arm live. Controls unlock
          after the first joint-state update and cancel autonomous motion.</div>
      </div></section>
    </aside>
  </main>
<script>
const notice=document.getElementById('notice');
async function post(path,payload){
  try { const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
    const j=await r.json(); notice.textContent=j.message||'Command sent'; }
  catch(e){ notice.textContent='Dashboard connection failed'; }
}
document.querySelectorAll('[data-colour]').forEach(b=>b.onclick=()=>post('/api/target',{colour:b.dataset.colour}));
document.querySelectorAll('[data-command]').forEach(b=>b.onclick=()=>post('/api/command',{command:b.dataset.command}));
const sliderIds=['j0','j1','j2','j3','gripper'];
let manualPoseInitialised=false;
let manualPoseDirty=false;
let manualSendTimer=null;
let lastManualSend=0;
const manualIntervalMs=125;
function manualPayload(duration){return {
  joints:['j0','j1','j2','j3'].map(id=>Number(document.getElementById(id).value)),
  gripper:Number(document.getElementById('gripper').value), duration:duration};}
function sendManual(duration=.30){
  if (!manualPoseInitialised) return;
  lastManualSend=Date.now();
  post('/api/manual',manualPayload(duration));
}
function scheduleManual(){
  const delay=Math.max(0,manualIntervalMs-(Date.now()-lastManualSend));
  if (manualSendTimer!==null) return;
  manualSendTimer=setTimeout(()=>{manualSendTimer=null;sendManual();},delay);
}
sliderIds.forEach(id=>{const el=document.getElementById(id); const out=el.parentElement.querySelector('output');
  const show=()=>out.textContent=Number(el.value).toFixed(2);
  el.oninput=()=>{manualPoseDirty=true; show(); scheduleManual();}; show();});
document.getElementById('sendManual').onclick=()=>sendManual(1.0);
async function update(){
  try { const s=await (await fetch('/api/state',{cache:'no-store'})).json();
    document.getElementById('status').textContent=s.status;
    document.getElementById('target').textContent=s.target_colour.toUpperCase();
    document.querySelectorAll('[data-colour]').forEach(b=>b.classList.toggle('active',b.dataset.colour===s.target_colour));
    const age=s.camera_age_s; document.getElementById('age').textContent=age===null?'WAITING':age.toFixed(2)+' s';
    document.getElementById('frame').textContent=age===null?'NO FRAME':'DEBUG VIEW · '+age.toFixed(2)+' s';
    document.getElementById('joints').innerHTML=Object.entries(s.joints).map(([k,v],i)=>
      `<div class="joint"><small>J${i+1}</small><strong>${Number(v).toFixed(3)} rad</strong></div>`).join('');
    // Initialise from the actual arm pose once.  This prevents the first
    // manual command from unexpectedly sending the range-input midpoints.
    if (!manualPoseInitialised && !manualPoseDirty) {
      const values=Object.values(s.joints);
      if (values.length===4 && values.every(Number.isFinite)) {
        ['j0','j1','j2','j3'].forEach((id,i)=>{
          const el=document.getElementById(id); el.value=values[i];
          el.parentElement.querySelector('output').textContent=Number(el.value).toFixed(2);
        });
        manualPoseInitialised=true;
        sliderIds.forEach(id=>document.getElementById(id).disabled=false);
        document.getElementById('sendManual').disabled=false;
      }
    }
  } catch(e){ document.getElementById('status').textContent='OFFLINE'; }
}
setInterval(update,500); update();
const cam=document.getElementById('camera'); setInterval(()=>{cam.src='/camera.jpg?t='+Date.now()},200);
</script></body></html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    dashboard: "ArmDashboard"

    def log_message(self, _format: str, *_args) -> None:
        return

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, DASHBOARD_HTML.encode(), "text/html; charset=utf-8")
        elif path == "/api/state":
            self._send(
                200,
                json.dumps(self.dashboard.snapshot()).encode(),
                "application/json",
            )
        elif path == "/camera.jpg":
            frame = self.dashboard.latest_jpeg()
            if frame is None:
                self._send(404, b"Camera frame not available", "text/plain")
            else:
                self._send(200, frame, "image/jpeg")
        else:
            self._send(404, b"Not found", "text/plain")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        try:
            size = min(int(self.headers.get("Content-Length", "0")), 4096)
            payload = json.loads(self.rfile.read(size) or b"{}")
        except (ValueError, json.JSONDecodeError):
            self._send(400, b'{"message":"Invalid JSON"}', "application/json")
            return

        path = urlparse(self.path).path
        if path == "/api/command":
            command = str(payload.get("command", "")).upper()
            if command not in {"START", "STOP", "RESET"}:
                self._send(400, b'{"message":"Invalid command"}', "application/json")
                return
            self.dashboard.queue_command(command)
            message = f"{command.title()} command sent"
        elif path == "/api/target":
            colour = str(payload.get("colour", "")).lower()
            if colour not in {"red", "green", "blue"}:
                self._send(400, b'{"message":"Invalid colour"}', "application/json")
                return
            self.dashboard.queue_target(colour)
            message = f"Target changed to {colour}"
        elif path == "/api/manual":
            joints = payload.get("joints")
            gripper = payload.get("gripper")
            duration = payload.get("duration", 2.0)
            if (
                not isinstance(joints, list)
                or len(joints) != 4
                or not all(isinstance(value, (int, float)) for value in joints)
                or not isinstance(gripper, (int, float))
                or not isinstance(duration, (int, float))
                or not 0.25 <= duration <= 5.0
            ):
                self._send(400, b'{"message":"Invalid manual pose"}', "application/json")
                return
            self.dashboard.queue_manual(
                [float(value) for value in joints], float(gripper), float(duration)
            )
            message = "Manual pose sent; autonomous motion cancelled"
        else:
            self._send(404, b'{"message":"Not found"}', "application/json")
            return
        self._send(200, json.dumps({"message": message}).encode(), "application/json")


class ArmDashboard(Node):
    def __init__(self) -> None:
        super().__init__("arm_dashboard")
        self.declare_parameter("listen_address", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("target_color", "red")
        self.declare_parameter("open_browser", False)
        self.declare_parameter("joint_state_topic", "/joint_states")

        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._frame_time: float | None = None
        self._status = "READY"
        self._target_colour = str(self.get_parameter("target_color").value).lower()
        self._joints = {name: 0.0 for name in ARM_JOINTS}
        self._pending: deque[tuple[str, object]] = deque()
        self._bridge = CvBridge()

        self._command_pub = self.create_publisher(String, "/pick_place/command", 10)
        self._target_pub = self.create_publisher(
            String, "/pick_place/target_color", 10
        )
        self._manual_arm_pub = self.create_publisher(
            JointTrajectory, "/pick_place/manual_arm", 10
        )
        self._manual_gripper_pub = self.create_publisher(
            JointTrajectory, "/pick_place/manual_gripper", 10
        )
        self.create_subscription(String, "/pick_place/status", self._status_cb, 10)
        self.create_subscription(
            Image,
            "/color_tracker/debug_image",
            self._image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._joint_cb,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._flush_commands)

        address = str(self.get_parameter("listen_address").value)
        port = int(self.get_parameter("port").value)
        handler = type("BoundDashboardHandler", (DashboardHandler,), {})
        handler.dashboard = self
        self._server = ThreadingHTTPServer((address, port), handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._server_thread.start()
        self.url = f"http://{address}:{port}"
        self.get_logger().info(f"Operator dashboard: {self.url}")
        if bool(self.get_parameter("open_browser").value):
            threading.Timer(1.0, lambda: webbrowser.open(self.url)).start()

    def _status_cb(self, msg: String) -> None:
        with self._lock:
            self._status = msg.data

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82]
            )
            if ok:
                with self._lock:
                    self._jpeg = encoded.tobytes()
                    self._frame_time = time.monotonic()
        except Exception as exc:
            self.get_logger().debug(f"Dashboard frame conversion failed: {exc}")

    def _joint_cb(self, msg: JointState) -> None:
        incoming = dict(zip(msg.name, msg.position))
        with self._lock:
            for name in ARM_JOINTS:
                if name in incoming:
                    self._joints[name] = float(incoming[name])

    def queue_command(self, command: str) -> None:
        with self._lock:
            self._pending.append(("command", command))

    def queue_target(self, colour: str) -> None:
        with self._lock:
            self._target_colour = colour
            self._pending.append(("target", colour))

    def queue_manual(
        self, joints: list[float], gripper: float, duration: float = 2.0
    ) -> None:
        limits = [(-3.14, 3.14), (-2.20, 1.20), (-2.60, 2.60), (-2.60, 2.60)]
        if any(value < low or value > high for value, (low, high) in zip(joints, limits)):
            return
        if not 0.0 <= gripper <= 0.30:
            return
        with self._lock:
            self._pending.append(("manual", (joints, gripper, duration)))

    @staticmethod
    def _trajectory(
        names: list[str], positions: list[float], duration: float
    ) -> JointTrajectory:
        message = JointTrajectory()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        message.points = [point]
        return message

    def _flush_commands(self) -> None:
        with self._lock:
            pending = list(self._pending)
            self._pending.clear()
        for kind, value in pending:
            if kind == "command":
                self._command_pub.publish(String(data=value))
            elif kind == "target":
                self._target_pub.publish(String(data=value))
            else:
                joints, gripper, duration = value
                self._manual_arm_pub.publish(
                    self._trajectory(ARM_JOINTS, joints, duration)
                )
                self._manual_gripper_pub.publish(self._trajectory(
                    ["left_finger_joint", "right_finger_joint"],
                    [-gripper, gripper],
                    duration,
                ))

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def snapshot(self) -> dict:
        with self._lock:
            frame_age = (
                None
                if self._frame_time is None
                else max(0.0, time.monotonic() - self._frame_time)
            )
            return {
                "status": self._status,
                "target_colour": self._target_colour,
                "camera_age_s": frame_age,
                "joints": dict(self._joints),
            }

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
