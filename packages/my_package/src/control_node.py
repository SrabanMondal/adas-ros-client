#!/usr/bin/env python3
"""
Duckiebot Web-Controlled Driving Node
--------------------------------------
Single ROS node that:
  1. Runs a Flask web server (port 8080) serving the control UI
  2. Receives telemetry  {steer, brake}  via POST /api/control
  3. Converts to differential wheel velocities
  4. Publishes WheelsCmdStamped at 10 Hz
  5. Auto-stops if no update received within 500 ms
"""

import os
import threading
import time

import rospy
from flask import Flask, request, jsonify, send_from_directory
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

# ── Tuning constants ──────────────────────────────────────
MAX_SPEED = 0.2          # m/s  – maximum forward wheel speed
MAX_STEER = 27.0         # deg  – maximum steering angle
BRAKE_ZERO = 0.8         # brake force at which effective speed = 0
CONTROL_HZ = 10          # wheel-command publish rate
SAFETY_TIMEOUT = 0.5     # seconds – auto-stop if no update
FLASK_PORT = 8080


class ControlNode(DTROS):

    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # ── ROS setup ──────────────────────────────────────
        self._vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        self._pub = rospy.Publisher(
            f"/{self._vehicle}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1,
        )

        # ── Shared state (guarded by lock) ─────────────────
        self._lock = threading.Lock()
        self._steer = 0.0        # degrees  (+ = right)
        self._brake = 1.0        # 0..1     (default: full-brake = stopped)
        self._mode = "manual"
        self._last_update = time.time()

        # ── Start Flask in a daemon thread ─────────────────
        self._start_flask()

        # ── Periodic wheel-command publisher ───────────────
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / CONTROL_HZ), self._control_cb
        )
        rospy.loginfo(
            "[ControlNode] Ready  —  UI at http://0.0.0.0:%d", FLASK_PORT
        )

    # ════════════════════════════════════════════════════════
    #  Flask web server
    # ════════════════════════════════════════════════════════
    def _start_flask(self):
        static_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "static"
        )
        app = Flask(__name__, static_folder=static_dir)

        # Serve the control-panel page
        @app.route("/")
        def index():
            return send_from_directory(app.static_folder, "index.html")

        # Receive steer + brake from the UI (manual or auto-forwarded)
        @app.route("/api/control", methods=["POST"])
        def api_control():
            data = request.get_json(silent=True) or {}
            s = max(-MAX_STEER, min(MAX_STEER, float(data.get("steer", 0.0))))
            b = max(0.0, min(1.0, float(data.get("brake", 1.0))))
            with self._lock:
                self._steer = s
                self._brake = b
                self._last_update = time.time()
            return jsonify(status="ok")

        # Switch between manual / auto
        @app.route("/api/mode", methods=["POST"])
        def api_mode():
            data = request.get_json(silent=True) or {}
            mode = data.get("mode", "manual")
            with self._lock:
                self._mode = mode
                if mode == "auto":
                    self._brake = 1.0
                    self._steer = 0.0
            return jsonify(status="ok", mode=mode)

        # Return live state (polled by the UI for telemetry)
        @app.route("/api/status")
        def api_status():
            with self._lock:
                vl, vr = self._wheel_velocities()
                return jsonify(
                    steer=round(self._steer, 2),
                    brake=round(self._brake, 3),
                    mode=self._mode,
                    vel_left=round(vl, 4),
                    vel_right=round(vr, 4),
                )

        t = threading.Thread(
            target=lambda: app.run(
                host="0.0.0.0", port=FLASK_PORT, threaded=True
            ),
            daemon=True,
        )
        t.start()

    # ════════════════════════════════════════════════════════
    #  Differential-drive math
    # ════════════════════════════════════════════════════════
    def _wheel_velocities(self):
        """Convert (steer, brake) → (vel_left, vel_right)."""
        # brake → effective speed
        if self._brake >= BRAKE_ZERO:
            speed = 0.0
        else:
            speed = MAX_SPEED * max(0.0, 1.0 - self._brake / BRAKE_ZERO)

        # steer → differential  (+steer ⇒ turn right ⇒ left faster)
        n = self._steer / MAX_STEER  # normalised [-1, 1]
        vl = speed * (1.0 + n)
        vr = speed * (1.0 - n)
        return vl, vr

    # ════════════════════════════════════════════════════════
    #  ROS timer callback
    # ════════════════════════════════════════════════════════
    def _control_cb(self, _event):
        with self._lock:
            elapsed = time.time() - self._last_update
            if elapsed > SAFETY_TIMEOUT:
                vl, vr = 0.0, 0.0
            else:
                vl, vr = self._wheel_velocities()

        msg = WheelsCmdStamped()
        msg.vel_left = vl
        msg.vel_right = vr
        self._pub.publish(msg)

    # ════════════════════════════════════════════════════════
    #  Graceful shutdown
    # ════════════════════════════════════════════════════════
    def on_shutdown(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self._pub.publish(msg)
        rospy.loginfo("[ControlNode] Shutdown — wheels stopped.")


if __name__ == "__main__":
    node = ControlNode(node_name="control_node")
    rospy.spin()
