#!/usr/bin/env python3
"""Tkinter MQTT server for experiment/robot_exp.c challenge mapping modes."""

from __future__ import annotations

import argparse
import json
import math
import queue
import signal
import time
import tkinter as tk
from dataclasses import dataclass, field
from typing import Any, Optional

import paho.mqtt.client as mqtt


@dataclass
class RobotConfig:
    module: str
    username: str
    password: str = ""
    map_x: float = 0.0
    map_y: float = 0.0
    map_theta: float = 0.0


@dataclass
class RobotViewState:
    robot_id: str
    status: str = "listening"
    ready: bool = False
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    mode: str = "idle"
    moving: bool = False
    done: bool = True
    ir_left: bool = False
    ir_right: bool = False
    tof_ready: Optional[bool] = None
    last_message_at: Optional[float] = None
    local_x: float = 0.0
    local_y: float = 0.0
    local_theta: float = 0.0
    last_area_command_at: float = 0.0
    path: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class BorderPoint:
    x: float
    y: float
    theta: float
    event: str
    detected_at: float
    ir_left: bool = False
    ir_right: bool = False


@dataclass
class BorderLine:
    x: float
    y: float
    theta: float
    detected_at: float


@dataclass
class MountainDetection:
    x: float
    y: float
    distance_m: float
    theta: float
    detected_at: float
    hits: int = 1


@dataclass
class ColorDetection:
    x: float
    y: float
    color: str
    red: int
    green: int
    blue: int
    samples: int
    detected_at: float
    hits: int = 1


@dataclass
class CraterDetection:
    x: float
    y: float
    detected_at: float
    hits: int = 1


def make_client(client_id: str) -> mqtt.Client:
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id)
    return mqtt.Client(client_id=client_id)


class ExperimentServerApp:
    robot_stale_after_s = 6.0
    robot_colors = {
        "74": {"fill": "#d62828", "outline": "#7f1111", "path": "#111111"},
        "28": {"fill": "#2563eb", "outline": "#1e3a8a", "path": "#444444"},
    }
    default_robot_colors = {"fill": "#d62828", "outline": "#7f1111", "path": "#111111"}
    color_fills = {
        "red": "#d62828",
        "green": "#2a9d55",
        "blue": "#2563eb",
        "white": "#ffffff",
        "black": "#111111",
    }

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.topic_root = args.topic_root.rstrip("/")
        self.robot_configs = self._robot_configs_from_args(args)
        self._apply_robot_poses(args)
        self.robot_config_by_module = {
            config.module: config for config in self.robot_configs
        }
        self.robots = {
            config.module: RobotViewState(robot_id=config.module)
            for config in self.robot_configs
        }
        self.active_robot_id = self.robot_configs[0].module
        self.mqtt_statuses = {
            config.module: "connecting" for config in self.robot_configs
        }
        self.border_points: list[BorderPoint] = []
        self.border_lines: list[BorderLine] = []
        self.mountains: list[MountainDetection] = []
        self.colors: list[ColorDetection] = []
        self.craters: list[CraterDetection] = []
        self.border_complete = False
        self.map_sanitized = False
        self.area_started = False
        self.messages: queue.Queue[tuple[str, str, dict[str, Any]]] = queue.Queue()

        if args.tof_robot in self.robots:
            self.robots[args.tof_robot].tof_ready = True

        self.root = tk.Tk()
        self.root.title("VENUS Experiment Challenge Server v2")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.mqtt_var = tk.StringVar(value="MQTT: connecting")
        self.robot_vars = {
            config.module: tk.StringVar(
                value=f"Robot {config.module} ({config.username}): listening"
            )
            for config in self.robot_configs
        }
        self.coord_var = tk.StringVar(value="Coordinates: x=0.000 m, y=0.000 m")
        self.mode_var = tk.StringVar(value="Mode: idle")
        self.motion_var = tk.StringVar(value="Motion: moving=false, done=true")
        self.ir_var = tk.StringVar(value="IR: left=false, right=false")
        self.border_var = tk.StringVar(value="Border: 0 points, 0 lines")
        self.detection_var = tk.StringVar(value="Mountains: 0 detected")
        self.color_var = tk.StringVar(value="Colors: 0 detected")
        self.crater_var = tk.StringVar(value="Craters: 0 detected")
        self.last_var = tk.StringVar(value="Last message: never")

        self._build_ui()
        self.clients = {
            config.module: self._build_mqtt_client(config)
            for config in self.robot_configs
        }

    @staticmethod
    def _robot_configs_from_args(args: argparse.Namespace) -> list[RobotConfig]:
        if args.robots:
            return args.robots

        module = args.module or "28"
        username = args.username or f"robot_{module}_1"
        return [RobotConfig(module=module, username=username, password=args.password)]

    def _apply_robot_poses(self, args: argparse.Namespace) -> None:
        if args.robot_poses:
            configs_by_module = {config.module: config for config in self.robot_configs}
            for pose in args.robot_poses:
                config = configs_by_module.get(pose.module)
                if config is None:
                    continue
                config.map_x = pose.map_x
                config.map_y = pose.map_y
                config.map_theta = pose.map_theta
            return

        if len(self.robot_configs) != 2:
            return

        left, right = self.robot_configs
        half_spacing = args.facing_away_spacing / 2.0
        left.map_x = -half_spacing
        left.map_y = 0.0
        left.map_theta = 0.0
        right.map_x = half_spacing
        right.map_y = 0.0
        right.map_theta = math.pi

    def send_topic(self, module: str) -> str:
        return f"{self.topic_root}/{module}/send"

    def recv_topics(self, module: str) -> list[str]:
        return [
            f"{self.topic_root}/{module}/recv",
            f"{self.topic_root}/{module}/RECV",
        ]

    def _build_ui(self) -> None:
        top = tk.Frame(self.root, padx=12, pady=10)
        top.pack(fill=tk.X)

        tk.Label(top, textvariable=self.mqtt_var, anchor="w").grid(
            row=0, column=0, sticky="w"
        )
        for row, config in enumerate(self.robot_configs, start=1):
            tk.Label(top, textvariable=self.robot_vars[config.module], anchor="w").grid(
                row=row, column=0, sticky="w"
            )

        details_row = 1 + len(self.robot_configs)
        tk.Label(top, textvariable=self.coord_var, anchor="w").grid(
            row=details_row, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.mode_var, anchor="w").grid(
            row=details_row + 1, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.motion_var, anchor="w").grid(
            row=details_row + 2, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.ir_var, anchor="w").grid(
            row=details_row + 3, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.border_var, anchor="w").grid(
            row=details_row + 4, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.detection_var, anchor="w").grid(
            row=details_row + 5, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.color_var, anchor="w").grid(
            row=details_row + 6, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.crater_var, anchor="w").grid(
            row=details_row + 7, column=0, sticky="w"
        )
        tk.Label(top, textvariable=self.last_var, anchor="w").grid(
            row=details_row + 8, column=0, sticky="w"
        )

        controls = tk.Frame(top)
        controls.grid(
            row=0,
            column=1,
            rowspan=details_row + 9,
            padx=(18, 0),
            sticky="e",
        )

        self.start_button = tk.Button(
            controls,
            text="Start Border + Mountain",
            command=self.publish_start,
            width=16,
            state=tk.DISABLED,
        )
        self.start_button.grid(row=0, column=0, pady=2, sticky="ew")

        self.mountain_button = tk.Button(
            controls,
            text="Mountain Search",
            command=self.toggle_mountain_search,
            width=16,
            state=tk.DISABLED,
        )
        self.mountain_button.grid(row=1, column=0, pady=2, sticky="ew")

        self.close_border_button = tk.Button(
            controls,
            text="Close Border",
            command=self.manual_close_border,
            width=16,
            state=tk.DISABLED,
        )
        self.close_border_button.grid(row=2, column=0, pady=2, sticky="ew")

        self.stop_button = tk.Button(
            controls,
            text="Stop All",
            command=self.publish_stop,
            width=16,
            state=tk.DISABLED,
        )
        self.stop_button.grid(row=3, column=0, pady=2, sticky="ew")

        self.clear_button = tk.Button(
            controls,
            text="Clear Map",
            command=self.clear_map,
            width=16,
        )
        self.clear_button.grid(row=4, column=0, pady=2, sticky="ew")
        top.columnconfigure(0, weight=1)

        self.canvas_width = 760
        self.canvas_height = 460
        self.margin = 44
        self.map_size_m = 3.0
        self.meters_to_pixels = (
            min(
                self.canvas_width - (2 * self.margin),
                self.canvas_height - (2 * self.margin),
            )
            / self.map_size_m
        )
        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white",
            highlightthickness=1,
            highlightbackground="#999999",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))
        self._draw_map()

    def _build_mqtt_client(self, config: RobotConfig) -> mqtt.Client:
        client = make_client(f"venus-experiment-server-{config.module}")
        client.user_data_set(config.module)
        client.on_connect = self._on_connect
        client.on_connect_fail = self._on_connect_fail
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        if config.username or config.password:
            client.username_pw_set(config.username, config.password)
        return client

    def start(self) -> None:
        self.mqtt_var.set("MQTT: listening for broker/robots")
        for module, client in self.clients.items():
            try:
                client.connect_async(self.args.broker, self.args.port, keepalive=30)
                client.loop_start()
            except OSError as exc:
                self.mqtt_statuses[module] = f"connection failed: {exc}"
        self.root.after(100, self._poll_messages)
        self.root.after(1000, self._tick_refresh)
        self.root.mainloop()

    def close(self) -> None:
        try:
            for client in self.clients.values():
                client.loop_stop()
                client.disconnect()
        finally:
            self.root.destroy()

    def _publish_payload(self, robot: RobotViewState, payload: dict[str, Any]) -> bool:
        module = robot.robot_id
        client = self.clients.get(module)
        if client is None:
            self.mqtt_statuses[module] = "no MQTT client for this robot"
            return False

        publish_results = []
        encoded = json.dumps(payload, separators=(",", ":"))
        for topic in self.recv_topics(module):
            info = client.publish(topic, encoded, qos=1)
            publish_results.append(f"{topic} rc={info.rc}")
        self.mqtt_statuses[module] = "published " + "; ".join(publish_results)
        return True

    def publish_start(self) -> None:
        border_robot = self._border_robot()
        if border_robot is None:
            self.mqtt_var.set("MQTT: no non-ToF border robot visible yet")
            return

        self.border_complete = False
        self.map_sanitized = False
        self.area_started = False
        border_robot.status = "start border sent"
        self._publish_payload(
            border_robot, {"cmd": "start_border", "mode": "border_search"}
        )

        tof_robot = self._tof_robot()
        if tof_robot is not None and tof_robot.robot_id != border_robot.robot_id:
            tof_robot.status = "mountain search sent"
            self._publish_payload(
                tof_robot, {"cmd": "set_mode", "mode": "mountain_detect"}
            )

        self.active_robot_id = border_robot.robot_id
        self._refresh_labels()

    def toggle_mountain_search(self) -> None:
        robot = self._tof_robot()
        if robot is None:
            self.mqtt_var.set("MQTT: no ToF robot visible yet")
            return

        if robot.mode == "mountain_detect":
            robot.status = "stop mountain sent"
            self._publish_payload(robot, {"cmd": "stop"})
            self._refresh_labels()
            return

        robot.status = "mountain search sent"
        self.active_robot_id = robot.robot_id
        self._publish_payload(robot, {"cmd": "set_mode", "mode": "mountain_detect"})
        self._refresh_labels()

    def publish_stop(self) -> None:
        robots = self._commandable_robots()
        if not robots:
            self.mqtt_var.set("MQTT: no robot visible yet")
            return

        for robot in robots:
            robot.status = "stop sent"
            self._publish_payload(robot, {"cmd": "stop"})
        self.area_started = False
        self.craters.clear()
        self._refresh_labels()
        self._draw_map()

    def manual_close_border(self) -> None:
        polygon = self._border_polygon()
        if len(polygon) < 3:
            self.mqtt_statuses["map"] = "need at least 3 border points to close"
            self._refresh_labels()
            return

        first_x, first_y = polygon[0]
        last_x, last_y = polygon[-1]
        if math.hypot(last_x - first_x, last_y - first_y) > 0.01:
            self.border_points.append(
                BorderPoint(
                    x=first_x,
                    y=first_y,
                    theta=0.0,
                    event="manual_close",
                    detected_at=time.time(),
                )
            )

        for robot in self._connected_robots():
            if robot.mode in {
                "border_search",
                "align_line",
                "turn_away_from_line",
                "leave_line",
                "turn_to_line",
                "border_map",
            }:
                robot.status = "manual border close stop sent"
                robot.mode = "idle"
                robot.moving = False
                self._publish_payload(robot, {"cmd": "stop"})

        self.border_complete = True
        self.map_sanitized = False
        self.area_started = False
        self.mqtt_statuses["map"] = "border manually closed"
        self._maybe_sanitize_map()
        self.mqtt_statuses["map"] = "border manually closed; area mode requested"
        self._refresh_labels()
        self._draw_map()

    def clear_map(self) -> None:
        self.border_points.clear()
        self.border_lines.clear()
        self.mountains.clear()
        self.colors.clear()
        self.craters.clear()
        self.border_complete = False
        self.map_sanitized = False
        self.area_started = False
        for robot in self.robots.values():
            robot.path.clear()
        self._refresh_labels()
        self._draw_map()

    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: Any, rc: int) -> None:
        del flags
        module = str(userdata)
        if rc != 0:
            self.messages.put(
                (
                    "mqtt_status",
                    module,
                    {"status": f"connect failed rc={rc}, retrying"},
                )
            )
            return
        client.subscribe(self.send_topic(module))
        self.messages.put(
            (
                "mqtt_status",
                module,
                {"status": f"connected, subscribed {self.send_topic(module)}"},
            )
        )

    def _on_connect_fail(self, client: mqtt.Client, userdata: Any) -> None:
        del client
        self.messages.put(
            ("mqtt_status", str(userdata), {"status": "connect failed, retrying"})
        )

    def _on_disconnect(self, client: mqtt.Client, userdata: Any, rc: int) -> None:
        del client
        self.messages.put(
            (
                "mqtt_status",
                str(userdata),
                {"status": f"disconnected rc={rc}, retrying"},
            )
        )

    def _on_message(
        self, client: mqtt.Client, userdata: Any, message: mqtt.MQTTMessage
    ) -> None:
        del client
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            module = self._module_from_topic(message.topic, str(userdata))
            self.messages.put((message.topic, module, payload))

    def _poll_messages(self) -> None:
        while True:
            try:
                topic, module, payload = self.messages.get_nowait()
            except queue.Empty:
                break
            self._handle_message(topic, module, payload)

        self.root.after(100, self._poll_messages)

    def _tick_refresh(self) -> None:
        self._refresh_labels()
        self.root.after(1000, self._tick_refresh)

    def _handle_message(
        self, topic: str, module: str, payload: dict[str, Any]
    ) -> None:
        if topic == "mqtt_status":
            self.mqtt_statuses[module] = str(payload.get("status", "unknown"))
            self._refresh_labels()
            return

        msg_type = str(payload.get("type", ""))
        robot_id = str(payload.get("robot", module))
        robot = self.robots.setdefault(robot_id, RobotViewState(robot_id=robot_id))
        self.active_robot_id = robot_id
        robot.last_message_at = time.time()

        if msg_type == "status":
            robot.status = str(payload.get("status", "status"))
            if robot.status in {
                "border_map_complete",
                "border_loop_closed",
                "border_path_revisited",
            }:
                self.border_complete = True
            robot.ready = robot.status in {
                "idle",
                "border_search",
                "ready_no_tof",
                "mountain_scan_complete",
                "border_map_complete",
                "border_loop_closed",
                "border_path_revisited",
            }
            self._update_robot_pose(robot, payload)
        elif msg_type == "border_point":
            robot.status = str(payload.get("event", "border point"))
            self._update_robot_pose(robot, payload)
            self._add_border_point(robot, payload)
        elif msg_type == "border_line":
            robot.status = "line angle found"
            self._update_robot_pose(robot, payload)
            self._add_border_line(robot, payload)
        elif msg_type == "mountain_detection":
            robot.status = "mountain detected"
            robot.tof_ready = True
            self._update_robot_pose(robot, payload)
            self._add_mountain_detection(robot, payload)
        elif msg_type == "color_detection":
            robot.status = f"{payload.get('color', 'color')} detected"
            self._update_robot_pose(robot, payload)
            self._add_color_detection(robot, payload)
        elif msg_type == "crater_detection":
            robot.status = "crater detected"
            self._update_robot_pose(robot, payload)
            self._add_crater_detection(robot, payload)
        elif msg_type == "ready":
            robot.ready = True
            robot.status = str(payload.get("status", "ready"))
            self._update_robot_pose(robot, payload)

        self._update_tof_state(robot)
        self._maybe_sanitize_map()
        self._maybe_restart_idle_area_robot(robot)
        self._maybe_send_map_hint(robot)
        self._refresh_labels()
        self._draw_map()

    def _module_from_topic(self, topic: str, fallback: str) -> str:
        parts = [part for part in topic.split("/") if part]
        root_parts = [part for part in self.topic_root.split("/") if part]
        if not root_parts:
            return fallback
        for index in range(0, len(parts) - len(root_parts)):
            if parts[index : index + len(root_parts)] == root_parts:
                module_index = index + len(root_parts)
                if module_index < len(parts):
                    return parts[module_index]
        return fallback

    def _update_robot_pose(
        self, robot: RobotViewState, payload: dict[str, Any]
    ) -> None:
        robot.local_x = self._float_value(payload.get("coordX"), robot.local_x)
        robot.local_y = self._float_value(payload.get("coordY"), robot.local_y)
        robot.local_theta = self._float_value(
            payload.get("theta"), robot.local_theta
        )
        robot.x, robot.y = self._transform_point(
            robot.robot_id, robot.local_x, robot.local_y
        )
        robot.theta = self._transform_theta(robot.robot_id, robot.local_theta)
        robot.mode = str(payload.get("mode", robot.mode))
        robot.moving = self._bool_value(payload.get("moving"), robot.moving)
        robot.done = self._bool_value(payload.get("done"), robot.done)
        robot.ir_left = self._bool_value(payload.get("irLeft"), robot.ir_left)
        robot.ir_right = self._bool_value(payload.get("irRight"), robot.ir_right)

        point = (robot.x, robot.y)
        if not robot.path or robot.path[-1] != point:
            robot.path.append(point)

    def _update_tof_state(self, robot: RobotViewState) -> None:
        if self.args.tof_robot == robot.robot_id:
            robot.tof_ready = True
            return

        status = robot.status.lower()
        if status in {"ready_no_tof", "tof_unavailable"}:
            robot.tof_ready = False
        elif status in {"mode_mountain_detect", "mountain_scan_complete"}:
            robot.tof_ready = True

    def _add_border_point(
        self, robot: RobotViewState, payload: dict[str, Any]
    ) -> None:
        local_theta = self._float_value(payload.get("lineTheta"), robot.local_theta)
        theta = self._transform_theta(robot.robot_id, local_theta)
        local_x = self._float_value(payload.get("borderX"), robot.local_x)
        local_y = self._float_value(payload.get("borderY"), robot.local_y)
        x, y = self._transform_point(robot.robot_id, local_x, local_y)
        event = str(payload.get("event", "point"))
        self.border_points.append(
            BorderPoint(
                x=x,
                y=y,
                theta=theta,
                event=event,
                detected_at=time.time(),
                ir_left=self._bool_value(payload.get("irLeft"), False),
                ir_right=self._bool_value(payload.get("irRight"), False),
            )
        )

    def _add_border_line(self, robot: RobotViewState, payload: dict[str, Any]) -> None:
        local_x = self._float_value(payload.get("borderX"), robot.local_x)
        local_y = self._float_value(payload.get("borderY"), robot.local_y)
        x, y = self._transform_point(robot.robot_id, local_x, local_y)
        local_theta = self._float_value(payload.get("lineTheta"), robot.local_theta)
        self.border_lines.append(
            BorderLine(
                x=x,
                y=y,
                theta=self._transform_theta(robot.robot_id, local_theta),
                detected_at=time.time(),
            )
        )

    def _add_mountain_detection(
        self, robot: RobotViewState, payload: dict[str, Any]
    ) -> None:
        local_theta = self._float_value(payload.get("theta"), robot.local_theta)
        theta = self._transform_theta(robot.robot_id, local_theta)
        distance_m = self._float_value(payload.get("distance_m"), 0.0)
        fallback_local_x = robot.local_x + (distance_m * math.cos(local_theta))
        fallback_local_y = robot.local_y + (distance_m * math.sin(local_theta))
        local_det_x = self._float_value(payload.get("detX"), fallback_local_x)
        local_det_y = self._float_value(payload.get("detY"), fallback_local_y)
        det_x, det_y = self._transform_point(
            robot.robot_id, local_det_x, local_det_y
        )
        now = time.time()

        existing = self._nearest_mountain(det_x, det_y)
        if existing is not None:
            next_hits = existing.hits + 1
            existing.x = ((existing.x * existing.hits) + det_x) / next_hits
            existing.y = ((existing.y * existing.hits) + det_y) / next_hits
            existing.distance_m = (
                (existing.distance_m * existing.hits) + distance_m
            ) / next_hits
            existing.theta = theta
            existing.detected_at = now
            existing.hits = next_hits
            return

        self.mountains.append(
            MountainDetection(
                x=det_x,
                y=det_y,
                distance_m=distance_m,
                theta=theta,
                detected_at=now,
            )
        )

    def _add_color_detection(
        self, robot: RobotViewState, payload: dict[str, Any]
    ) -> None:
        local_x = self._float_value(payload.get("colorX"), robot.local_x)
        local_y = self._float_value(payload.get("colorY"), robot.local_y)
        x, y = self._transform_point(robot.robot_id, local_x, local_y)
        color = str(payload.get("color", "black")).lower()
        red = self._int_value(payload.get("red"), 0)
        green = self._int_value(payload.get("green"), 0)
        blue = self._int_value(payload.get("blue"), 0)
        samples = self._int_value(payload.get("samples"), 0)
        now = time.time()

        existing = self._nearest_color(x, y)
        if existing is not None:
            next_hits = existing.hits + 1
            existing.x = ((existing.x * existing.hits) + x) / next_hits
            existing.y = ((existing.y * existing.hits) + y) / next_hits
            existing.color = color
            existing.red = red
            existing.green = green
            existing.blue = blue
            existing.samples = samples
            existing.detected_at = now
            existing.hits = next_hits
            return

        self.colors.append(
            ColorDetection(
                x=x,
                y=y,
                color=color,
                red=red,
                green=green,
                blue=blue,
                samples=samples,
                detected_at=now,
            )
        )

    def _add_crater_detection(
        self, robot: RobotViewState, payload: dict[str, Any]
    ) -> None:
        local_x = self._float_value(payload.get("craterX"), robot.local_x)
        local_y = self._float_value(payload.get("craterY"), robot.local_y)
        x, y = self._transform_point(robot.robot_id, local_x, local_y)
        now = time.time()

        existing = self._nearest_crater(x, y)
        if existing is not None:
            next_hits = existing.hits + 1
            existing.x = ((existing.x * existing.hits) + x) / next_hits
            existing.y = ((existing.y * existing.hits) + y) / next_hits
            existing.detected_at = now
            existing.hits = next_hits
            return

        self.craters.append(CraterDetection(x=x, y=y, detected_at=now))

    def _nearest_mountain(self, x_m: float, y_m: float) -> Optional[MountainDetection]:
        closest: Optional[MountainDetection] = None
        closest_distance = self.args.mountain_merge_radius

        for mountain in self.mountains:
            distance = math.hypot(mountain.x - x_m, mountain.y - y_m)
            if distance <= closest_distance:
                closest = mountain
                closest_distance = distance

        return closest

    def _nearest_crater(self, x_m: float, y_m: float) -> Optional[CraterDetection]:
        closest: Optional[CraterDetection] = None
        closest_distance = self.args.crater_merge_radius

        for crater in self.craters:
            distance = math.hypot(crater.x - x_m, crater.y - y_m)
            if distance <= closest_distance:
                closest = crater
                closest_distance = distance

        return closest

    def _nearest_color(self, x_m: float, y_m: float) -> Optional[ColorDetection]:
        closest: Optional[ColorDetection] = None
        closest_distance = self.args.color_merge_radius

        for color in self.colors:
            distance = math.hypot(color.x - x_m, color.y - y_m)
            if distance <= closest_distance:
                closest = color
                closest_distance = distance

        return closest

    def _nearest_known_object(
        self, robot: RobotViewState
    ) -> tuple[float, float, str]:
        best_distance = 0.0
        best_bearing = 0.0
        best_source = "none"

        objects: list[tuple[float, float, str]] = []
        objects.extend(
            (mountain.x, mountain.y, "mountain") for mountain in self.mountains
        )
        objects.extend((color.x, color.y, "color") for color in self.colors)
        objects.extend((crater.x, crater.y, "crater") for crater in self.craters)
        hint_radius = max(
            self.args.map_hint_radius,
            self.args.robot_separation_radius,
        )
        bearing_limit = math.radians(self.args.map_hint_bearing_deg)
        for other in self.robots.values():
            if other.robot_id == robot.robot_id or not self._robot_is_visible(other):
                continue
            objects.append((other.x, other.y, "robot"))

        for x_m, y_m, source in objects:
            local_x, local_y = self._inverse_transform_point(robot.robot_id, x_m, y_m)
            dx = local_x - robot.local_x
            dy = local_y - robot.local_y
            distance = math.hypot(dx, dy)
            if distance <= 1e-9 or distance > hint_radius:
                continue
            bearing = self._normalize_angle(math.atan2(dy, dx) - robot.local_theta)
            if abs(bearing) > bearing_limit:
                continue
            if best_distance == 0.0 or distance < best_distance:
                best_distance = distance
                best_bearing = bearing
                best_source = source

        return best_distance, best_bearing, best_source

    def _nearest_border_distance(self, robot: RobotViewState) -> float:
        if not self.border_points:
            return 999.0
        return min(
            math.hypot(point.x - robot.x, point.y - robot.y)
            for point in self.border_points
        )

    def _maybe_send_map_hint(self, robot: RobotViewState) -> None:
        if robot.mode not in {"area_sweep", "area_avoid"}:
            return
        if not self._robot_is_visible(robot) or robot.robot_id not in self.clients:
            return

        obstacle_distance, obstacle_bearing, obstacle_source = (
            self._nearest_known_object(robot)
        )
        payload = {
            "cmd": "map_hint",
            "type": "map_hint",
            "obstacleDistance": round(obstacle_distance, 4),
            "obstacleBearing": round(obstacle_bearing, 4),
            "obstacleSource": obstacle_source,
            "nearestBorderDistance": round(self._nearest_border_distance(robot), 4),
        }
        self._publish_payload(robot, payload)

    def _transform_point(
        self, robot_id: str, local_x: float, local_y: float
    ) -> tuple[float, float]:
        config = self.robot_config_by_module.get(robot_id)
        if config is None:
            return local_x, local_y

        cos_t = math.cos(config.map_theta)
        sin_t = math.sin(config.map_theta)
        return (
            config.map_x + (local_x * cos_t) - (local_y * sin_t),
            config.map_y + (local_x * sin_t) + (local_y * cos_t),
        )

    def _inverse_transform_point(
        self, robot_id: str, x_m: float, y_m: float
    ) -> tuple[float, float]:
        config = self.robot_config_by_module.get(robot_id)
        if config is None:
            return x_m, y_m

        dx = x_m - config.map_x
        dy = y_m - config.map_y
        cos_t = math.cos(config.map_theta)
        sin_t = math.sin(config.map_theta)
        return (
            (dx * cos_t) + (dy * sin_t),
            (-dx * sin_t) + (dy * cos_t),
        )

    def _transform_theta(self, robot_id: str, local_theta: float) -> float:
        config = self.robot_config_by_module.get(robot_id)
        if config is None:
            return self._normalize_angle(local_theta)
        return self._normalize_angle(local_theta + config.map_theta)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _float_value(value: Any, fallback: float) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    @staticmethod
    def _bool_value(value: Any, fallback: bool) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            lowered = value.lower()
            if lowered in {"true", "1", "yes"}:
                return True
            if lowered in {"false", "0", "no"}:
                return False
        return fallback

    @staticmethod
    def _int_value(value: Any, fallback: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return fallback

    def _active_robot(self) -> RobotViewState:
        return self.robots.get(
            self.active_robot_id, self.robots[self.robot_configs[0].module]
        )

    def _connected_robots(self) -> list[RobotViewState]:
        return [
            robot
            for robot in self.robots.values()
            if self._robot_is_visible(robot) and robot.robot_id in self.clients
        ]

    def _commandable_robots(self) -> list[RobotViewState]:
        return [
            self.robots[config.module]
            for config in self.robot_configs
            if config.module in self.clients
        ]

    def _known_commandable_robots(self) -> list[RobotViewState]:
        return [
            robot
            for robot in self._commandable_robots()
            if robot.last_message_at is not None
        ]

    def _tof_robot(self) -> Optional[RobotViewState]:
        if self.args.tof_robot:
            robot = self.robots.get(self.args.tof_robot)
            if robot is not None and self._robot_is_visible(robot):
                return robot
            return None

        connected = self._connected_robots()
        if not connected:
            return None

        known_tof = [robot for robot in connected if robot.tof_ready is True]
        if known_tof:
            return max(known_tof, key=lambda robot: robot.last_message_at or 0.0)

        possible_tof = [robot for robot in connected if robot.tof_ready is not False]
        if possible_tof:
            return max(possible_tof, key=lambda robot: robot.last_message_at or 0.0)

        return None

    def _border_robot(self) -> Optional[RobotViewState]:
        connected = self._connected_robots()
        if not connected:
            return None

        tof_robot_id = self.args.tof_robot
        if tof_robot_id:
            for robot in connected:
                if robot.robot_id != tof_robot_id:
                    return robot
            return max(connected, key=lambda robot: robot.last_message_at or 0.0)

        known_no_tof = [robot for robot in connected if robot.tof_ready is False]
        if known_no_tof:
            return max(known_no_tof, key=lambda robot: robot.last_message_at or 0.0)

        tof_robot = self._tof_robot()
        non_tof = [
            robot
            for robot in connected
            if tof_robot is None or robot.robot_id != tof_robot.robot_id
        ]
        if non_tof:
            return max(non_tof, key=lambda robot: robot.last_message_at or 0.0)

        return None

    def _maybe_sanitize_map(self) -> None:
        if self.map_sanitized or not self.border_complete:
            return

        polygon = self._border_polygon()
        if len(polygon) < 3:
            return

        old_mountains = len(self.mountains)
        old_colors = len(self.colors)
        old_craters = len(self.craters)
        self.mountains = [
            mountain
            for mountain in self.mountains
            if self._point_in_polygon(mountain.x, mountain.y, polygon)
        ]
        self.colors = [
            color
            for color in self.colors
            if self._point_in_polygon(color.x, color.y, polygon)
        ]
        self.craters = [
            crater
            for crater in self.craters
            if self._point_in_polygon(crater.x, crater.y, polygon)
        ]
        removed = (old_mountains - len(self.mountains)) + (
            old_colors - len(self.colors)
        ) + (old_craters - len(self.craters))
        self.map_sanitized = True
        self.mqtt_statuses["map"] = (
            f"sanitized, removed {removed} outside objects"
        )
        self._maybe_start_area_mode(polygon)

    def _maybe_start_area_mode(self, polygon: list[tuple[float, float]]) -> None:
        if self.area_started:
            return

        ordered_robots = self._ordered_area_robots()
        if not ordered_robots:
            return

        for robot in ordered_robots:
            if robot.mode == "mountain_detect" or robot.moving:
                continue
            self._send_area_sweep(robot, polygon, "area sweep sent", force=True)

        self.area_started = True

    def _maybe_restart_idle_area_robot(self, robot: RobotViewState) -> None:
        if not (self.border_complete and self.map_sanitized and self.area_started):
            return
        if robot.robot_id not in self.clients or not self._robot_is_visible(robot):
            return
        if robot.moving or robot.mode != "idle":
            return

        polygon = self._border_polygon()
        if len(polygon) < 3:
            return

        self._send_area_sweep(robot, polygon, "area sweep resumed")

    def _ordered_area_robots(self) -> list[RobotViewState]:
        known_ids = {
            robot.robot_id
            for robot in self._known_commandable_robots()
        }
        ordered = [
            self.robots[config.module]
            for config in self.robot_configs
            if config.module in known_ids
        ]
        return ordered or [
            robot
            for robot in self._known_commandable_robots()
            if robot.robot_id in self.clients
        ]

    def _area_payload_for_robot(
        self, robot: RobotViewState, polygon: list[tuple[float, float]]
    ) -> Optional[dict[str, Any]]:
        ordered_robots = self._ordered_area_robots()
        if robot.robot_id not in {
            area_robot.robot_id for area_robot in ordered_robots
        }:
            return None

        min_x = min(x for x, _ in polygon)
        max_x = max(x for x, _ in polygon)
        min_y = min(y for _, y in polygon)
        max_y = max(y for _, y in polygon)
        split_x_axis = (max_x - min_x) >= (max_y - min_y)
        count = len(ordered_robots)
        index = next(
            i for i, area_robot in enumerate(ordered_robots)
            if area_robot.robot_id == robot.robot_id
        )

        if split_x_axis:
            span = (max_x - min_x) / count
            part_min_x = min_x + (index * span)
            part_max_x = min_x + ((index + 1) * span)
            world_bounds = (part_min_x, part_max_x, min_y, max_y)
        else:
            span = (max_y - min_y) / count
            part_min_y = min_y + (index * span)
            part_max_y = min_y + ((index + 1) * span)
            world_bounds = (min_x, max_x, part_min_y, part_max_y)

        local_bounds = self._local_bounds_from_world_rect(
            robot.robot_id, world_bounds
        )
        return {
            "cmd": "set_mode",
            "mode": "area_sweep",
            "areaMinX": round(local_bounds[0], 4),
            "areaMaxX": round(local_bounds[1], 4),
            "areaMinY": round(local_bounds[2], 4),
            "areaMaxY": round(local_bounds[3], 4),
            "sweepTheta": 0.0,
            "rowSpacing": self.args.area_row_spacing,
            "sweepStep": self.args.area_sweep_step,
        }

    def _send_area_sweep(
        self,
        robot: RobotViewState,
        polygon: list[tuple[float, float]],
        status: str,
        force: bool = False,
    ) -> None:
        now = time.time()
        if (
            not force
            and now - robot.last_area_command_at < self.args.area_command_cooldown
        ):
            return

        payload = self._area_payload_for_robot(robot, polygon)
        if payload is None:
            return

        robot.status = status
        robot.mode = "area_sweep"
        robot.last_area_command_at = now
        self._publish_payload(robot, payload)

    def _local_bounds_from_world_rect(
        self, robot_id: str, bounds: tuple[float, float, float, float]
    ) -> tuple[float, float, float, float]:
        min_x, max_x, min_y, max_y = bounds
        corners = [
            self._inverse_transform_point(robot_id, min_x, min_y),
            self._inverse_transform_point(robot_id, min_x, max_y),
            self._inverse_transform_point(robot_id, max_x, min_y),
            self._inverse_transform_point(robot_id, max_x, max_y),
        ]
        local_xs = [x for x, _ in corners]
        local_ys = [y for _, y in corners]
        return min(local_xs), max(local_xs), min(local_ys), max(local_ys)

    def _border_polygon(self) -> list[tuple[float, float]]:
        polygon: list[tuple[float, float]] = []
        for point in self.border_points:
            if point.event not in {
                "map_start",
                "map_point",
                "map_tape",
                "corner_hit",
                "tape_hit",
                "manual_close",
            }:
                continue
            next_point = (point.x, point.y)
            if not polygon or math.hypot(
                polygon[-1][0] - next_point[0], polygon[-1][1] - next_point[1]
            ) > 0.01:
                polygon.append(next_point)
        return polygon

    @staticmethod
    def _point_in_polygon(
        x: float, y: float, polygon: list[tuple[float, float]]
    ) -> bool:
        inside = False
        j = len(polygon) - 1
        for i, point_i in enumerate(polygon):
            xi, yi = point_i
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (
                x < ((xj - xi) * (y - yi) / ((yj - yi) or 1e-12)) + xi
            ):
                inside = not inside
            j = i
        return inside

    def _robot_is_visible(self, robot: RobotViewState) -> bool:
        if robot.last_message_at is None:
            return False
        return time.time() - robot.last_message_at <= self.robot_stale_after_s

    def _refresh_labels(self) -> None:
        mqtt_summary = "; ".join(
            f"{module}: {status}" for module, status in self.mqtt_statuses.items()
        )
        self.mqtt_var.set(f"MQTT: {mqtt_summary}")

        for config in self.robot_configs:
            robot = self.robots[config.module]
            visible_text = "visible" if self._robot_is_visible(robot) else "not visible"
            ready_text = "ready" if robot.ready else "not ready"
            tof_text = self._tof_label(robot)
            if robot.last_message_at is None:
                age_text = "never"
            else:
                age_text = f"{time.time() - robot.last_message_at:.1f} s ago"
            self.robot_vars[config.module].set(
                f"Robot {config.module} ({config.username}): {visible_text}, "
                f"{ready_text}, {tof_text}, mode={robot.mode}, "
                f"status={robot.status}, last={age_text}"
            )

        robot = self._active_robot()
        self.coord_var.set(
            f"Active robot {robot.robot_id}: x={robot.x:.3f} m, "
            f"y={robot.y:.3f} m, theta={robot.theta:.3f} rad"
        )
        self.mode_var.set(f"Mode: {robot.mode}")
        self.motion_var.set(
            f"Motion: moving={str(robot.moving).lower()}, "
            f"done={str(robot.done).lower()}"
        )
        self.ir_var.set(
            f"IR: left={str(robot.ir_left).lower()}, "
            f"right={str(robot.ir_right).lower()}"
        )
        self.border_var.set(
            f"Border: {len(self.border_points)} points, "
            f"{len(self.border_lines)} line angles"
        )
        total_hits = sum(mountain.hits for mountain in self.mountains)
        self.detection_var.set(
            f"Mountains: {len(self.mountains)} shown, {total_hits} detections"
        )
        total_color_hits = sum(color.hits for color in self.colors)
        self.color_var.set(
            f"Colors: {len(self.colors)} squares, {total_color_hits} detections"
        )
        total_crater_hits = sum(crater.hits for crater in self.craters)
        self.crater_var.set(
            f"Craters: {len(self.craters)} shown, {total_crater_hits} detections"
        )
        if robot.last_message_at is None:
            self.last_var.set("Last message: never")
        else:
            age = time.time() - robot.last_message_at
            self.last_var.set(f"Last message: {age:.1f} s ago")

        connected = self._connected_robots()
        tof_robot = self._tof_robot()
        self.start_button.config(
            state=tk.NORMAL if self._border_robot() is not None else tk.DISABLED
        )
        self.mountain_button.config(
            text=(
                "Stop Mountain"
                if tof_robot is not None and tof_robot.mode == "mountain_detect"
                else "Mountain Search"
            ),
            state=tk.NORMAL if tof_robot is not None else tk.DISABLED,
        )
        self.close_border_button.config(
            state=tk.NORMAL if len(self._border_polygon()) >= 3 else tk.DISABLED
        )
        self.stop_button.config(state=tk.NORMAL if connected else tk.DISABLED)

    def _tof_label(self, robot: RobotViewState) -> str:
        if robot.tof_ready is True:
            return "tof=yes"
        if robot.tof_ready is False:
            return "tof=no"
        return "tof=unknown"

    def _world_to_canvas(self, x_m: float, y_m: float) -> tuple[float, float]:
        origin_x = self.canvas_width / 2.0
        origin_y = self.canvas_height / 2.0
        return (
            origin_x - (x_m * self.meters_to_pixels),
            origin_y - (y_m * self.meters_to_pixels),
        )

    def _draw_map(self) -> None:
        self.canvas.delete("all")
        self._draw_axes()

        for mountain in self.mountains:
            self._draw_mountain(mountain)

        for crater in self.craters:
            self._draw_crater(crater)

        if len(self.border_points) >= 2:
            coords: list[float] = []
            for point in self.border_points:
                x_px, y_px = self._world_to_canvas(point.x, point.y)
                coords.extend([x_px, y_px])
            self.canvas.create_line(*coords, fill="#111111", width=4, smooth=False)

        for line in self.border_lines:
            self._draw_border_line(line)

        for point in self.border_points:
            self._draw_border_point(point)

        for robot in self.robots.values():
            if len(robot.path) < 2:
                continue
            coords = []
            for x_m, y_m in robot.path:
                x_px, y_px = self._world_to_canvas(x_m, y_m)
                coords.extend([x_px, y_px])
            colors = self._robot_colors(robot.robot_id)
            width = 3 if robot.robot_id == self.active_robot_id else 2
            self.canvas.create_line(
                *coords, fill=colors["path"], width=width, smooth=False
            )

        for detection in self.colors:
            self._draw_color_detection(detection)

        for robot in self.robots.values():
            self._draw_robot(robot)

    def _draw_axes(self) -> None:
        x0, y0 = self._world_to_canvas(0.0, 0.0)
        self.canvas.create_line(0, y0, self.canvas_width, y0, fill="#eeeeee")
        self.canvas.create_line(x0, 0, x0, self.canvas_height, fill="#eeeeee")

    def _draw_border_point(self, point: BorderPoint) -> None:
        x_px, y_px = self._world_to_canvas(point.x, point.y)
        radius = 5 if point.event in {"map_point", "map_tape"} else 7
        fill = "#111111" if point.event in {"map_point", "map_tape"} else "#f77f00"
        self.canvas.create_oval(
            x_px - radius,
            y_px - radius,
            x_px + radius,
            y_px + radius,
            fill=fill,
            outline="",
        )

    def _draw_border_line(self, line: BorderLine) -> None:
        half_len_m = 0.35
        x1 = line.x - (half_len_m * math.cos(line.theta))
        y1 = line.y - (half_len_m * math.sin(line.theta))
        x2 = line.x + (half_len_m * math.cos(line.theta))
        y2 = line.y + (half_len_m * math.sin(line.theta))
        x1_px, y1_px = self._world_to_canvas(x1, y1)
        x2_px, y2_px = self._world_to_canvas(x2, y2)
        self.canvas.create_line(
            x1_px,
            y1_px,
            x2_px,
            y2_px,
            fill="#15616d",
            width=3,
            dash=(6, 4),
        )

    def _draw_robot(self, robot: RobotViewState) -> None:
        colors = self._robot_colors(robot.robot_id)
        robot_x, robot_y = self._world_to_canvas(robot.x, robot.y)
        radius = 11 if robot.robot_id == self.active_robot_id else 9
        heading_x = robot_x - (18 * math.cos(robot.theta))
        heading_y = robot_y - (18 * math.sin(robot.theta))
        self.canvas.create_oval(
            robot_x - radius,
            robot_y - radius,
            robot_x + radius,
            robot_y + radius,
            fill=colors["fill"],
            outline=colors["outline"],
            width=2,
        )
        self.canvas.create_line(
            robot_x, robot_y, heading_x, heading_y, fill=colors["outline"], width=3
        )
        self.canvas.create_text(
            robot_x,
            robot_y - radius - 5,
            text=robot.robot_id,
            anchor="s",
            fill=colors["outline"],
        )

    def _robot_colors(self, robot_id: str) -> dict[str, str]:
        return self.robot_colors.get(robot_id, self.default_robot_colors)

    def _draw_mountain(self, mountain: MountainDetection) -> None:
        x_px, y_px = self._world_to_canvas(mountain.x, mountain.y)
        size = min(30, 13 + (mountain.hits - 1) * 3)
        self.canvas.create_polygon(
            x_px,
            y_px - size,
            x_px - size,
            y_px + size,
            x_px + size,
            y_px + size,
            fill="#8f8f8f",
            outline="#555555",
            width=2,
        )
        self.canvas.create_text(
            x_px,
            y_px + size + 10,
            text=f"{mountain.distance_m:.2f} m x{mountain.hits}",
            fill="#555555",
            font=("TkDefaultFont", 9),
        )

    def _draw_crater(self, crater: CraterDetection) -> None:
        x_px, y_px = self._world_to_canvas(crater.x, crater.y)
        radius = min(18, 9 + (crater.hits - 1) * 2)
        self.canvas.create_oval(
            x_px - radius,
            y_px - radius,
            x_px + radius,
            y_px + radius,
            fill="#111111",
            outline="#f77f00",
            width=2,
        )
        self.canvas.create_text(
            x_px,
            y_px + radius + 10,
            text=f"crater x{crater.hits}",
            fill="#333333",
            font=("TkDefaultFont", 9),
        )

    def _draw_color_detection(self, detection: ColorDetection) -> None:
        x_px, y_px = self._world_to_canvas(detection.x, detection.y)
        size = 12
        fill = self.color_fills.get(detection.color, "#111111")
        outline = "#111111" if detection.color == "white" else "#ffffff"
        self.canvas.create_rectangle(
            x_px - size,
            y_px - size,
            x_px + size,
            y_px + size,
            fill=fill,
            outline=outline,
            width=2,
        )
        self.canvas.create_text(
            x_px,
            y_px + size + 10,
            text=(
                f"{detection.color} "
                f"{detection.red}/{detection.green}/{detection.blue} "
                f"x{detection.hits}"
            ),
            fill="#333333",
            font=("TkDefaultFont", 9),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="mqtt.ics.ele.tue.nl")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic-root", default="/pynqbridge")
    parser.add_argument(
        "--robot",
        action="append",
        dest="robots",
        default=[],
        type=parse_robot_config,
        metavar="MODULE:USERNAME[:PASSWORD]",
        help="Robot login to monitor. Can be used more than once.",
    )
    parser.add_argument(
        "--tof-robot",
        default=None,
        help="Module number of the robot with ToF. If omitted, infer from status.",
    )
    parser.add_argument(
        "--robot-pose",
        action="append",
        dest="robot_poses",
        default=[],
        type=parse_robot_pose,
        metavar="MODULE:X:Y:THETA_DEG",
        help="Map start pose for a robot. Theta is degrees in the shared map.",
    )
    parser.add_argument(
        "--facing-away-spacing",
        type=float,
        default=0.30,
        help=(
            "Default distance in meters between two robots when no --robot-pose is "
            "given. First robot faces 0 deg, second faces 180 deg."
        ),
    )
    parser.add_argument(
        "--module",
        default=None,
        help="Single robot module to monitor, kept for older command lines.",
    )
    parser.add_argument(
        "--username",
        default=None,
        help="Single robot MQTT username, kept for older command lines.",
    )
    parser.add_argument("--password", default="brO4r8d9")
    parser.add_argument(
        "--mountain-merge-radius",
        type=float,
        default=0.3,
        help="Merge ToF detections within this many meters into one mountain.",
    )
    parser.add_argument(
        "--crater-merge-radius",
        type=float,
        default=0.18,
        help="Merge crater detections within this many meters into one crater.",
    )
    parser.add_argument(
        "--color-merge-radius",
        type=float,
        default=0.16,
        help="Merge color detections within this many meters into one block.",
    )
    parser.add_argument(
        "--map-hint-radius",
        type=float,
        default=0.45,
        help="Send the nearest known object within this many meters as a map hint.",
    )
    parser.add_argument(
        "--map-hint-bearing-deg",
        type=float,
        default=30.0,
        help="Only send obstacle map hints for objects within this forward bearing.",
    )
    parser.add_argument(
        "--robot-separation-radius",
        type=float,
        default=0.45,
        help="Treat other visible robots within this many meters as moving obstacles.",
    )
    parser.add_argument(
        "--area-row-spacing",
        type=float,
        default=0.12,
        help="Row spacing in meters for robot area sweep commands.",
    )
    parser.add_argument(
        "--area-sweep-step",
        type=float,
        default=0.05,
        help="Forward step size in meters for robot area sweep commands.",
    )
    parser.add_argument(
        "--area-command-cooldown",
        type=float,
        default=2.0,
        help="Minimum seconds between automatic area-sweep resume commands.",
    )
    return parser.parse_args()


def parse_robot_config(value: str) -> RobotConfig:
    parts = value.split(":", 2)
    if len(parts) < 2 or not parts[0] or not parts[1]:
        raise argparse.ArgumentTypeError(
            "robot must use MODULE:USERNAME or MODULE:USERNAME:PASSWORD"
        )
    password = parts[2] if len(parts) == 3 else ""
    return RobotConfig(module=parts[0], username=parts[1], password=password)


def parse_robot_pose(value: str) -> RobotConfig:
    parts = value.split(":", 3)
    if len(parts) != 4 or not parts[0]:
        raise argparse.ArgumentTypeError(
            "robot pose must use MODULE:X:Y:THETA_DEG"
        )
    try:
        return RobotConfig(
            module=parts[0],
            username="",
            map_x=float(parts[1]),
            map_y=float(parts[2]),
            map_theta=math.radians(float(parts[3])),
        )
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "robot pose X, Y, and THETA_DEG must be numbers"
        ) from exc


def main() -> int:
    args = parse_args()
    app = ExperimentServerApp(args)

    def stop(signum: int, frame: Any) -> None:
        del signum, frame
        app.close()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    app.start()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
