#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Accelerometer calibration for multiple IMU modules reporting to the Master over Serial.

Data path (matches Master IMU_SERIAL_BINARY=1):
- Binary frame A5 5A A5 5A + 58-byte imu_packet_raw_t + 2-byte CRC (imu_serial_codec)
- Or text line IMU,MAC,micros[,seq],ax..gz[,mx_uT,my_uT,mz_uT][,temp_c]
- CALIB_* / CLRCAL / CALIB_REPORT / CALGET_* lines from Master shown in log
- GETCALIB,<MAC> → Master queries slave; CALREP → Serial CALIB_REPORT (applied on PC)

Features:
- Auto-detect MAC from IMU frames (binary or text)
- MAC list to pick module to calibrate
- Only ingest/process data for selected MAC
- Live view for selected module
- 6-face accel calib (+X, -X, +Y, -Y, +Z, -Z); gyro when still per face → mean gyro_bias sent with CALIB
- Print bias/scale and corrected accel
- (With T_die) Log gyro + temperature while IMU is still and board warms → linear fit bias_gyro(T) per axis;
  optional correction in live plot (slave still receives fixed offset when sending CALIB)

Run:
    python imu_calib.py [COM_PORT] [BAUD]
Examples:
    python imu_calib.py COM18
    python imu_calib.py COM18 921600

If no COM argument: scan for a port carrying IMU Master stream, then Connect.
"""

import math
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial

from imu_serial_codec import (
    feed_imu_serial,
    find_serial_port_with_imu,
    list_serial_port_devices,
    open_serial_for_esp32,
    serial_port_descriptions,
)

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 921600
G = 9.81
SAMPLES_PER_POSE = 400
UI_REFRESH_MS = 10
SERIAL_LIVE_READ_MAX_BYTES = 4096
SERIAL_READ_IDLE_BYTES = 256
PORT_PROBE_MIN_IMU_FRAMES = 2
PORT_PROBE_TIMEOUT_S = 0.85
PLOT_BUFFER_SIZE = 300
LIVE_PLOT_MA_WINDOW = 100
# Panel 3 (live plot): ω calib — MA100 on long buffer (watch thermal / bias drift)
GYRO_MA_LONG_BUFFER_LEN = 3000  # ~30 s @ 100 Hz
GYRO_MA_DISPLAY_N = 100
PLOT_Y_MARGIN_RATIO = 0.1
PLOT_MIN_SPAN = 1.0
MIN_CAPTURE_DURATION_US = 1_500_000
CAPTURE_TIMEOUT_S = 30.0
# Capture thresholds (relaxed for easier pass)
CAPTURE_MAX_AXIS_STD = 0.50
CAPTURE_MAX_AXIS_RANGE = 2.40
CAPTURE_MIN_DOMINANT_RATIO = 0.55
# Non-up axes should be near 0 (m/s²) and small vs dominant axis (~g);
# samples only when pose_alignment_check passes. max|two secondary| ≤ this (0.3 m/s² fails).
POSE_MAX_SECONDARY_ABS_MS2 = 0.2
POSE_MAX_SECONDARY_TO_DOMINANT = 0.35
POSE_MIN_DOMINANT_ABS_MS2 = 5.0
# Gyro vs die temp (still IMU, WiFi warms board — fit bias ≈ b0 + b1·T per axis)
MIN_TEMP_GYRO_SAMPLES = 25
GYRO_TEMP_RECORD_THROTTLE_US = 50_000  # at most ~20 samples/s
GYRO_TEMP_STATIONARY_ACC_MIN_MS2 = 8.5
GYRO_TEMP_STATIONARY_ACC_MAX_MS2 = 11.5
MIN_TEMP_SPAN_C_WARN = 0.35  # warn if ΔT too small (hard to separate noise)

POSES = [
    "+X (X axis up)",
    "-X (X axis down)",
    "+Y (Y axis up)",
    "-Y (Y axis down)",
    "+Z (Z axis up)",
    "-Z (Z axis down)",
]

# Spirit-level widget: ball offset from two secondary accel axes (m/s² → pixels).
BUBBLE_LEVEL_CANVAS_PX = 200
BUBBLE_BALL_RADIUS_PX = 10
BUBBLE_OUTER_RING_RADIUS_PX = 74
BUBBLE_CENTER_MARK_PX = 14
# Ring radii map to tilt (m/s²): inner=OK, mid=warn, outer=large (ball can travel to outer).
BUBBLE_OK_TILT_MS2 = POSE_MAX_SECONDARY_ABS_MS2
BUBBLE_WARN_TILT_MS2 = max(POSE_MAX_SECONDARY_ABS_MS2 * 1.5, 0.30)
BUBBLE_OUTER_TILT_MS2 = 0.75
# EMA on secondary-axis tilt (0..1): lower = smoother ball, slower response.
BUBBLE_SMOOTH_ALPHA = 0.07


def parse_pose_axis_sign(pose_str):
    """'+X (X axis up)' → ('X', +1) or (None, None)."""
    if not pose_str or len(pose_str) < 2:
        return None, None
    if pose_str[0] == "+":
        sign = 1
    elif pose_str[0] == "-":
        sign = -1
    else:
        return None, None
    axis = pose_str[1].upper()
    if axis not in ("X", "Y", "Z"):
        return None, None
    return axis, sign


def infer_up_axis_sign_from_accel(ax, ay, az):
    """Dominant gravity axis for auto spirit-level (same idea as rough pose text)."""
    axes = {"X": float(ax), "Y": float(ay), "Z": float(az)}
    acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if acc_norm < 0.5:
        return None, None
    dominant_axis = max(axes, key=lambda k: abs(axes[k]))
    dominant_value = axes[dominant_axis]
    dominant_abs = abs(dominant_value)
    other_values = [abs(value) for key, value in axes.items() if key != dominant_axis]
    second_abs = max(other_values) if other_values else 0.0
    if dominant_abs < 0.75 * acc_norm:
        return None, None
    if second_abs > 0.45 * dominant_abs:
        return None, None
    sign = 1 if dominant_value >= 0 else -1
    return dominant_axis, sign


def secondary_tilt_for_level(ax, ay, az, up_axis):
    """Tilt in the plane ⊥ gravity when `up_axis` points up/down (m/s²)."""
    vec = {"X": float(ax), "Y": float(ay), "Z": float(az)}
    h_name, v_name = {"X": ("Y", "Z"), "Y": ("X", "Z"), "Z": ("X", "Y")}[up_axis]
    return vec[h_name], vec[v_name], h_name, v_name


def trailing_moving_average(seq, window):
    """Trailing mean over `window` samples (through current index); same length as seq."""
    x = np.asarray(seq, dtype=float)
    n = x.size
    if n == 0:
        return x
    w = max(1, int(window))
    cs = np.concatenate([[0.0], np.cumsum(x)])
    out = np.empty(n)
    for i in range(n):
        j = max(0, i - w + 1)
        out[i] = (cs[i + 1] - cs[j]) / (i - j + 1)
    return out


def norm_mac_compact(mac_str):
    """AA:BB:... or AABBCC... → uppercase hex digits only for MAC match."""
    return "".join(c for c in str(mac_str).upper() if c in "0123456789ABCDEF")


def pose_alignment_check(vec, pose_str):
    """
    Check raw accel vector (m/s²) matches 6-face pose:
    correct axis + sign, that axis dominant, two others near 0.

    Returns (ok: bool, failures: list[str]).
    """
    sign_char = pose_str[0]
    axis_char = pose_str[1].upper()
    ax, ay, az = float(vec[0]), float(vec[1]), float(vec[2])
    vec_map = {"X": ax, "Y": ay, "Z": az}
    expected_val = vec_map[axis_char]
    failures = []

    dominant_axis = max(vec_map, key=lambda k: abs(vec_map[k]))
    if dominant_axis != axis_char:
        failures.append(
            f"largest axis is {dominant_axis} (|{dominant_axis}|={abs(vec_map[dominant_axis]):.3f}), "
            f"need {axis_char}"
        )

    if abs(expected_val) < POSE_MIN_DOMINANT_ABS_MS2:
        failures.append(
            f"axis {axis_char} too small: |{axis_char}|={abs(expected_val):.3f} < {POSE_MIN_DOMINANT_ABS_MS2} m/s²"
        )

    if sign_char == "+" and expected_val <= 0:
        failures.append(f"{axis_char} must be positive, got {expected_val:.3f}")
    if sign_char == "-" and expected_val >= 0:
        failures.append(f"{axis_char} must be negative, got {expected_val:.3f}")

    others = [abs(v) for k, v in vec_map.items() if k != axis_char]
    max_sec = max(others) if others else 0.0
    if max_sec > POSE_MAX_SECONDARY_ABS_MS2:
        failures.append(
            f"secondary axes not near 0: max|secondary|={max_sec:.3f} > {POSE_MAX_SECONDARY_ABS_MS2} m/s²"
        )

    dom_abs = abs(expected_val)
    if dom_abs > 0 and max_sec > POSE_MAX_SECONDARY_TO_DOMINANT * dom_abs:
        failures.append(
            f"tilted too much: max|secondary|={max_sec:.3f} > {POSE_MAX_SECONDARY_TO_DOMINANT:.2f}×|{axis_char}|"
        )

    acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
    if acc_norm > 0 and abs(expected_val) < CAPTURE_MIN_DOMINANT_RATIO * acc_norm:
        failures.append(
            f"{axis_char} component not dominant enough: |{axis_char}|={abs(expected_val):.3f}, "
            f"need ≥ {CAPTURE_MIN_DOMINANT_RATIO:.2f}×|acc|={CAPTURE_MIN_DOMINANT_RATIO * acc_norm:.3f}"
        )

    return (len(failures) == 0, failures)


def normalize_imu_row_for_calib(row):
    """
    Dict from imu_serial_codec (kind=imu, accel m/s², gyro °/s) → dict for UI.
    Mag (µT) and temp_c (°C die) from 58-byte binary or text line if present.
    """
    mac = str(row["mac"]).strip().upper()
    micros = int(row["micros"])
    out = {
        "mac": mac,
        "micros": micros,
        "timestamp": micros // 1000,
        "ax": float(row["ax"]),
        "ay": float(row["ay"]),
        "az": float(row["az"]),
        "gx": float(row["gx"]),
        "gy": float(row["gy"]),
        "gz": float(row["gz"]),
        "mx": float(row.get("mx_uT", row.get("mx", 0.0))),
        "my": float(row.get("my_uT", row.get("my", 0.0))),
        "mz": float(row.get("mz_uT", row.get("mz", 0.0))),
    }
    if row.get("temp_c") is not None:
        out["temp_c"] = float(row["temp_c"])
    return out


class IMUCalibApp:
    def __init__(self, root, default_port, default_baud, *, auto_scan_connect: bool = True):
        self.root = root
        self.root.title("IMU Calibration Tool (ESP-NOW Master)")

        self.ser = None
        self.reader_thread = None
        self.reader_running = False
        self.rx_queue = queue.Queue()

        self.known_macs = []
        self.selected_mac = None
        self.latest_by_mac = {}

        self.calibration_running = False
        self.capture_mode = None
        self.capture_target = None
        self.capture_samples = []
        self.capture_gyro_batch = []
        self.calib_gyro_accum = []
        self.pose_results = []
        self.validation_results = []
        self.pose_index = 0
        self.capture_deadline = 0.0
        self.bubble_target_pose = None
        self.capture_start_micros = None
        self.capture_first_sample_micros = None
        self.capture_last_sample_micros = None
        self.bias = None
        self.scale = None
        self.global_scale = 1.0
        # Gyro bias (°/s) sent with CALIB: mean over 6 poses; GETCALIB can sync from board
        self.gyro_bias = np.zeros(3, dtype=float)
        # Gyro bias ~ b0 + b1*T (°C) per axis; coefficients shape (3,2) after fit
        self.gyro_temp_coef = None
        self.gyro_temp_samples = []
        self.gyro_temp_record_active = False
        self._gyro_temp_last_record_us = None
        self.gyro_temp_stationary_var = tk.BooleanVar(value=True)
        self.gyro_temp_apply_var = tk.BooleanVar(value=False)
        self.gyro_temp_status_var = tk.StringVar(
            value="T–gyro: no samples yet. Stream must include T_die (58-byte binary packet or text line)."
        )
        self.latest_corrected = None
        self.corrected_history = {
            "norm": deque(maxlen=PLOT_BUFFER_SIZE),
            "acc_raw_norm": deque(maxlen=PLOT_BUFFER_SIZE),
            "gxc": deque(maxlen=PLOT_BUFFER_SIZE),
            "gyc": deque(maxlen=PLOT_BUFFER_SIZE),
            "gzc": deque(maxlen=PLOT_BUFFER_SIZE),
            "gxr": deque(maxlen=PLOT_BUFFER_SIZE),
            "gyr": deque(maxlen=PLOT_BUFFER_SIZE),
            "gzr": deque(maxlen=PLOT_BUFFER_SIZE),
        }
        self.plot_fig = None
        self.plot_ani = None
        self.plot_live_artists = []
        # ω calib (°/s) — long buffer for MA100 panel (separate from 300-sample plot deque)
        self.gyro_ma_long_buffer = deque(maxlen=GYRO_MA_LONG_BUFFER_LEN)
        self._rx_buf = bytearray()
        self._port_probe_thread = None

        self.port_var = tk.StringVar(value=default_port)
        self.baud_var = tk.StringVar(value=str(default_baud))
        self.status_var = tk.StringVar(value="Not connected")
        self.selected_mac_var = tk.StringVar(value="No MAC selected")
        self.live_var = tk.StringVar(value="No data yet")
        self.orientation_var = tk.StringVar(value="Rough pose: unknown")
        self.corrected_var = tk.StringVar(value="No calib data yet")
        self.calib_quality_var = tk.StringVar(value="Calib quality: none yet")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(UI_REFRESH_MS, self.process_rx_queue)
        self.root.after(150, self.refresh_port_list)
        if auto_scan_connect:
            self.root.after(250, self.auto_detect_port)
        else:
            self.root.after(300, self.connect)

    def refresh_port_list(self) -> None:
        ports = list_serial_port_devices()
        desc = serial_port_descriptions()
        labels = []
        for dev in ports:
            d = desc.get(dev, "")
            labels.append(f"{dev} — {d}" if d else dev)
        self._combo_port["values"] = labels
        self._port_device_by_label = {
            labels[i]: ports[i] for i in range(len(ports))
        }
        current = self.port_var.get().strip()
        if ports:
            if current in ports:
                idx = ports.index(current)
                self._combo_port.set(labels[idx])
                self.port_var.set(current)
            else:
                self._combo_port.set(labels[0])
                self.port_var.set(ports[0])
        if not ports:
            self.log("No COM ports detected on this PC.")
        else:
            self.log(
                "COM ports: "
                + ", ".join(f"{p} ({desc.get(p) or '?'})" for p in ports)
            )

    def _combo_label_to_device(self, label: str) -> str:
        label = (label or "").strip()
        if not label:
            return ""
        if label in getattr(self, "_port_device_by_label", {}):
            return self._port_device_by_label[label]
        return label.split(" — ", 1)[0].strip()

    def _on_port_combo_selected(self, _event=None) -> None:
        dev = self._combo_label_to_device(self._combo_port.get())
        if dev:
            self.port_var.set(dev)

    def auto_detect_port(self, *, auto_connect: bool = True) -> None:
        if self.ser is not None and self.ser.is_open:
            return
        if self._port_probe_thread is not None and self._port_probe_thread.is_alive():
            return
        self.status_var.set("Scanning COM ports for IMU Master stream…")
        self._port_probe_thread = threading.Thread(
            target=self._auto_detect_port_worker,
            kwargs={"auto_connect": auto_connect},
            daemon=True,
        )
        self._port_probe_thread.start()

    def _auto_detect_port_worker(self, *, auto_connect: bool = True) -> None:
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            baud = DEFAULT_BAUD
        ports = list_serial_port_devices()
        found, counts = find_serial_port_with_imu(
            baud,
            ports=ports,
            probe_timeout_s=PORT_PROBE_TIMEOUT_S,
            min_imu_frames=PORT_PROBE_MIN_IMU_FRAMES,
        )

        def apply() -> None:
            self.refresh_port_list()
            if found:
                desc = serial_port_descriptions().get(found, "")
                labels = list(self._combo_port["values"])
                pick = next(
                    (lb for lb in labels if self._combo_label_to_device(lb) == found),
                    found,
                )
                self._combo_port.set(pick)
                self.port_var.set(found)
                n = counts.get(found, 0)
                extra = (
                    f" ({desc})" if desc else ""
                )
                self.status_var.set(f"Auto-found: {found}{extra} — {n} IMU frames")
                self.log(
                    f"Auto-scan: selected {found} ({n} IMU frames in ~{PORT_PROBE_TIMEOUT_S:.1f}s)."
                )
                parts = [
                    f"{p}={c}" for p, c in sorted(counts.items()) if c > 0
                ]
                if parts:
                    self.log("Scan results: " + ", ".join(parts))
                if auto_connect:
                    self.log("Auto-connecting to found port…")
                    self.connect()
            else:
                self.status_var.set("No IMU stream found — pick a port manually and Connect")
                self.log(
                    "Auto-scan: no IMU stream (Master on + slave linked?). "
                    "Try «Scan COM» again or choose a port manually."
                )

        self.root.after(0, apply)

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill="both", expand=True)

        top = ttk.Frame(frame)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port").grid(row=0, column=0, sticky="w")
        self._combo_port = ttk.Combobox(top, width=36, state="readonly")
        self._combo_port.grid(row=0, column=1, padx=(6, 6), sticky="ew")
        self._combo_port.bind("<<ComboboxSelected>>", self._on_port_combo_selected)
        self._port_device_by_label: dict[str, str] = {}
        ttk.Button(top, text="Refresh", command=self.refresh_port_list).grid(
            row=0, column=2, padx=(0, 4)
        )
        ttk.Button(top, text="Scan COM", command=self.auto_detect_port).grid(
            row=0, column=3, padx=(0, 10)
        )
        ttk.Label(top, text="Baud").grid(row=0, column=4, sticky="w")
        ttk.Entry(top, textvariable=self.baud_var, width=12).grid(row=0, column=5, padx=(6, 10))
        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=6, padx=(0, 6))
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=7)
        top.columnconfigure(1, weight=1)

        ttk.Label(frame, textvariable=self.status_var).pack(anchor="w", pady=(8, 6))

        main = ttk.Frame(frame)
        main.pack(fill="both", expand=True)

        left = ttk.LabelFrame(main, text="MAC list", padding=8)
        left.pack(side="left", fill="y")

        self.mac_listbox = tk.Listbox(left, width=24, height=12, exportselection=False)
        self.mac_listbox.pack(fill="y", expand=True)
        self.mac_listbox.bind("<<ListboxSelect>>", self.on_select_mac)

        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))

        info = ttk.LabelFrame(right, text="Selected module", padding=8)
        info.pack(fill="x")
        ttk.Label(info, textvariable=self.selected_mac_var).pack(anchor="w")

        live = ttk.LabelFrame(right, text="Live IMU data", padding=8)
        live.pack(fill="x", pady=(10, 0))
        live_row = ttk.Frame(live)
        live_row.pack(fill="x")
        live_text = ttk.Frame(live_row)
        live_text.pack(side="left", fill="both", expand=True)
        ttk.Label(live_text, textvariable=self.live_var, justify="left").pack(anchor="w")
        self.orientation_label = tk.Label(
            live_text,
            textvariable=self.orientation_var,
            justify="left",
            fg="red",
            font=("TkDefaultFont", 12, "bold"),
        )
        self.orientation_label.pack(anchor="w", pady=(6, 0))
        self._build_bubble_level(live_row)

        corrected = ttk.LabelFrame(right, text="Accel after calib", padding=8)
        corrected.pack(fill="x", pady=(10, 0))
        ttk.Label(corrected, textvariable=self.corrected_var, justify="left").pack(anchor="w")
        self.calib_quality_label = tk.Label(
            corrected,
            textvariable=self.calib_quality_var,
            justify="left",
            fg="black",
            font=("TkDefaultFont", 10, "bold"),
        )
        self.calib_quality_label.pack(anchor="w", pady=(6, 0))

        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=(10, 0))
        ttk.Button(buttons, text="Start 6-Point Calibration", command=self.start_calibration).pack(side="left")
        ttk.Button(buttons, text="Clear Results", command=self.clear_calibration).pack(side="left", padx=(8, 0))
        ttk.Button(buttons, text="Show Calibrated Live Plot", command=self.show_calibrated_plot).pack(
            side="left", padx=(8, 0)
        )
        ttk.Button(buttons, text="Validate 6 Faces", command=self.start_validation).pack(
            side="left", padx=(8, 0)
        )
        ttk.Button(buttons, text="Send Calib To Module", command=self.send_calibration_to_module).pack(
            side="left", padx=(8, 0)
        )
        ttk.Button(buttons, text="Clear Calib On Module", command=self.clear_calibration_on_module).pack(
            side="left", padx=(8, 0)
        )

        buttons_get = ttk.Frame(right)
        buttons_get.pack(fill="x", pady=(8, 0))
        ttk.Button(
            buttons_get,
            text="Get calib from module (GETCALIB)",
            command=self.send_get_calibration_from_module,
        ).pack(side="left")

        temp_gyro = ttk.LabelFrame(
            right,
            text="Gyro bias vs temperature (still IMU, WiFi warms board)",
            padding=8,
        )
        temp_gyro.pack(fill="x", pady=(8, 0))
        row_tg = ttk.Frame(temp_gyro)
        row_tg.pack(fill="x")
        self.btn_temp_rec = ttk.Button(
            row_tg,
            text="Start T–gyro capture",
            command=self.toggle_gyro_temp_record,
        )
        self.btn_temp_rec.pack(side="left")
        self.btn_temp_fit = ttk.Button(
            row_tg,
            text="Fit linear regression",
            command=self.fit_gyro_temp_model,
            state=tk.DISABLED,
        )
        self.btn_temp_fit.pack(side="left", padx=(8, 0))
        ttk.Button(
            row_tg,
            text="Clear samples & model",
            command=self.clear_gyro_temp_data,
        ).pack(side="left", padx=(8, 0))
        ttk.Checkbutton(
            temp_gyro,
            text="Only log when |acc| ≈ g (~8.5–11.5 m/s², still pose)",
            variable=self.gyro_temp_stationary_var,
        ).pack(anchor="w", pady=(4, 0))
        ttk.Checkbutton(
            temp_gyro,
            text="Apply gyro correction b0 + b1·T in live plot / MA ω",
            variable=self.gyro_temp_apply_var,
            command=self.on_gyro_temp_apply_toggle,
        ).pack(anchor="w")
        ttk.Label(
            temp_gyro,
            textvariable=self.gyro_temp_status_var,
            justify="left",
            wraplength=640,
        ).pack(anchor="w", pady=(4, 0))

        log_frame = ttk.LabelFrame(right, text="Log / results", padding=8)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))
        self.text = tk.Text(log_frame, height=18, width=72)
        self.text.pack(fill="both", expand=True)

    def _build_bubble_level(self, parent):
        """Spirit level: ball moves toward the tilted side (secondary accel axes)."""
        wrap = ttk.Frame(parent)
        wrap.pack(side="right", padx=(8, 0))
        self.bubble_pose_var = tk.StringVar(value="Level: —")
        ttk.Label(
            wrap,
            textvariable=self.bubble_pose_var,
            font=("TkDefaultFont", 9),
            justify="center",
        ).pack()
        ttk.Label(
            wrap,
            text="Rings: green=OK · orange=warn · dashed=large tilt",
            font=("TkDefaultFont", 8),
        ).pack()
        size = BUBBLE_LEVEL_CANVAS_PX
        self.bubble_canvas = tk.Canvas(
            wrap,
            width=size,
            height=size,
            highlightthickness=1,
            highlightbackground="#888",
            bg="#f5f5f0",
        )
        self.bubble_canvas.pack()
        cx = cy = size // 2
        outer_r = BUBBLE_OUTER_RING_RADIUS_PX
        self._bubble_cx = cx
        self._bubble_cy = cy
        self._bubble_outer_r = outer_r
        self._bubble_px_per_ms2 = outer_r / BUBBLE_OUTER_TILT_MS2

        def ring_px(tilt_ms2):
            return tilt_ms2 * self._bubble_px_per_ms2

        r_outer = ring_px(BUBBLE_OUTER_TILT_MS2)
        r_warn = ring_px(BUBBLE_WARN_TILT_MS2)
        r_ok = ring_px(BUBBLE_OK_TILT_MS2)
        # Large-tilt guide (outer) — ball may sit here when badly misaligned.
        self.bubble_canvas.create_oval(
            cx - r_outer,
            cy - r_outer,
            cx + r_outer,
            cy + r_outer,
            outline="#c62828",
            width=2,
            dash=(5, 4),
            tags="static",
        )
        self.bubble_canvas.create_oval(
            cx - r_warn,
            cy - r_warn,
            cx + r_warn,
            cy + r_warn,
            outline="#ef6c00",
            width=2,
            tags="static",
        )
        self.bubble_canvas.create_oval(
            cx - r_ok,
            cy - r_ok,
            cx + r_ok,
            cy + r_ok,
            outline="#558b2f",
            width=2,
            tags="static",
        )
        self.bubble_canvas.create_line(
            cx - r_outer, cy, cx + r_outer, cy, fill="#ccc", tags="static"
        )
        self.bubble_canvas.create_line(
            cx, cy - r_outer, cx, cy + r_outer, fill="#ccc", tags="static"
        )
        tr = BUBBLE_CENTER_MARK_PX
        self.bubble_canvas.create_oval(
            cx - tr,
            cy - tr,
            cx + tr,
            cy + tr,
            outline="#33691e",
            width=2,
            tags="static",
        )
        self._bubble_label_items = []
        br = BUBBLE_BALL_RADIUS_PX
        self.bubble_ball = self.bubble_canvas.create_oval(
            cx - br, cy - br, cx + br, cy + br, fill="#9e9e9e", outline="#424242", width=1
        )
        self.bubble_tilt_text = self.bubble_canvas.create_text(
            cx,
            size - 18,
            text="",
            font=("TkDefaultFont", 8),
            justify="center",
        )
        self._bubble_smooth_axis = None
        self._bubble_smooth = None

    def _reset_bubble_smooth(self):
        self._bubble_smooth_axis = None
        self._bubble_smooth = None

    def _smooth_bubble_tilt(self, up_axis, tilt_h, tilt_v):
        """Exponential moving average on tilt in the level plane (per up-axis)."""
        if self._bubble_smooth_axis != up_axis or self._bubble_smooth is None:
            self._bubble_smooth_axis = up_axis
            self._bubble_smooth = (float(tilt_h), float(tilt_v))
            return self._bubble_smooth

        sh, sv = self._bubble_smooth
        a = BUBBLE_SMOOTH_ALPHA
        sh = a * float(tilt_h) + (1.0 - a) * sh
        sv = a * float(tilt_v) + (1.0 - a) * sv
        self._bubble_smooth = (sh, sv)
        return sh, sv

    def _bubble_correction_hint(self, tilt_h, tilt_v, h_name, v_name, smooth_max):
        """Which axis to tilt toward so the ball moves to center."""
        if smooth_max <= POSE_MAX_SECONDARY_ABS_MS2 * 0.45:
            return ""
        parts = []
        thresh = 0.025
        if abs(tilt_h) > thresh:
            parts.append(f"{'−' if tilt_h > 0 else '+'}{h_name}")
        if abs(tilt_v) > thresh:
            parts.append(f"{'−' if tilt_v > 0 else '+'}{v_name}")
        if not parts:
            return ""
        return "Tilt toward: " + ", ".join(parts)

    def _update_bubble_axis_labels(self, h_name, v_name):
        for item in self._bubble_label_items:
            self.bubble_canvas.delete(item)
        self._bubble_label_items = []
        cx, cy, r = self._bubble_cx, self._bubble_cy, self._bubble_outer_r
        font = ("TkDefaultFont", 8, "bold")
        pad = 12
        self._bubble_label_items.append(
            self.bubble_canvas.create_text(
                cx + r - pad, cy, text=f"+{h_name}", font=font, fill="#333"
            )
        )
        self._bubble_label_items.append(
            self.bubble_canvas.create_text(
                cx - r + pad, cy, text=f"−{h_name}", font=font, fill="#333"
            )
        )
        self._bubble_label_items.append(
            self.bubble_canvas.create_text(
                cx, cy - r + pad, text=f"−{v_name}", font=font, fill="#333"
            )
        )
        self._bubble_label_items.append(
            self.bubble_canvas.create_text(
                cx, cy + r - pad, text=f"+{v_name}", font=font, fill="#333"
            )
        )

    def _place_bubble_ball(self, tilt_h, tilt_v, fill):
        """Place ball from secondary-axis tilts (m/s²); clamp inside outer ring."""
        ox = tilt_h * self._bubble_px_per_ms2
        oy = tilt_v * self._bubble_px_per_ms2
        max_dist = self._bubble_outer_r - BUBBLE_BALL_RADIUS_PX - 2
        dist = math.hypot(ox, oy)
        if dist > max_dist and dist > 0:
            ox *= max_dist / dist
            oy *= max_dist / dist
        bx = self._bubble_cx + ox
        by = self._bubble_cy + oy
        br = BUBBLE_BALL_RADIUS_PX
        self.bubble_canvas.coords(self.bubble_ball, bx - br, by - br, bx + br, by + br)
        self.bubble_canvas.itemconfig(self.bubble_ball, fill=fill)

    def update_bubble_level(self, ax, ay, az):
        if not hasattr(self, "bubble_canvas"):
            return

        pose_hint = self.capture_target or self.bubble_target_pose
        if pose_hint:
            up_axis, _up_sign = parse_pose_axis_sign(pose_hint)
            short = pose_hint.split("(", 1)[0].strip()
            self.bubble_pose_var.set(f"Target face: {short}")
        else:
            up_axis, up_sign = infer_up_axis_sign_from_accel(ax, ay, az)
            if up_axis is not None:
                sign_ch = "+" if up_sign > 0 else "−"
                self.bubble_pose_var.set(f"Auto face: {sign_ch}{up_axis}")
            else:
                self.bubble_pose_var.set("Level: need |acc| ≈ g on one axis")

        if up_axis is None:
            self._reset_bubble_smooth()
            self._place_bubble_ball(0.0, 0.0, "#bdbdbd")
            self.bubble_canvas.itemconfig(self.bubble_tilt_text, text="")
            for item in self._bubble_label_items:
                self.bubble_canvas.delete(item)
            self._bubble_label_items = []
            return

        tilt_h, tilt_v, h_name, v_name = secondary_tilt_for_level(ax, ay, az, up_axis)
        max_tilt = max(abs(tilt_h), abs(tilt_v))
        smooth_h, smooth_v = self._smooth_bubble_tilt(up_axis, tilt_h, tilt_v)
        smooth_max = max(abs(smooth_h), abs(smooth_v))
        if smooth_max <= POSE_MAX_SECONDARY_ABS_MS2:
            color = "#43a047"
        elif smooth_max <= POSE_MAX_SECONDARY_ABS_MS2 * 1.5:
            color = "#fb8c00"
        else:
            color = "#e53935"
        self._place_bubble_ball(smooth_h, smooth_v, color)
        self._update_bubble_axis_labels(h_name, v_name)
        ok_mark = "OK" if max_tilt <= POSE_MAX_SECONDARY_ABS_MS2 else "tilt"
        status = f"{ok_mark} max|2⊥|={max_tilt:.3f} (≤{POSE_MAX_SECONDARY_ABS_MS2})"
        hint = self._bubble_correction_hint(smooth_h, smooth_v, h_name, v_name, smooth_max)
        if hint:
            status += f"\n{hint}"
        self.bubble_canvas.itemconfig(self.bubble_tilt_text, text=status, fill=color)

    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        self.text.insert(tk.END, f"[{stamp}] {msg}\n")
        self.text.see(tk.END)

    def connect(self):
        if self.ser is not None and self.ser.is_open:
            self.log("Serial is already connected.")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Invalid baud rate.")
            return

        port = self._combo_label_to_device(self._combo_port.get()) or self.port_var.get().strip()
        if not port:
            messagebox.showwarning("COM", "Select a COM port or press «Scan COM».")
            return

        try:
            self.ser = open_serial_for_esp32(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            messagebox.showerror("Serial open error", str(exc))
            return

        self._rx_buf = bytearray()
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"Connected: {port} @ {baud}")
        self.log(
            f"Connected (DTR=False, RTS=False). Waiting for IMU data and MAC discovery..."
        )

    def disconnect(self):
        self.reader_running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._rx_buf = bytearray()
        self.status_var.set("Disconnected")
        self.log("Serial disconnected.")

    def serial_reader(self):
        while self.reader_running and self.ser is not None and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(min(waiting, SERIAL_LIVE_READ_MAX_BYTES))
                else:
                    chunk = self.ser.read(SERIAL_READ_IDLE_BYTES)
                if not chunk:
                    time.sleep(0.001)
                    continue
                rows, self._rx_buf = feed_imu_serial(
                    self._rx_buf, chunk, include_unparsed_lines=True
                )
                for row in rows:
                    k = row.get("kind")
                    if k == "imu":
                        self.rx_queue.put(normalize_imu_row_for_calib(row))
                    elif k == "vl53":
                        continue
                    elif k == "log":
                        t = row.get("text", "")
                        if t.startswith(
                            (
                                "CALIB_",
                                "CALIB,",
                                "CLRCAL",
                                "CALIB_REPORT,",
                                "CALGET_",
                                "CALGET_PARSE",
                            )
                        ):
                            self.rx_queue.put({"_log": t})
            except Exception as exc:
                self.rx_queue.put({"_error": str(exc)})
                break

    def _ingest_live_sample(self, item):
        """Update plot buffers (light) — called for every sample, does not touch Tk labels."""
        if self.bias is None or self.scale is None:
            return
        raw = np.array([item["ax"], item["ay"], item["az"]], dtype=float)
        corrected = (raw - self.bias) * self.scale * self.global_scale
        self.latest_corrected = corrected
        corrected_norm = float(np.linalg.norm(corrected))
        self.append_corrected_sample(item, corrected, corrected_norm)

    def process_rx_queue(self):
        latest_ui = None
        while True:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break

            if "_error" in item:
                self.status_var.set("Reader thread stopped")
                self.log(f"Serial read error: {item['_error']}")
                self.reader_running = False
                break

            if "_log" in item:
                t = item["_log"]
                self.log(t)
                if t.startswith("CALIB_REPORT,"):
                    self.apply_calib_report_from_serial_line(t)
                continue

            mac = item["mac"]
            self.latest_by_mac[mac] = item
            self.ensure_mac(mac)

            if self.selected_mac == mac:
                latest_ui = item
                self._ingest_live_sample(item)
                if self.gyro_temp_record_active:
                    self._record_gyro_temp_sample(item)
                if self.capture_target is not None and self.should_accept_capture_sample(item):
                    self.capture_samples.append(np.array([item["ax"], item["ay"], item["az"]], dtype=float))
                    if self.capture_mode == "calibration":
                        self.capture_gyro_batch.append(
                            np.array([item["gx"], item["gy"], item["gz"]], dtype=float)
                        )
                    if self.capture_first_sample_micros is None:
                        self.capture_first_sample_micros = item["micros"]
                    self.capture_last_sample_micros = item["micros"]

        if latest_ui is not None:
            self._refresh_live_labels(latest_ui)

        self.root.after(UI_REFRESH_MS, self.process_rx_queue)

    def _stationary_for_gyro_temp_record(self, row):
        acc_norm = math.sqrt(row["ax"] ** 2 + row["ay"] ** 2 + row["az"] ** 2)
        return (
            GYRO_TEMP_STATIONARY_ACC_MIN_MS2
            <= acc_norm
            <= GYRO_TEMP_STATIONARY_ACC_MAX_MS2
        )

    def _record_gyro_temp_sample(self, row):
        if row.get("temp_c") is None:
            return
        if self.gyro_temp_stationary_var.get() and not self._stationary_for_gyro_temp_record(
            row
        ):
            return
        mu = int(row["micros"])
        if self._gyro_temp_last_record_us is not None:
            if mu - self._gyro_temp_last_record_us < GYRO_TEMP_RECORD_THROTTLE_US:
                return
        self._gyro_temp_last_record_us = mu
        self.gyro_temp_samples.append(
            (
                float(row["temp_c"]),
                float(row["gx"]),
                float(row["gy"]),
                float(row["gz"]),
            )
        )
        self._refresh_gyro_temp_fit_button()
        n = len(self.gyro_temp_samples)
        t0 = self.gyro_temp_samples[0][0]
        t1 = self.gyro_temp_samples[-1][0]
        self.gyro_temp_status_var.set(
            f"Recording: {n} samples  |  T_die ≈ {t0:.2f} → {t1:.2f} °C  (stop to fit)"
        )

    def _refresh_gyro_temp_fit_button(self):
        if len(self.gyro_temp_samples) >= MIN_TEMP_GYRO_SAMPLES:
            self.btn_temp_fit.configure(state=tk.NORMAL)
        else:
            self.btn_temp_fit.configure(state=tk.DISABLED)

    def effective_gyro_bias(self, row):
        """
        °/s: fixed bias (gyro_bias) or b0+b1*T if fitted, apply checkbox on, and temp_c present.
        """
        if (
            self.gyro_temp_apply_var.get()
            and self.gyro_temp_coef is not None
            and row.get("temp_c") is not None
        ):
            T = float(row["temp_c"])
            c = self.gyro_temp_coef
            return c[:, 0] + c[:, 1] * T
        return self.gyro_bias

    def toggle_gyro_temp_record(self):
        if not self.selected_mac:
            messagebox.showwarning("No MAC selected", "Select a module in the list first.")
            return
        if not self.gyro_temp_record_active:
            latest = self.latest_by_mac.get(self.selected_mac)
            if latest is None or latest.get("temp_c") is None:
                if not messagebox.askyesno(
                    "No temperature yet",
                    "No samples with T_die yet (need 58-byte binary packets from Master).\n"
                    "Start capture anyway? (logging begins when T_die appears).",
                ):
                    return
            self.gyro_temp_samples = []
            self._gyro_temp_last_record_us = None
            self.gyro_temp_record_active = True
            self.btn_temp_rec.configure(text="Stop T–gyro capture")
            self.gyro_temp_status_var.set("Recording… keep IMU still; let WiFi warm the board.")
            self.log(
                "T–gyro capture: keep IMU still (|acc|≈g if filter enabled), watch T rise over time."
            )
        else:
            self.gyro_temp_record_active = False
            self.btn_temp_rec.configure(text="Start T–gyro capture")
            n = len(self.gyro_temp_samples)
            self._refresh_gyro_temp_fit_button()
            if n == 0:
                self.gyro_temp_status_var.set(
                    "Stopped — no samples yet (need T_die in stream)."
                )
            else:
                t0 = self.gyro_temp_samples[0][0]
                t1 = self.gyro_temp_samples[-1][0]
                self.gyro_temp_status_var.set(
                    f"Stopped: {n} samples, ΔT={t1 - t0:.2f} °C. "
                    f"Press «Fit linear regression» if you have ≥{MIN_TEMP_GYRO_SAMPLES} samples."
                )
            self.log(f"Stopped T–gyro capture: {n} samples saved.")

    def fit_gyro_temp_model(self):
        n = len(self.gyro_temp_samples)
        if n < MIN_TEMP_GYRO_SAMPLES:
            messagebox.showwarning(
                "Not enough samples",
                f"Need at least {MIN_TEMP_GYRO_SAMPLES} samples (have {n}).",
            )
            return
        data = np.array(self.gyro_temp_samples, dtype=np.float64)
        T = data[:, 0]
        t_span = float(np.max(T) - np.min(T))
        coef = np.zeros((3, 2), dtype=np.float64)
        r2s = []
        for i in range(3):
            g = data[:, 1 + i]
            A = np.column_stack([np.ones(len(T)), T])
            sol, *_ = np.linalg.lstsq(A, g, rcond=None)
            coef[i, :] = sol
            pred = A @ sol
            ss_tot = np.sum((g - np.mean(g)) ** 2)
            if ss_tot > 1e-18:
                r2 = 1.0 - np.sum((g - pred) ** 2) / ss_tot
            else:
                r2 = 1.0
            r2s.append(float(r2))
        self.gyro_temp_coef = coef
        warn = ""
        if t_span < MIN_TEMP_SPAN_C_WARN:
            warn = (
                f" Warning: temperature span only ΔT≈{t_span:.2f} °C — "
                "capture more as the board warms for a more reliable model."
            )
        self.gyro_temp_status_var.set(
            f"Fitted: n={n}, ΔT={t_span:.2f} °C. "
            f"R² ≈ ({r2s[0]:.3f}, {r2s[1]:.3f}, {r2s[2]:.3f}) (gx,gy,gz).{warn}"
        )
        self.log(
            "Fit gyro(T): each axis ω_bias ≈ b0 + b1·T (°C). "
            f"gx: b0={coef[0,0]:.6f}, b1={coef[0,1]:.7f} | "
            f"gy: b0={coef[1,0]:.6f}, b1={coef[1,1]:.7f} | "
            f"gz: b0={coef[2,0]:.6f}, b1={coef[2,1]:.7f} °/s per °C"
        )
        self.log(
            f"R² (gx,gy,gz) = ({r2s[0]:.4f}, {r2s[1]:.4f}, {r2s[2]:.4f}). "
            "Slave stores a fixed offset only — to send CALIB, use bias at one T_ref (e.g. 25 °C) "
            "from the table above or gyro_bias from 6-face calibration."
        )
        latest = self.latest_by_mac.get(self.selected_mac) if self.selected_mac else None
        if latest is not None:
            self.reset_corrected_history()
            self.update_live_view(latest)

    def clear_gyro_temp_data(self):
        self.gyro_temp_samples = []
        self.gyro_temp_coef = None
        self._gyro_temp_last_record_us = None
        self.gyro_temp_record_active = False
        self.btn_temp_rec.configure(text="Start T–gyro capture")
        self._refresh_gyro_temp_fit_button()
        self.gyro_temp_status_var.set(
            "T–gyro: cleared samples and model. Need T_die in stream to capture again."
        )
        self.log("Cleared T–gyro samples and fit coefficients.")
        if self.selected_mac:
            latest = self.latest_by_mac.get(self.selected_mac)
            if latest is not None:
                self.reset_corrected_history()
                self.update_live_view(latest)

    def on_gyro_temp_apply_toggle(self):
        self.reset_corrected_history()
        if self.selected_mac:
            latest = self.latest_by_mac.get(self.selected_mac)
            if latest is not None:
                self.update_live_view(latest)

    def _clear_gyro_temp_on_mac_change(self):
        if not (
            self.gyro_temp_samples
            or self.gyro_temp_coef is not None
            or self.gyro_temp_record_active
        ):
            return
        self.gyro_temp_samples = []
        self.gyro_temp_coef = None
        self.gyro_temp_record_active = False
        self._gyro_temp_last_record_us = None
        self.btn_temp_rec.configure(text="Start T–gyro capture")
        self._refresh_gyro_temp_fit_button()
        self.gyro_temp_apply_var.set(False)
        self.gyro_temp_status_var.set("T–gyro: reset after MAC change.")
        self.log("MAC changed — cleared gyro vs temp samples/model (one model per module).")

    def should_accept_capture_sample(self, row):
        if self.capture_start_micros is not None and row["micros"] <= self.capture_start_micros:
            return False
        if self.capture_target is not None:
            ok, _ = pose_alignment_check(
                (row["ax"], row["ay"], row["az"]),
                self.capture_target,
            )
            return ok
        return True

    def ensure_mac(self, mac):
        if mac in self.known_macs:
            return
        self.known_macs.append(mac)
        self.known_macs.sort()
        self.refresh_mac_list()
        self.log(f"New MAC detected: {mac}")

    def refresh_mac_list(self):
        current = self.selected_mac
        self.mac_listbox.delete(0, tk.END)
        for idx, mac in enumerate(self.known_macs):
            self.mac_listbox.insert(tk.END, mac)
            if mac == current:
                self.mac_listbox.selection_set(idx)

    def on_select_mac(self, _event=None):
        selection = self.mac_listbox.curselection()
        if not selection:
            return
        mac = self.mac_listbox.get(selection[0])
        self.selected_mac = mac
        self.reset_corrected_history()
        self._clear_gyro_temp_on_mac_change()
        self.selected_mac_var.set(f"MAC: {mac}")
        self.log(f"Selected module: {mac}")
        latest = self.latest_by_mac.get(mac)
        if latest is not None:
            self.update_live_view(latest)

    def update_live_view(self, row):
        self._ingest_live_sample(row)
        self._refresh_live_labels(row)

    def _refresh_live_labels(self, row):
        acc_norm = math.sqrt(row["ax"] ** 2 + row["ay"] ** 2 + row["az"] ** 2)
        if row.get("temp_c") is not None:
            temp_line = "T_die = {:.2f} °C".format(float(row["temp_c"]))
        else:
            temp_line = "T_die = — (not in serial stream)"
        extra_bias = ""
        if self.gyro_temp_coef is not None and row.get("temp_c") is not None:
            gbT = self.gyro_temp_coef[:, 0] + self.gyro_temp_coef[:, 1] * float(
                row["temp_c"]
            )
            tag = (
                " [applied in plot]"
                if self.gyro_temp_apply_var.get()
                else " [enable «Apply gyro correction…» for plot]"
            )
            extra_bias = (
                "\nGyro bias(T) fit: {:.4f}, {:.4f}, {:.4f} °/s{}".format(
                    gbT[0], gbT[1], gbT[2], tag
                )
            )
        self.live_var.set(
            (
                "ax={:.4f}  ay={:.4f}  az={:.4f}\n"
                "gx={:.4f}  gy={:.4f}  gz={:.4f}\n"
                "{}\n"
                "mx={:.4f}  my={:.4f}  mz={:.4f}\n"
                "|acc|={:.4f}  ts={}  us={}"
            ).format(
                row["ax"], row["ay"], row["az"],
                row["gx"], row["gy"], row["gz"],
                temp_line,
                row["mx"], row["my"], row["mz"],
                acc_norm, row["timestamp"], row["micros"],
            )
            + extra_bias
        )
        ori_text, ori_fg = self.estimate_orientation_text(row, acc_norm)
        self.orientation_var.set(ori_text)
        self.orientation_label.config(fg=ori_fg)
        self.update_bubble_level(row["ax"], row["ay"], row["az"])

        if self.bias is not None and self.scale is not None and self.latest_corrected is not None:
            corrected = self.latest_corrected
            corrected_norm = float(np.linalg.norm(corrected))
            self.corrected_var.set(
                "ax_c={:.4f}  ay_c={:.4f}  az_c={:.4f}\n|acc_c|={:.4f}  global_scale={:.6f}".format(
                    corrected[0], corrected[1], corrected[2], corrected_norm, self.global_scale
                )
            )
            quality_text, quality_color = self.evaluate_calibration_quality(corrected_norm)
            self.calib_quality_var.set(quality_text)
            self.calib_quality_label.config(fg=quality_color)
        else:
            self.corrected_var.set("No calib data yet")
            self.calib_quality_var.set("Calib quality: none yet")
            self.calib_quality_label.config(fg="black")

    def estimate_orientation_text(self, row, acc_norm):
        axes = {
            "X": float(row["ax"]),
            "Y": float(row["ay"]),
            "Z": float(row["az"]),
        }
        dominant_axis = max(axes, key=lambda key: abs(axes[key]))
        dominant_value = axes[dominant_axis]
        other_values = [abs(value) for key, value in axes.items() if key != dominant_axis]
        dominant_abs = abs(dominant_value)
        second_abs = max(other_values) if other_values else 0.0

        if acc_norm < 0.5:
            return ("Rough pose: unknown (|acc| too small)", "red")

        if dominant_abs < 0.75 * acc_norm:
            return ("Rough pose: unknown (not aligned to one axis)", "red")

        if second_abs > 0.45 * dominant_abs:
            return ("Rough pose: unknown (two axes equally large)", "red")

        sign = "+" if dominant_value >= 0 else "-"
        sec_hint = ""
        if second_abs <= POSE_MAX_SECONDARY_ABS_MS2:
            sec_hint = f"\nSecondary axes OK (≤{POSE_MAX_SECONDARY_ABS_MS2} m/s²)"
            fg = "green"
        else:
            sec_hint = (
                f"\nBring max|2 secondary axes| ≤ {POSE_MAX_SECONDARY_ABS_MS2} m/s² "
                f"(now {second_abs:.3f})"
            )
            fg = "red"
        return (
            f"Rough pose: {sign}{dominant_axis}  "
            f"(axis {dominant_axis} = {dominant_value:.3f} m/s², |acc| = {acc_norm:.3f}){sec_hint}",
            fg,
        )

    def evaluate_calibration_quality(self, corrected_norm):
        error = abs(corrected_norm - G)
        if error <= 0.2:
            return (
                f"Calib quality: EXCELLENT (|acc_c|-g = {error:.3f} m/s²)",
                "green",
            )
        if error <= 0.5:
            return (
                f"Calib quality: ACCEPTABLE (|acc_c|-g = {error:.3f} m/s²)",
                "#b8860b",
            )
        return (
            f"Calib quality: RECALIBRATE NEEDED (|acc_c|-g = {error:.3f} m/s²)",
            "red",
        )

    def analyze_capture_samples(self, samples):
        stats = {}
        axis_names = ["ax", "ay", "az"]
        for idx, axis_name in enumerate(axis_names):
            axis_values = samples[:, idx]
            axis_min = float(np.min(axis_values))
            axis_max = float(np.max(axis_values))
            axis_std = float(np.std(axis_values))
            stats[axis_name] = {
                "min": axis_min,
                "max": axis_max,
                "std": axis_std,
                "range": axis_max - axis_min,
            }
        return stats

    def validate_capture_stats(self, stats):
        failures = []
        for axis_name, axis_stats in stats.items():
            if axis_stats["range"] > CAPTURE_MAX_AXIS_RANGE:
                failures.append(
                    f"{axis_name}: range={axis_stats['range']:.3f} > {CAPTURE_MAX_AXIS_RANGE:.3f}"
                )
            if axis_stats["std"] > CAPTURE_MAX_AXIS_STD:
                failures.append(
                    f"{axis_name}: std={axis_stats['std']:.3f} > {CAPTURE_MAX_AXIS_STD:.3f}"
                )
        return failures

    def format_capture_stats(self, stats):
        return " | ".join(
            (
                f"{axis}: min={axis_stats['min']:.3f}, "
                f"max={axis_stats['max']:.3f}, "
                f"range={axis_stats['range']:.3f}, "
                f"std={axis_stats['std']:.3f}"
            )
            for axis, axis_stats in stats.items()
        )

    def validate_capture_pose(self, pose, mean_vec):
        _ok, failures = pose_alignment_check(mean_vec, pose)
        return failures

    def start_calibration(self):
        if self.calibration_running:
            self.log("Calibration already in progress.")
            return

        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to serial.")
            return

        if not self.selected_mac:
            messagebox.showerror("Error", "Select a MAC before calibration.")
            return

        clear_existing = messagebox.askyesno(
            "Clear calib on module?",
            "Before starting calibration, clear all calibration stored on the target module?\n\n"
            "If YES, the app sends a clear-calib command before capture.",
        )
        if clear_existing:
            if not self.send_clear_calibration_command():
                return
            self.log("Cleared calibration on module before capture.")
            self.root.update_idletasks()
            time.sleep(0.2)

        self.calibration_running = True
        self.capture_mode = "calibration"
        self.pose_results = []
        self.validation_results = []
        self.pose_index = 0
        self.bias = None
        self.scale = None
        self.global_scale = 1.0
        self.capture_target = None
        self.capture_samples = []
        self.capture_gyro_batch = []
        self.calib_gyro_accum = []
        self.capture_start_micros = None
        self.capture_first_sample_micros = None
        self.capture_last_sample_micros = None
        self.corrected_var.set("No calib data yet")
        self.reset_corrected_history()
        self.log(f"Starting 6-pose calibration for MAC {self.selected_mac}")
        self.root.after(100, self.run_next_pose)

    def start_validation(self):
        if self.calibration_running:
            self.log("Another capture session is already running.")
            return

        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to serial.")
            return

        if not self.selected_mac:
            messagebox.showerror("Error", "Select a MAC before validation.")
            return

        if self.bias is None or self.scale is None:
            messagebox.showerror(
                "Error",
                "No calibration parameters on PC.\n\n"
                "Run 6-pose calibration or press «Get calib from module (GETCALIB)».",
            )
            return

        self.calibration_running = True
        self.capture_mode = "validation"
        self.validation_results = []
        self.pose_index = 0
        self.capture_target = None
        self.capture_samples = []
        self.capture_start_micros = None
        self.capture_first_sample_micros = None
        self.capture_last_sample_micros = None
        self.log(f"Starting 6-face validation for MAC {self.selected_mac}")
        self.root.after(100, self.run_next_pose)

    def send_calibration_to_module(self):
        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to serial (master).")
            return

        if not self.selected_mac:
            messagebox.showerror("Error", "Select the MAC to send calibration to.")
            return

        if self.bias is None or self.scale is None:
            messagebox.showerror("Error", "No calibration data to send.")
            return

        gb = self.gyro_bias
        command = (
            f"CALIB,{self.selected_mac},"
            f"{self.bias[0]:.8f},{self.bias[1]:.8f},{self.bias[2]:.8f},"
            f"{self.scale[0]:.8f},{self.scale[1]:.8f},{self.scale[2]:.8f},"
            f"{self.global_scale:.8f},"
            f"{gb[0]:.8f},{gb[1]:.8f},{gb[2]:.8f}\n"
        )
        try:
            self.ser.write(command.encode("utf-8"))
            self.ser.flush()
            self.log(
                "Sent CALIB to module {} | acc bias={} | scale={} | gscale={:.8f} | gyro_bias={}".format(
                    self.selected_mac,
                    tuple(round(float(v), 8) for v in self.bias),
                    tuple(round(float(v), 8) for v in self.scale),
                    self.global_scale,
                    tuple(round(float(v), 8) for v in gb),
                )
            )
            if np.allclose(gb, 0.0):
                self.log(
                    "Hint: gyro_bias is 0 — after 6-pose calib with enough gyro samples, "
                    "press «Send Calib To Module» to write gyro to the board (no need to redo 6 faces)."
                )
        except Exception as exc:
            messagebox.showerror("Calib send error", str(exc))

    def clear_calibration_on_module(self):
        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to serial (master).")
            return

        if not self.selected_mac:
            messagebox.showerror("Error", "Select the MAC to clear calibration on.")
            return

        self.send_clear_calibration_command()

    def apply_calib_report_from_serial_line(self, line):
        """
        CALIB_REPORT,<MAC>,ENABLED|DISABLED,bx,by,bz,sx,sy,sz,global_scale[,gxb,gyb,gzb]
        (from Master when CALREP packet is received from slave).
        """
        parts = line.strip().split(",")
        if len(parts) < 10 or parts[0] != "CALIB_REPORT":
            return
        mac_report = parts[1].strip().upper()
        if not self.selected_mac:
            self.log("CALIB_REPORT: no MAC selected in list — not applied on PC.")
            return
        if norm_mac_compact(mac_report) != norm_mac_compact(self.selected_mac):
            self.log(
                "CALIB_REPORT: MAC differs from selected module — not applying bias/scale on PC."
            )
            return
        try:
            bx, by, bz = float(parts[3]), float(parts[4]), float(parts[5])
            sx, sy, sz = float(parts[6]), float(parts[7]), float(parts[8])
            gs = float(parts[9])
        except (ValueError, IndexError):
            self.log("CALIB_REPORT: failed to parse numbers.")
            return
        self.bias = np.array([bx, by, bz], dtype=float)
        self.scale = np.array([sx, sy, sz], dtype=float)
        self.global_scale = gs
        if len(parts) >= 13:
            try:
                self.gyro_bias = np.array(
                    [float(parts[10]), float(parts[11]), float(parts[12])], dtype=float
                )
            except (ValueError, IndexError):
                self.gyro_bias = np.zeros(3, dtype=float)
        else:
            self.gyro_bias = np.zeros(3, dtype=float)
        en = parts[2].strip().upper() == "ENABLED"
        latest = self.latest_by_mac.get(self.selected_mac) if self.selected_mac else None
        if latest is not None:
            self.reset_corrected_history()
            self.update_live_view(latest)
        self.log(
            f"Applied parameters from module on PC ({mac_report}, ENABLED={en}). "
            "You can run Validate 6 Faces or open the plot."
        )
        if (
            np.allclose(self.bias, 0.0)
            and np.allclose(self.scale, 1.0)
            and abs(float(self.global_scale) - 1.0) < 1e-6
        ):
            self.log(
                "Hint: board reports identity (0/1/1) — often because «Send Calib To Module» "
                "was not used after PC calibration, or CLRCAL was run."
            )
        if len(parts) >= 13 and np.allclose(self.gyro_bias, 0.0):
            self.log(
                "Hint: gyro_bias on board = 0 — if PC has mean gyro from 6-pose calib, "
                "press «Send Calib To Module» (one send for accel + gyro). "
                "Master firmware must support full CALIB packet (≈76 bytes)."
            )

    def send_get_calibration_from_module(self):
        if self.ser is None or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to serial (master).")
            return
        if not self.selected_mac:
            messagebox.showerror("Error", "Select the IMU module MAC to read parameters from.")
            return
        command = f"GETCALIB,{self.selected_mac}\n"
        try:
            self.ser.write(command.encode("utf-8"))
            self.ser.flush()
            self.log(f"Sent to master: {command.strip()}")
        except Exception as exc:
            messagebox.showerror("GETCALIB error", str(exc))
            self.log(f"GETCALIB send failed: {exc}")

    def send_clear_calibration_command(self):
        command = f"CLRCAL,{self.selected_mac}\n"
        try:
            self.ser.write(command.encode("utf-8"))
            self.ser.flush()
            self.log(f"Sent clear calib to module {self.selected_mac}: {command.strip()}")
            return True
        except Exception as exc:
            messagebox.showerror("Clear calib error", str(exc))
            self.log(f"Clear calib send failed to module {self.selected_mac}: {exc}")
            return False

    def run_next_pose(self):
        if not self.calibration_running:
            return

        if self.pose_index >= len(POSES):
            try:
                if self.capture_mode == "calibration":
                    self.compute_calibration()
                elif self.capture_mode == "validation":
                    self.finish_validation()
            except Exception as exc:
                self.log(f"Calibration error: {exc}")
                messagebox.showerror("Calibration error", str(exc))
            finally:
                self.capture_mode = None
                self.capture_target = None
                self.bubble_target_pose = None
                self.capture_samples = []
                self.capture_gyro_batch = []
                self.calibration_running = False
            return

        pose = POSES[self.pose_index]
        self.bubble_target_pose = pose
        mode_label = "calib" if self.capture_mode == "calibration" else "validation"
        messagebox.showinfo(
            "Place IMU pose",
            f"Confirm correct module MAC:\n{self.selected_mac}\n\n"
            f"Mode: {mode_label}\n\n"
            f"Place IMU in pose:\n{pose}\n\n"
            f"The two axes that are not pointing up should be near 0 "
            f"(≤ {POSE_MAX_SECONDARY_ABS_MS2} m/s²). "
            "Samples are taken only when alignment is straight enough.\n\n"
            f"Hold the sensor still, then press OK to capture {SAMPLES_PER_POSE} valid samples.",
        )
        latest = self.latest_by_mac.get(self.selected_mac)
        self.capture_target = pose
        self.capture_samples = []
        self.capture_gyro_batch = []
        self.capture_start_micros = latest["micros"] if latest is not None else None
        self.capture_first_sample_micros = None
        self.capture_last_sample_micros = None
        self.capture_deadline = time.time() + CAPTURE_TIMEOUT_S
        if self.capture_start_micros is None:
            self.log(f"Capturing pose {pose}... (no baseline timestamp yet, waiting for new sample)")
        else:
            self.log(
                f"Capturing pose {pose}... "
                    f"discarding data up to micros={self.capture_start_micros}, "
                    f"timeout={CAPTURE_TIMEOUT_S:.0f}s"
            )
        self.root.after(20, self.check_capture_progress)

    def check_capture_progress(self):
        if not self.calibration_running or self.capture_target is None:
            return

        capture_duration_us = 0
        if self.capture_first_sample_micros is not None and self.capture_last_sample_micros is not None:
            capture_duration_us = self.capture_last_sample_micros - self.capture_first_sample_micros

        if len(self.capture_samples) >= SAMPLES_PER_POSE and capture_duration_us >= MIN_CAPTURE_DURATION_US:
            capture_array = np.array(self.capture_samples, dtype=float)
            stats = self.analyze_capture_samples(capture_array)
            stats_text = self.format_capture_stats(stats)
            pose = self.capture_target
            mean_vec = np.mean(capture_array, axis=0)
            self.log(f"{pose}: sample stats | {stats_text}")
            self.log(
                f"{pose}: provisional mean = "
                f"[{mean_vec[0]:.6f}, {mean_vec[1]:.6f}, {mean_vec[2]:.6f}]"
            )

            failures = self.validate_capture_stats(stats)
            failures.extend(self.validate_capture_pose(pose, mean_vec))
            if failures:
                self.capture_target = None
                self.capture_samples = []
                self.capture_gyro_batch = []
                self.capture_start_micros = None
                self.capture_first_sample_micros = None
                self.capture_last_sample_micros = None
                failure_text = "; ".join(failures)
                self.log(f"{pose}: data not stable enough, retry pose | {failure_text}")
                messagebox.showwarning(
                    "Redo capture",
                    f"Pose {pose} is not stable enough for calibration.\n\n"
                    f"{stats_text}\n\n"
                    f"Reason: {failure_text}\n\n"
                    "Hold the IMU steadier and repeat this pose.",
                )
                self.root.after(200, self.run_next_pose)
                return

            if self.capture_mode == "calibration":
                self.pose_results.append(mean_vec)
                if len(self.capture_gyro_batch) == len(self.capture_samples):
                    self.calib_gyro_accum.extend(self.capture_gyro_batch)
                else:
                    self.log(
                        f"{pose}: warning — gyro sample count ({len(self.capture_gyro_batch)}) "
                        f"≠ accel ({len(self.capture_samples)}); skipping gyro for this pose."
                    )
                self.log(
                    f"{pose}: mean accel = "
                    f"[{mean_vec[0]:.6f}, {mean_vec[1]:.6f}, {mean_vec[2]:.6f}] | "
                    f"samples={len(self.capture_samples)} | duration={capture_duration_us / 1e6:.2f}s | OK"
                )
            elif self.capture_mode == "validation":
                corrected_mean = (mean_vec - self.bias) * self.scale * self.global_scale
                corrected_norm = float(np.linalg.norm(corrected_mean))
                norm_error = corrected_norm - G
                self.validation_results.append(
                    {
                        "pose": pose,
                        "raw_mean": mean_vec,
                        "corrected_mean": corrected_mean,
                        "corrected_norm": corrected_norm,
                        "norm_error": norm_error,
                    }
                )
                self.log(
                    f"{pose}: acc_c_mean = "
                    f"[{corrected_mean[0]:.6f}, {corrected_mean[1]:.6f}, {corrected_mean[2]:.6f}] | "
                    f"|acc_c|={corrected_norm:.6f} | err={norm_error:+.6f}"
                )
            self.capture_target = None
            self.capture_samples = []
            self.capture_gyro_batch = []
            self.capture_start_micros = None
            self.capture_first_sample_micros = None
            self.capture_last_sample_micros = None
            self.pose_index += 1
            self.root.after(200, self.run_next_pose)
            return

        if time.time() > self.capture_deadline:
            pose = self.capture_target
            self.capture_target = None
            self.capture_samples = []
            self.capture_gyro_batch = []
            self.capture_start_micros = None
            self.capture_first_sample_micros = None
            self.capture_last_sample_micros = None
            self.calibration_running = False
            msg = (
                f"Timeout while capturing pose {pose}. "
                "Check the selected module and serial data stream."
            )
            self.log(f"Calibration error: {msg}")
            messagebox.showerror("Calibration error", msg)
            return

        self.root.after(20, self.check_capture_progress)

    def finish_validation(self):
        if len(self.validation_results) != len(POSES):
            raise ValueError("Need all 6 poses to finish validation.")

        self.log("")
        self.log("===== VALIDATION 6 FACES =====")
        max_abs_error = 0.0
        for item in self.validation_results:
            max_abs_error = max(max_abs_error, abs(item["norm_error"]))
            corrected_mean = item["corrected_mean"]
            self.log(
                "{}: |acc_c|={:.6f} | err={:+.6f} | acc_c_mean=[{:.6f}, {:.6f}, {:.6f}]".format(
                    item["pose"],
                    item["corrected_norm"],
                    item["norm_error"],
                    corrected_mean[0],
                    corrected_mean[1],
                    corrected_mean[2],
                )
            )
        self.log(f"Max |err| across 6 faces = {max_abs_error:.6f} m/s²")

    def compute_calibration(self):
        if len(self.pose_results) != 6:
            raise ValueError("Need all 6 poses to compute calibration.")

        results = np.array(self.pose_results, dtype=float)
        ax_p, ax_n = results[0], results[1]
        ay_p, ay_n = results[2], results[3]
        az_p, az_n = results[4], results[5]

        denom_x = ax_p[0] - ax_n[0]
        denom_y = ay_p[1] - ay_n[1]
        denom_z = az_p[2] - az_n[2]

        if abs(denom_x) < 1e-6 or abs(denom_y) < 1e-6 or abs(denom_z) < 1e-6:
            raise ValueError("Invalid calibration data: difference between opposing faces too small.")

        self.bias = np.array(
            [
                (ax_p[0] + ax_n[0]) / 2.0,
                (ay_p[1] + ay_n[1]) / 2.0,
                (az_p[2] + az_n[2]) / 2.0,
            ],
            dtype=float,
        )
        self.scale = np.array(
            [
                2.0 * G / denom_x,
                2.0 * G / denom_y,
                2.0 * G / denom_z,
            ],
            dtype=float,
        )

        corrected_pose_results = (results - self.bias) * self.scale
        corrected_pose_norms = np.linalg.norm(corrected_pose_results, axis=1)
        mean_corrected_norm = float(np.mean(corrected_pose_norms))
        if mean_corrected_norm < 1e-6:
            raise ValueError("Cannot compute global scale: mean norm too small.")
        self.global_scale = G / mean_corrected_norm
        corrected_pose_results *= self.global_scale
        corrected_pose_norms *= self.global_scale

        if self.calib_gyro_accum:
            gstack = np.array(self.calib_gyro_accum, dtype=float)
            self.gyro_bias = np.mean(gstack, axis=0)
        else:
            self.gyro_bias = np.zeros(3, dtype=float)

        self.log("")
        self.log("===== CALIBRATION RESULTS =====")
        self.log(
            "Bias  = [{:.8f}, {:.8f}, {:.8f}]".format(
                self.bias[0], self.bias[1], self.bias[2]
            )
        )
        self.log(
            "Scale = [{:.8f}, {:.8f}, {:.8f}]".format(
                self.scale[0], self.scale[1], self.scale[2]
            )
        )
        self.log(f"Global scale = {self.global_scale:.8f}")
        if self.calib_gyro_accum:
            self.log(
                "Gyro bias (mean {} gyro samples, captured with IMU still per pose) = "
                "[{:.6f}, {:.6f}, {:.6f}] °/s".format(
                    len(self.calib_gyro_accum),
                    float(self.gyro_bias[0]),
                    float(self.gyro_bias[1]),
                    float(self.gyro_bias[2]),
                )
            )
        else:
            self.log("Gyro bias = [0, 0, 0] °/s (no gyro samples — check serial stream).")
        self.log(
            "Mean ||acc|| over 6 poses before global scale = "
            f"{mean_corrected_norm:.6f} m/s²"
        )
        self.log(
            "||acc|| for 6 poses after global scale = "
            + ", ".join(f"{val:.6f}" for val in corrected_pose_norms)
        )
        self.log("Formula: accel_calib = (accel_raw - bias) * scale * global_scale")

        latest = self.latest_by_mac.get(self.selected_mac)
        if latest is not None:
            self.reset_corrected_history()
            self.update_live_view(latest)
            if self.latest_corrected is not None:
                self.log(
                    "Current sample after calibration = "
                    "[{:.6f}, {:.6f}, {:.6f}]".format(
                        self.latest_corrected[0],
                        self.latest_corrected[1],
                        self.latest_corrected[2],
                    )
                )

        self.log(
            "Note: GETCALIB reads parameters stored on the module (already sent via ESP-NOW). "
            "Without «Send Calib To Module» the slave stays bias=0, scale=1 (neutral)."
        )
        gb = self.gyro_bias
        gyro_hint = (
            f"Gyro bias on PC (sent with CALIB): ({gb[0]:.6f}, {gb[1]:.6f}, {gb[2]:.6f}) °/s "
            "(from 6-pose calib if enough gyro samples per pose, hold still)."
        )
        if self.ser is not None and self.ser.is_open and self.selected_mac:
            if messagebox.askyesno(
                "Send calib to module?",
                "To match GETCALIB / CALIB_REPORT accel with the table above, send parameters to slave.\n\n"
                + gyro_hint
                + "\n\nSend accel (+ current gyro_bias) now?",
            ):
                self.send_calibration_to_module()

    def clear_calibration(self):
        self.bubble_target_pose = None
        self.pose_results = []
        self.bias = None
        self.scale = None
        self.global_scale = 1.0
        self.latest_corrected = None
        self.reset_corrected_history()
        self.corrected_var.set("No calib data yet")
        self.calib_quality_var.set("Calib quality: none yet")
        self.calib_quality_label.config(fg="black")
        self.gyro_bias = np.zeros(3, dtype=float)
        self.calib_gyro_accum = []
        self.capture_gyro_batch = []
        self.gyro_ma_long_buffer.clear()
        self.gyro_temp_apply_var.set(False)
        self.gyro_temp_samples = []
        self.gyro_temp_coef = None
        self.gyro_temp_record_active = False
        self._gyro_temp_last_record_us = None
        self.btn_temp_rec.configure(text="Start T–gyro capture")
        self._refresh_gyro_temp_fit_button()
        self.gyro_temp_status_var.set(
            "T–gyro: cleared with calibration (samples + model)."
        )
        self.log("Cleared current calibration results.")

    def reset_corrected_history(self):
        for values in self.corrected_history.values():
            values.clear()
        self.gyro_ma_long_buffer.clear()

    def append_corrected_sample(self, row, corrected, corrected_norm):
        self.corrected_history["norm"].append(float(corrected_norm))
        inv_scale = self.scale * self.global_scale
        raw_from_calib = corrected / inv_scale + self.bias
        self.corrected_history["acc_raw_norm"].append(float(np.linalg.norm(raw_from_calib)))

        gyro = np.array([row["gx"], row["gy"], row["gz"]], dtype=float)
        gb = self.effective_gyro_bias(row)
        g_corr = gyro - gb
        self.corrected_history["gxc"].append(float(g_corr[0]))
        self.corrected_history["gyc"].append(float(g_corr[1]))
        self.corrected_history["gzc"].append(float(g_corr[2]))
        self.corrected_history["gxr"].append(float(gyro[0]))
        self.corrected_history["gyr"].append(float(gyro[1]))
        self.corrected_history["gzr"].append(float(gyro[2]))

        self.gyro_ma_long_buffer.append(
            (float(g_corr[0]), float(g_corr[1]), float(g_corr[2]))
        )

    def auto_set_y_limits(self, axis, values, include_zero=False, reference=None):
        if not values:
            return

        data_min = min(values)
        data_max = max(values)

        if include_zero:
            data_min = min(data_min, 0.0)
            data_max = max(data_max, 0.0)

        if reference is not None:
            data_min = min(data_min, reference)
            data_max = max(data_max, reference)

        span = data_max - data_min
        if span < PLOT_MIN_SPAN:
            center = 0.5 * (data_max + data_min)
            half = 0.5 * PLOT_MIN_SPAN
            data_min = center - half
            data_max = center + half
            span = data_max - data_min

        margin = max(PLOT_MIN_SPAN * 0.1, span * PLOT_Y_MARGIN_RATIO)
        axis.set_ylim(data_min - margin, data_max + margin)

    def show_calibrated_plot(self):
        if self.bias is None or self.scale is None:
            messagebox.showerror(
                "Error",
                "No calibration parameters on PC.\n\n"
                "Run 6-pose calibration or «Get calib from module (GETCALIB)».",
            )
            return

        if self.selected_mac is None:
            messagebox.showerror("Error", "Select the MAC to monitor.")
            return

        if self.plot_fig is not None and plt.fignum_exists(self.plot_fig.number):
            self.plot_fig.canvas.manager.set_window_title(f"Calibrated Live Plot - {self.selected_mac}")
            self.plot_fig.show()
            return

        fig, (ax_acc, ax_gyro, ax_gma) = plt.subplots(3, 1, sharex=False, figsize=(10, 10))
        fig.suptitle(f"Live MAV — |acc| & gyro — {self.selected_mac}")
        fig.canvas.manager.set_window_title(f"Calibrated Live Plot - {self.selected_mac}")

        ax_acc.set_ylabel("|acc| (m/s²)")
        ax_acc.grid(True, alpha=0.3)
        ax_acc.set_xlim(0, max(1, PLOT_BUFFER_SIZE - 1))
        ax_acc.set_ylim(0, 14)
        ax_acc.axhline(y=G, color="red", linestyle=":", alpha=0.45, linewidth=0.8)

        ax_gyro.set_ylabel("ω (°/s)")
        ax_gyro.grid(True, alpha=0.3)
        ax_gyro.set_xlim(0, max(1, PLOT_BUFFER_SIZE - 1))
        ax_gyro.set_ylim(-200, 200)

        ma_n = GYRO_MA_DISPLAY_N
        ax_gma.set_ylabel(f"ω calib MA{ma_n} (°/s)")
        ax_gma.set_xlabel(
            f"Sample index (window max {GYRO_MA_LONG_BUFFER_LEN} ≈ {GYRO_MA_LONG_BUFFER_LEN / 100:.0f} s @ 100 Hz)"
        )
        ax_gma.grid(True, alpha=0.3)
        ax_gma.set_ylim(-20, 20)

        ma_lbl = f"MA{LIVE_PLOT_MA_WINDOW}"
        line_acc_nc_ma, = ax_acc.plot(
            [], [], color="#ff7f0e", lw=2.0, alpha=0.92, label=f"|acc|c ({ma_lbl})", zorder=4
        )
        line_acc_nr_ma, = ax_acc.plot(
            [], [], color="#555555", lw=2.0, linestyle=":", alpha=0.92, label=f"|acc|r ({ma_lbl})", zorder=4
        )

        line_gxc_ma, = ax_gyro.plot(
            [], [], color="r", lw=2.0, alpha=0.85, label=f"gxc ({ma_lbl})", zorder=4
        )
        line_gyc_ma, = ax_gyro.plot(
            [], [], color="g", lw=2.0, alpha=0.85, label=f"gyc ({ma_lbl})", zorder=4
        )
        line_gzc_ma, = ax_gyro.plot(
            [], [], color="b", lw=2.0, alpha=0.85, label=f"gzc ({ma_lbl})", zorder=4
        )
        line_gxr_ma, = ax_gyro.plot(
            [], [], color="#c66", lw=1.7, linestyle=":", alpha=0.88, label=f"gxr ({ma_lbl})", zorder=4
        )
        line_gyr_ma, = ax_gyro.plot(
            [], [], color="#6a6", lw=1.7, linestyle=":", alpha=0.88, label=f"gyr ({ma_lbl})", zorder=4
        )
        line_gzr_ma, = ax_gyro.plot(
            [], [], color="#66c", lw=1.7, linestyle=":", alpha=0.88, label=f"gzr ({ma_lbl})", zorder=4
        )

        line_gma_x, = ax_gma.plot(
            [], [], color="r", lw=1.6, alpha=0.9, label=f"gx MA{ma_n} (ω calib)"
        )
        line_gma_y, = ax_gma.plot(
            [], [], color="g", lw=1.6, alpha=0.9, label=f"gy MA{ma_n} (ω calib)"
        )
        line_gma_z, = ax_gma.plot(
            [], [], color="b", lw=1.6, alpha=0.9, label=f"gz MA{ma_n} (ω calib)"
        )

        ax_acc.legend(loc="upper right", fontsize=8)
        ax_gyro.legend(loc="upper right", fontsize=7, ncol=2)
        ax_gma.legend(loc="upper right", fontsize=8)

        text_acc_mean = ax_acc.text(
            0.02,
            0.98,
            "",
            transform=ax_acc.transAxes,
            fontsize=8,
            family="monospace",
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.82),
            zorder=6,
        )
        text_gyro_mean = ax_gyro.text(
            0.02,
            0.98,
            "",
            transform=ax_gyro.transAxes,
            fontsize=7,
            family="monospace",
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.82),
            zorder=6,
        )
        text_gma_hint = ax_gma.text(
            0.02,
            0.98,
            "",
            transform=ax_gma.transAxes,
            fontsize=7,
            family="monospace",
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="honeydew", alpha=0.85),
            zorder=6,
        )

        acc_ma_lines = {"norm": line_acc_nc_ma, "acc_raw_norm": line_acc_nr_ma}
        gyro_ma_calib_lines = {"gxc": line_gxc_ma, "gyc": line_gyc_ma, "gzc": line_gzc_ma}
        gyro_ma_raw_lines = {"gxr": line_gxr_ma, "gyr": line_gyr_ma, "gzr": line_gzr_ma}
        self.plot_fig = fig
        gma_lines = {"x": line_gma_x, "y": line_gma_y, "z": line_gma_z}
        self.plot_live_artists = [
            *acc_ma_lines.values(),
            *gyro_ma_calib_lines.values(),
            *gyro_ma_raw_lines.values(),
            *gma_lines.values(),
            text_acc_mean,
            text_gyro_mean,
            text_gma_hint,
        ]

        def init():
            for ln in self.plot_live_artists:
                if hasattr(ln, "set_data"):
                    ln.set_data([], [])
            text_acc_mean.set_text("")
            text_gyro_mean.set_text("")
            text_gma_hint.set_text("")
            return self.plot_live_artists

        def update(_frame):
            x_vals = np.arange(len(self.corrected_history["norm"]))
            ma_w = LIVE_PLOT_MA_WINDOW
            acc_vals_flat = []
            vals_nc = list(self.corrected_history["norm"])
            vals_nr = list(self.corrected_history["acc_raw_norm"])
            for key in ("norm", "acc_raw_norm"):
                vals = list(self.corrected_history[key])
                ma_vals = trailing_moving_average(vals, ma_w)
                acc_ma_lines[key].set_data(x_vals, ma_vals)
                acc_vals_flat.extend(list(ma_vals))

            if len(vals_nc) > 0:
                mnc = float(np.mean(np.asarray(vals_nc, dtype=float)))
                mnr = float(np.mean(np.asarray(vals_nr, dtype=float)))
                text_acc_mean.set_text(
                    f"Mean buffer |acc|: calib={mnc:.4f}  raw={mnr:.4f}  Δ={mnc - mnr:+.4f} m/s²"
                )
            else:
                text_acc_mean.set_text("Mean buffer |acc|: — (no samples yet)")

            gyro_vals_flat = []
            for key in ("gxc", "gyc", "gzc"):
                vals = list(self.corrected_history[key])
                ma_vals = trailing_moving_average(vals, ma_w)
                gyro_ma_calib_lines[key].set_data(x_vals, ma_vals)
                gyro_vals_flat.extend(list(ma_vals))
            for key in ("gxr", "gyr", "gzr"):
                vals = list(self.corrected_history[key])
                ma_vals = trailing_moving_average(vals, ma_w)
                gyro_ma_raw_lines[key].set_data(x_vals, ma_vals)
                gyro_vals_flat.extend(list(ma_vals))

            if len(self.corrected_history["gxc"]) > 0:
                mc = [
                    float(np.mean(np.asarray(list(self.corrected_history[k]), dtype=float)))
                    for k in ("gxc", "gyc", "gzc")
                ]
                mr = [
                    float(np.mean(np.asarray(list(self.corrected_history[k]), dtype=float)))
                    for k in ("gxr", "gyr", "gzr")
                ]
                text_gyro_mean.set_text(
                    "Mean buffer ω (°/s):\n"
                    f"  gx: calib={mc[0]:+.2f}  raw={mr[0]:+.2f}  Δ={mc[0]-mr[0]:+.2f}\n"
                    f"  gy: calib={mc[1]:+.2f}  raw={mr[1]:+.2f}  Δ={mc[1]-mr[1]:+.2f}\n"
                    f"  gz: calib={mc[2]:+.2f}  raw={mr[2]:+.2f}  Δ={mc[2]-mr[2]:+.2f}"
                )
            else:
                text_gyro_mean.set_text("Mean buffer ω: — (no samples yet)")

            buf = list(self.gyro_ma_long_buffer)
            gma_flat = []
            ma_n = GYRO_MA_DISPLAY_N
            if len(buf) > 0:
                gx = np.array([b[0] for b in buf], dtype=float)
                gy = np.array([b[1] for b in buf], dtype=float)
                gz = np.array([b[2] for b in buf], dtype=float)
                mx = trailing_moving_average(gx, ma_n)
                my = trailing_moving_average(gy, ma_n)
                mz = trailing_moving_average(gz, ma_n)
                x_m = np.arange(len(mx), dtype=float)
                gma_lines["x"].set_data(x_m, mx)
                gma_lines["y"].set_data(x_m, my)
                gma_lines["z"].set_data(x_m, mz)
                gma_flat.extend(mx.tolist())
                gma_flat.extend(my.tolist())
                gma_flat.extend(mz.tolist())
                ax_gma.set_xlim(0, max(1.0, float(x_m[-1])))
                nbuf = len(buf)
                tail = (
                    f"Latest MA{ma_n} (°/s): gx={mx[-1]:+.3f}  gy={my[-1]:+.3f}  gz={mz[-1]:+.3f}"
                )
                if nbuf < ma_n:
                    text_gma_hint.set_text(
                        f"Warming up: {nbuf}/{ma_n} samples for stable MA{ma_n} (~{(ma_n - nbuf) / 100:.1f}s @100Hz)\n"
                        + tail
                    )
                else:
                    text_gma_hint.set_text(
                        f"MA{ma_n} on ω calib | n={nbuf}/{GYRO_MA_LONG_BUFFER_LEN} (~{nbuf / 100:.1f}s in window)\n"
                        + tail
                    )
            else:
                gma_lines["x"].set_data([], [])
                gma_lines["y"].set_data([], [])
                gma_lines["z"].set_data([], [])
                ax_gma.set_xlim(0, 1)
                text_gma_hint.set_text("No ω calib samples yet (need accel calibration + incoming IMU).")

            self.auto_set_y_limits(ax_acc, acc_vals_flat, include_zero=True, reference=G)
            self.auto_set_y_limits(ax_gyro, gyro_vals_flat, include_zero=True)
            if gma_flat:
                self.auto_set_y_limits(ax_gma, gma_flat, include_zero=True)
            return self.plot_live_artists

        self.plot_ani = animation.FuncAnimation(
            fig, update, init_func=init, interval=50, blit=True, cache_frame_data=False
        )

        def handle_close(_event):
            self.plot_fig = None
            self.plot_ani = None
            self.plot_live_artists = []

        fig.canvas.mpl_connect("close_event", handle_close)
        plt.tight_layout()
        plt.show(block=False)

    def on_close(self):
        if self.plot_fig is not None and plt.fignum_exists(self.plot_fig.number):
            plt.close(self.plot_fig)
        self.disconnect()
        self.root.destroy()


def main():
    explicit_com = len(sys.argv) > 1
    port = sys.argv[1] if explicit_com else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    root = tk.Tk()
    app = IMUCalibApp(root, port, baud, auto_scan_connect=not explicit_com)
    if explicit_com:
        app.log(
            f"Calib: auto-connect {port} @ {baud}. Select MAC then Start 6-Point Calibration."
        )
    else:
        app.log(
            "Calib: scanning COM for IMU Master stream then Connect. "
            "Select MAC, then Start 6-Point Calibration."
        )
    root.mainloop()


if __name__ == "__main__":
    main()
