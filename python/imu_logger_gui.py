#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Giao diện cho IMU/VL53 logger: kết nối COM cho Live và (tuỳ chọn) Ghi CSV, log, danh sách GAP (thiếu mẫu) theo MAC,
bảng đếm mẫu thiếu (và thống kê khác) theo từng MAC; sau MAC: micros_timestamp (µs) mẫu mới nhất,
cột Δµs = micros − micros của Node 1 (MAC đầu tiên theo thứ tự sắp, khi có ≥2 node),
rồi % pin (SYNC) và RSSI (SYNC,GUI,… từ master).

Tab "Live IMU": pyqtgraph stream (ring buffer / slave, giống imu_live_plot.py);
kết nối COM (xem Live) tách khỏi ghi CSV. Buffer Live giữ vài trăm mẫu gần nhất / slave (≥5 IMU: 1 s ≈ 100 mẫu; ít hơn: 2 s ≈ 200 mẫu @ 100 Hz) —
spike hiếm có thể không lộ trên Live nhưng vẫn có trong CSV khi Record. Đồ thị chỉ làm mới khi mở tab «Live IMU»
(tab khác: dừng vẽ, tiết kiệm CPU).
Khi «Ghi CSV» bật, vẽ Live dừng (tk after) để ưu tiên ghi file;
«Dừng ghi / Ngắt» trong lúc ghi chỉ đóng file và bật lại Live vẫn giữ cổng; khi chỉ đang Live, cùng nút này để ngắt COM.

Tab "Vẽ CSV": danh sách *.csv trong thư mục ``recorded/`` (cạnh exe khi build), double-click
gọi plot_imu_csv_file trên main thread (Matplotlib/Tk an toàn; double-click dùng nearest nếu chưa chọn dòng); nút "Chọn file…" để mở file bất kỳ;
tuỳ chọn «Đánh dấu thiếu mẫu (gap)»: marker hình dạng khác nhau theo từng chuỗi (imu_log_plotter);
nút "Xóa file đã chọn" (hoặc phím Delete) xóa file khỏi đĩa sau hộp thoại xác nhận.

Logic serial/CSV giống imu_logger.py; khi Record, file được ghi vào ``recorded`` (imus + vl53 cùng cặp trong thư mục đó).
Bảng MAC: «micros (µs)» = micros_timestamp của mẫu IMU mới nhất từ slave;
«Δµs» = chênh micros so với Node 1 (cùng lúc, chỉ khi có ≥2 MAC có micros);
«Thiếu» = tổng số mẫu từng bị bỏ qua (gap); «Đã bù (hết hàng thiếu)» = số mẫu tới hợp lệ
đúng một sample_seq trước đó nằm trong hàng thiếu (kỳ vọng đuổi kịp Thiếu nếu bù hết; gap rất lớn có log riêng).
Mỗi lần bắt đầu ghi file CSV mới: reset Thiếu / Đã bù / Đã nhận (và pending seq), xóa danh sách GAP trên GUI.

Master có thể in dòng `IMU_LOG,LOST_REQ,...` / `IMU_LOG,LOST_RTX,...` / `IMU_LOG,RETX_DROP,...`
(cột Log hiển thị thông tin; timeout/retry do firmware master, không tính trên GUI).

Chạy:
  python imu_logger_gui.py

Debug thời gian (live ~ Hz thực, poll UI, chỗ chậm). Mở trước khi chạy:
  Windows PowerShell: ``$env:IMU_GUI_TIMING_DEBUG=1``
  Linux/macOS: ``IMU_GUI_TIMING_DEBUG=1 python imu_logger_gui.py``

Khi kết nối COM: đo khoảng arrival PC live giữa các gói IMU batch (5 mẫu, ~50 ms ≈ 20 batch/s);
in thống kê khi ngắt COM hoặc dừng ghi CSV.
"""

from __future__ import annotations

import fnmatch
import glob
import os
import queue
import statistics
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from tkinter import filedialog, messagebox, scrolledtext, ttk
import tkinter as tk

import math
import serial
from serial.tools import list_ports

from imu_logger import (
    DEFAULT_BAUD,
    VL53_ZONE_COUNT,
    _SEQ_RESET_THRESHOLD,
    _default_mac_seq_stats,
    _default_vl53_path,
    _norm_mac,
)
from imu_serial_codec import DEG_TO_RAD, feed_imu_serial
from imu_live_plot_core import LIVE_PLOT_INTERVAL_MS, LiveImuTkHost


def _imu_data_line_sort_key(line: str) -> tuple:
    """Sắp xếp dòng dữ liệu IMU: mac → sample_seq → micros (cột 0,1,2)."""
    p = line.rstrip("\n").split(",")
    if len(p) < 3:
        return (_norm_mac(p[0]) if p else "", 0, 0)
    try:
        return (_norm_mac(p[0]), int(p[2]), int(p[1]))
    except ValueError:
        return (_norm_mac(p[0]), 0, 0)


def _app_dir() -> str:
    """Thư mục dữ liệu: cạnh exe khi PyInstaller; cạnh .py khi chạy script."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


APP_DIR = _app_dir()
RECORDED_DIR = os.path.join(APP_DIR, "recorded")


def _ensure_recorded_dir() -> None:
    """Đảm bảo có thư mục chứa log IMU/VL53 đã ghi."""
    os.makedirs(RECORDED_DIR, exist_ok=True)


# Live plot: ring buffer / slave (pyqtgraph — xem imu_live_plot_core.py)
# Slave 100 Hz → 100 mẫu ≈ 1 s; 200 mẫu ≈ 2 s.
LIVE_WINDOW_SAMPLES_DEFAULT = 200
LIVE_WINDOW_SAMPLES_MANY_SLAVES = 100
LIVE_MANY_SLAVES_THRESHOLD = 5
# Windows PowerShell: $env:IMU_GUI_TIMING_DEBUG=1 ; python imu_logger_gui.py
LIVE_TIMING_DEBUG = os.environ.get("IMU_GUI_TIMING_DEBUG", "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
LIVE_TIMING_LOG_INTERVAL_SEC = 1.0
# Gói ESP-NOW IMU_RAWB trên Slave: 5 mẫu/gói (IMU_BATCH_SAMPLES).
IMU_BATCH_SAMPLES = 5
# Slave 100 Hz (IMU_LOOP_PERIOD_US=10000) → ~50 ms giữa hai gói batch liên tiếp.
IMU_SAMPLE_PERIOD_MS = 10.0
PACKET_INTERVAL_NOMINAL_MS = IMU_SAMPLE_PERIOD_MS * IMU_BATCH_SAMPLES
# Lọc outlier: quá ngắn (burst UART / nhầm lô) hoặc quá dài (pause / mất kết nối).
PACKET_INTERVAL_OUTLIER_MIN_MS = PACKET_INTERVAL_NOMINAL_MS * 0.35
PACKET_INTERVAL_OUTLIER_MAX_MS = PACKET_INTERVAL_NOMINAL_MS * 4.0
PACKET_INTERVAL_OUTLIER_IQR_K = 1.5
# Số khoảng thời gian PC giữa các gói batch trước khi in thống kê.
PACKET_INTERVAL_MAX_SAMPLES = 100_000
PACKET_INTERVAL_PROGRESS_EVERY = 10_000
# Live: đọc từng ít byte, xử lý ngay — không gom buffer lớn trước khi decode.
SERIAL_LIVE_READ_MAX_BYTES = 4096
SERIAL_READ_IDLE_BYTES = 256
SERIAL_RX_BUFFER_BYTES = 4 * 1024 * 1024
CSV_FLUSH_INTERVAL_SEC = 1.0
UI_STATS_UPDATE_INTERVAL_SEC = 0.1
TS_DIFF_AVG_WINDOW_SAMPLES = 200
TS_DIFF_PAIR_MAX_ABS_US = 5000
SERIAL_DTR_RUN_STATE = False
SERIAL_RTS_RUN_STATE = False

# Theo dõi từng sample_seq đã được tính vào "Thiếu"; khi mẫu tới trùng seq → "Đã bù" +1.
# Gói quá lớn: vẫn tăng missed (cộng dồn) nhưng không lưu từng seq (tránh hàng trăm nghìn phần tử).
_PENDING_GAP_SEQ_MAX = 65536


def _uint32_forward_gap_seqs(last: int, seq: int) -> list[int]:
    """
    Các sample_seq nằm giữa last và seq (không gồm endpoints), tiến trên uint32.
    Trả về rỗng nếu gap quá lớn (``> _PENDING_GAP_SEQ_MAX``) để tránh bộ pending quá tải.
    """
    delta = (seq - last) & 0xFFFFFFFF
    if delta <= 1 or delta > 0x7FFFFFFF:
        return []
    n = int(delta - 1)
    if n > _PENDING_GAP_SEQ_MAX:
        return []
    return [((last + 1 + i) & 0xFFFFFFFF) for i in range(n)]


def _nearest_delta_us(ts: int, ref_samples: deque[int]) -> Optional[int]:
    """Sai khác ts - ref gần nhất; bỏ nếu lệch quá xa một nửa chu kỳ 10 ms."""
    if not ref_samples:
        return None
    ref = min(ref_samples, key=lambda x: abs(int(ts) - int(x)))
    delta = int(ts) - int(ref)
    if abs(delta) > TS_DIFF_PAIR_MAX_ABS_US:
        return None
    return delta


def _mean_i64(samples: deque[int]) -> Optional[int]:
    if not samples:
        return None
    return int(round(sum(samples) / len(samples)))


def _percentile_sorted(sorted_vals: list[float], p: float) -> float:
    """Phân vị p (0–100) trên mảng đã sort."""
    n = len(sorted_vals)
    if n == 0:
        return float("nan")
    if n == 1:
        return sorted_vals[0]
    k = (n - 1) * p / 100.0
    f = int(math.floor(k))
    c = int(math.ceil(k))
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] * (c - k) + sorted_vals[c] * (k - f)


def _filter_packet_interval_outliers_ms(
    values: list[float],
) -> tuple[list[float], int]:
    """
    Loại outlier: dải vật lý quanh ~50 ms/gói, rồi IQR (1.5×) trên phần còn lại.
    Trả về (mảng đã lọc, số phần tử bị loại).
    """
    n_raw = len(values)
    if n_raw == 0:
        return [], 0

    kept = [
        v
        for v in values
        if PACKET_INTERVAL_OUTLIER_MIN_MS <= v <= PACKET_INTERVAL_OUTLIER_MAX_MS
    ]
    if len(kept) >= 4:
        sorted_k = sorted(kept)
        q1 = _percentile_sorted(sorted_k, 25.0)
        q3 = _percentile_sorted(sorted_k, 75.0)
        iqr = q3 - q1
        if iqr > 0.0:
            lo = q1 - PACKET_INTERVAL_OUTLIER_IQR_K * iqr
            hi = q3 + PACKET_INTERVAL_OUTLIER_IQR_K * iqr
            iqr_kept = [v for v in kept if lo <= v <= hi]
            if len(iqr_kept) >= max(3, len(kept) // 10):
                kept = iqr_kept

    return kept, n_raw - len(kept)


def _compute_packet_interval_stats_ms(
    values: list[float],
    *,
    filter_outliers: bool = True,
) -> dict[str, float | int]:
    """Thống kê khoảng arrival PC (ms) giữa các gói batch IMU."""
    n_raw = len(values)
    if n_raw == 0:
        return {"count": 0, "count_raw": 0, "outliers_removed": 0}

    if filter_outliers:
        filtered, n_removed = _filter_packet_interval_outliers_ms(values)
    else:
        filtered = list(values)
        n_removed = 0

    n = len(filtered)
    if n == 0:
        return {
            "count": 0,
            "count_raw": n_raw,
            "outliers_removed": n_removed,
        }

    mn = min(filtered)
    mx = max(filtered)
    mean_v = statistics.mean(filtered)
    std_v = statistics.pstdev(filtered) if n >= 2 else 0.0
    med_v = statistics.median(filtered)
    sorted_v = sorted(filtered)
    p95 = _percentile_sorted(sorted_v, 95.0)
    p99 = _percentile_sorted(sorted_v, 99.0)
    hz = 1000.0 / mean_v if mean_v > 0.0 else 0.0
    return {
        "count": n,
        "count_raw": n_raw,
        "outliers_removed": n_removed,
        "mean_ms": mean_v,
        "std_ms": std_v,
        "min_ms": mn,
        "max_ms": mx,
        "median_ms": med_v,
        "p95_ms": p95,
        "p99_ms": p99,
        "range_ms": mx - mn,
        "hz_est": hz,
    }


def _format_packet_interval_stats(
    stats: dict[str, float | int],
    *,
    mac: Optional[str] = None,
    label: str = "Inter-batch arrival (PC live)",
) -> str:
    """Build log string for inter-batch interval statistics."""
    if not stats or int(stats.get("count", 0)) == 0:
        n_raw = int(stats.get("count_raw", 0)) if stats else 0
        n_rm = int(stats.get("outliers_removed", 0)) if stats else 0
        if n_raw > 0:
            return (
                f"[PKT-INT] {label}: no samples left after outlier filter "
                f"(raw={n_raw}, removed={n_rm})."
            )
        return f"[PKT-INT] {label}: no samples yet."
    mac_s = f" mac={mac}" if mac else ""
    n_raw = int(stats.get("count_raw", stats["count"]))
    n_rm = int(stats.get("outliers_removed", 0))
    filt_s = f" | raw={n_raw} removed={n_rm}" if n_rm > 0 else ""
    return (
        f"[PKT-INT] {label}{mac_s} | n={stats['count']}{filt_s} | "
        f"mean={stats['mean_ms']:.4f} ms | std={stats['std_ms']:.4f} ms | "
        f"min={stats['min_ms']:.4f} ms | max={stats['max_ms']:.4f} ms | "
        f"median={stats['median_ms']:.4f} ms | p95={stats['p95_ms']:.4f} ms | "
        f"p99={stats['p99_ms']:.4f} ms | range={stats['range_ms']:.4f} ms | "
        f"~{stats['hz_est']:.2f} batch/s"
    )


def _serial_read_live(ser: serial.Serial) -> bytes:
    """
    Đọc serial theo kiểu live: tối đa SERIAL_LIVE_READ_MAX_BYTES byte đang chờ,
    rồi decode/xử lý ngay; không gom hàng trăm KB trước khi xử lý.
    """
    waiting = ser.in_waiting
    if waiting > 0:
        return ser.read(min(waiting, SERIAL_LIVE_READ_MAX_BYTES))
    return ser.read(SERIAL_READ_IDLE_BYTES)


def _feed_imu_batch_pc_interval(
    mac: str,
    sample_seq: int,
    state_by_mac: dict[str, dict[str, Any]],
    intervals_by_mac: dict[str, list[float]],
    *,
    batch_size: int = IMU_BATCH_SAMPLES,
    max_samples: int = PACKET_INTERVAL_MAX_SAMPLES,
) -> bool:
    """
    Ghi khoảng arrival PC live (ms) giữa hai gói batch IMU liên tiếp (mẫu đầu lô).
    Chỉ cập nhật mốc PC khi khoảng hợp lệ (~50 ms) — bỏ qua burst trong cùng vòng xử lý.
    Trả về True nếu MAC này đã thu đủ max_samples khoảng.
    """
    mac_n = _norm_mac(mac)
    seq = int(sample_seq) & 0xFFFFFFFF
    now = time.perf_counter()
    st = state_by_mac.setdefault(mac_n, {"seq0": None, "pc_ts": None})

    seq0 = st["seq0"]
    if seq0 is None:
        if seq % batch_size != 0:
            return False
        st["seq0"] = seq
        st["pc_ts"] = now
        return False

    delta = (seq - seq0) & 0xFFFFFFFF
    if delta == 0:
        return False
    if 0 < delta < batch_size:
        return False

    new_batch = delta == batch_size or (delta > batch_size and seq % batch_size == 0)
    if not new_batch:
        return False

    intervals_ms = intervals_by_mac.setdefault(mac_n, [])
    pc_prev = st.get("pc_ts")
    if pc_prev is not None and len(intervals_ms) < max_samples:
        dt_ms = (now - float(pc_prev)) * 1000.0
        if PACKET_INTERVAL_OUTLIER_MIN_MS <= dt_ms <= PACKET_INTERVAL_OUTLIER_MAX_MS:
            intervals_ms.append(dt_ms)
            st["pc_ts"] = now
    st["seq0"] = seq
    return len(intervals_ms) >= max_samples


def _packet_interval_stats_by_mac(
    intervals_by_mac: dict[str, list[float]],
) -> dict[str, dict[str, dict[str, float | int]]]:
    """Thống kê khoảng batch theo MAC: bản lọc outlier và bản raw (không lọc)."""
    out: dict[str, dict[str, dict[str, float | int]]] = {}
    vals: list[float]
    for mac in sorted(intervals_by_mac.keys()):
        vals = intervals_by_mac[mac]
        out[mac] = {
            "filtered": _compute_packet_interval_stats_ms(vals, filter_outliers=True),
            "unfiltered": _compute_packet_interval_stats_ms(vals, filter_outliers=False),
        }
    return out


def _ui_packet_interval_stats_msg(
    intervals_by_mac: dict[str, list[float]],
    *,
    label_suffix: str = "",
    reset: bool = False,
) -> dict[str, Any]:
    return {
        "type": "packet_interval_stats",
        "by_mac": _packet_interval_stats_by_mac(intervals_by_mac),
        "reset": reset,
        "label_suffix": label_suffix,
    }


def _set_esp32_serial_run_state(ser: serial.Serial) -> None:
    ser.setDTR(SERIAL_DTR_RUN_STATE)
    ser.setRTS(SERIAL_RTS_RUN_STATE)


def _open_serial_for_esp32(port: str, baud: int, timeout: float = 0.05) -> serial.Serial:
    """
    Open port without unintentionally pulsing ESP32-CAM auto-reset.

    USB-UART (CH340, CP2102, …) typically tie DTR/RTS to EN/GPIO0 via
    auto-reset/boot circuitry. With the ESP32-CAM/USB-UART setup used here, a
    steady run state is DTR=False, RTS=False; apply before open and again after open
    to avoid reset when the GUI connects.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.rtscts = False
    ser.dsrdtr = False
    try:
        ser.dtr = SERIAL_DTR_RUN_STATE
        ser.rts = SERIAL_RTS_RUN_STATE
    except (AttributeError, ValueError):
        pass
    ser.open()
    try:
        _set_esp32_serial_run_state(ser)
    except (AttributeError, OSError, serial.SerialException):
        pass
    time.sleep(0.15)
    return ser


def _refresh_ports(combo: ttk.Combobox) -> None:
    ports = [p.device for p in list_ports.comports()]
    combo["values"] = ports
    if ports and combo.get() not in ports:
        combo.set(ports[0])


def _check_sample_seq_gap_ui(
    mac: str,
    seq: int,
    last_by_mac: dict,
    csv_file_line: int,
    stats_by_mac: dict,
    ui_q: queue.Queue,
    pending_missing_by_mac: dict[str, set[int]],
) -> Tuple[bool, bool]:
    """
    (ghi_csv, mẫu_bù_sau_mốc). ``mẫu_bù_sau_mốc`` luôn False trong bản nhẹ.
    Mẫu seq lùi/trễ vẫn được ghi CSV, nhưng không cập nhật mốc last để tránh tạo gap giả.

    Thiếu / Đã bù: mỗi lần phát hiện gap (seq nhảy tới) cộng (delta-1) vào ``missed`` và
    ghi các ``sample_seq`` đó vào pending; mỗi lần một mẫu tới đúng một seq đang pending
    thì ``filled`` +1. Kỳ vọng: sau khi bù đủ, ``filled`` đuổi kịp phần đã ghi vào pending.
    """
    st = stats_by_mac[mac]

    last = last_by_mac.get(mac)
    if last is None:
        last_by_mac[mac] = seq
        return True, False

    pend = pending_missing_by_mac[mac]
    if seq in pend:
        pend.discard(seq)
        st["filled"] = st.get("filled", 0) + 1

    delta = (seq - last) & 0xFFFFFFFF
    if delta == 0:
        st["duplicate"] += 1
        ui_q.put(
            {
                "type": "log",
                "text": (
                    f"[WARN] duplicate sample_seq? csv_line={csv_file_line} "
                    f"mac={mac} seq={seq}"
                ),
            }
        )
    elif delta == 1:
        pass
    elif delta > 0x7FFFFFFF:
        return True, False
    elif delta > _SEQ_RESET_THRESHOLD:
        pend.clear()
        ui_q.put(
            {
                "type": "log",
                "text": (
                    f"[seq reset] csv_line={csv_file_line} mac={mac} large jump (treated as reset): "
                    f"last={last} -> seq={seq}"
                ),
            }
        )
    elif delta > 1:
        missing = int(delta - 1)
        st["missed"] += missing
        gap_seqs = _uint32_forward_gap_seqs(last, seq)
        if gap_seqs:
            pend.update(gap_seqs)
        elif missing > _PENDING_GAP_SEQ_MAX:
            ui_q.put(
                {
                    "type": "log",
                    "text": (
                        f"[GAP] (wide) csv_line={csv_file_line} mac={mac} missing ~{missing} samples; "
                        f"per-seq fill tracking up to {_PENDING_GAP_SEQ_MAX} (no longer 1-1 if larger)"
                    ),
                }
            )
        ui_q.put(
            {
                "type": "gap",
                "mac": mac,
                "missing": missing,
                "last": last,
                "seq": seq,
                "csv_line": csv_file_line,
            }
        )
        ui_q.put(
            {
                "type": "mac_stats",
                "mac": mac,
                "missed": st["missed"],
                "filled": st.get("filled", 0),
                "received": st["received"],
                "duplicate": st["duplicate"],
            }
        )

    last_by_mac[mac] = seq
    return True, False


class ImuLoggerGuiApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("IMU / VL53 Serial Logger & plot CSV")
        self.root.minsize(880, 620)

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        """Tham chiếu cổng COM mở trong worker — đóng từ main khi thoát để giải phóng khóa file CSV."""
        self._worker_serial: Optional[serial.Serial] = None
        self._worker_serial_lock = threading.Lock()
        self._ui_q: queue.Queue = queue.Queue()
        self._mac_rows: dict[str, str] = {}  # mac -> tree iid
        self._last_stats: dict[str, dict[str, int]] = {}
        self._mac_last_micros: dict[str, int] = {}  # mac -> micros_timestamp (µs) mẫu IMU gần nhất
        self._mac_avg_ts_diff_us: dict[str, int] = {}
        self._mac_telem: dict[str, tuple[Optional[int], Optional[int]]] = {}
        self._plot_csv_paths: list[str] = []

        self._recording = threading.Event()
        self._live_host: Optional[LiveImuTkHost] = None
        self._live_plot_window_samples: int = LIVE_WINDOW_SAMPLES_DEFAULT
        self._live_plot_after_id: Any = None
        self._live_plot_suppressed: bool = False
        self._idx_tab_live_imu: int = 0
        self._idx_tab_plot_csv: int = 0
        self._live_imu_tab_selected: bool = False

        # --- IMU_GUI_TIMING_DEBUG: tích lũy rồi in mỗi LIVE_TIMING_LOG_INTERVAL_SEC ---
        self._timing_log_deadline = 0.0
        self._ti_live_wall_prev: Optional[float] = None
        self._ti_live_gap_sum = 0.0
        self._ti_live_gap_n = 0
        self._ti_live_updates = 0
        self._ti_live_sub_sum: Dict[str, float] = {}
        self._ti_live_drawidle_ms = 0.0
        self._ti_live_idletasks_ms = 0.0
        self._ti_poll_rounds = 0
        self._ti_poll_msg_total = 0
        self._ti_poll_msg_by_type: Dict[str, int] = defaultdict(int)
        self._ti_poll_drain_ms = 0.0
        self._ti_poll_flush_stats_ms = 0.0
        self._ti_poll_flush_telem_ms = 0.0
        self._ti_poll_append_ms = 0.0

        self._apply_ttk_button_styles()

        self._nb = ttk.Notebook(self.root)
        self._nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 8))

        tab_serial = ttk.Frame(self._nb, padding=0)
        self._nb.add(tab_serial, text="Serial logger")

        top = ttk.Frame(tab_serial, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="COM:").pack(side=tk.LEFT, padx=(0, 4))
        self._combo_port = ttk.Combobox(top, width=14, state="readonly")
        self._combo_port.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(
            top,
            text="Refresh",
            command=self._on_refresh_ports,
            style="GuiSecondary.TButton",
        ).pack(
            side=tk.LEFT, padx=(0, 12)
        )

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(0, 4))
        self._baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(top, width=10, textvariable=self._baud_var).pack(
            side=tk.LEFT, padx=(0, 12)
        )

        self._btn_open = ttk.Button(
            top,
            text="Connect (Live)",
            command=self._on_open,
            style="GuiPrimary.TButton",
        )
        self._btn_open.pack(side=tk.LEFT, padx=(0, 6))
        self._btn_record = ttk.Button(
            top,
            text="Record CSV",
            command=self._on_start_recording,
            state=tk.DISABLED,
            style="GuiSecondary.TButton",
        )
        self._btn_record.pack(side=tk.LEFT, padx=(0, 6))
        self._btn_close = ttk.Button(
            top,
            text="Stop recording / Disconnect",
            command=self._on_close,
            state=tk.DISABLED,
            style="GuiStop.TButton",
        )
        self._btn_close.pack(side=tk.LEFT)

        mid = ttk.Panedwindow(tab_serial, orient=tk.VERTICAL)
        mid.pack(fill=tk.BOTH, expand=True, padx=0, pady=(0, 8))

        lf_log = ttk.Labelframe(mid, text="Log", padding=4)
        mid.add(lf_log, weight=2)
        self._txt_log = scrolledtext.ScrolledText(
            lf_log, height=14, wrap=tk.WORD, font=("Consolas", 9)
        )
        self._txt_log.pack(fill=tk.BOTH, expand=True)

        bottom = ttk.Panedwindow(mid, orient=tk.HORIZONTAL)
        mid.add(bottom, weight=2)

        lf_gap = ttk.Labelframe(
            bottom, text="GAP — missing-sample lines (by MAC, sample_seq)", padding=4
        )
        bottom.add(lf_gap, weight=1)
        self._txt_gap = scrolledtext.ScrolledText(
            lf_gap, height=10, wrap=tk.NONE, font=("Consolas", 9)
        )
        self._txt_gap.pack(fill=tk.BOTH, expand=True)

        lf_mac = ttk.Labelframe(
            bottom, text="Per MAC — missing samples (and stats)", padding=4
        )
        bottom.add(lf_mac, weight=1)

        cols = (
            "mac",
            "micros_ts",
            "ts_diff",
            "bat_pct",
            "rssi",
            "missed",
            "filled",
            "recv",
        )
        self._tree = ttk.Treeview(
            lf_mac,
            columns=cols,
            show="headings",
            height=10,
            selectmode=tk.BROWSE,
        )
        self._tree.heading("mac", text="MAC", anchor=tk.CENTER)
        self._tree.heading(
            "micros_ts",
            text="micros (µs)",
            anchor=tk.CENTER,
        )
        self._tree.heading(
            "ts_diff",
            text="Δµs avg (vs Node 1)",
            anchor=tk.CENTER,
        )
        self._tree.heading("bat_pct", text="Bat %", anchor=tk.CENTER)
        self._tree.heading("rssi", text="RSSI (dBm)", anchor=tk.CENTER)
        self._tree.heading("missed", text="Missed", anchor=tk.CENTER)
        self._tree.heading("filled", text="Filled", anchor=tk.CENTER)
        self._tree.heading("recv", text="Received", anchor=tk.CENTER)
        self._tree.column("mac", width=130, anchor=tk.CENTER)
        self._tree.column("micros_ts", width=120, anchor=tk.E)
        self._tree.column("ts_diff", width=100, anchor=tk.E)
        self._tree.column("bat_pct", width=64, anchor=tk.CENTER)
        self._tree.column("rssi", width=90, anchor=tk.CENTER)
        self._tree.column("missed", width=90, anchor=tk.CENTER)
        self._tree.column("filled", width=80, anchor=tk.CENTER)
        self._tree.column("recv", width=80, anchor=tk.CENTER)
        sy = ttk.Scrollbar(lf_mac, orient=tk.VERTICAL, command=self._tree.yview)
        self._tree.configure(yscrollcommand=sy.set)
        self._tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sy.pack(side=tk.RIGHT, fill=tk.Y)

        tab_live = ttk.Frame(self._nb, padding=4)
        self._nb.add(tab_live, text="Live IMU")
        self._idx_tab_live_imu = self._nb.index(tab_live)
        self._build_live_tab(tab_live)

        tab_plot = ttk.Frame(self._nb, padding=8)
        self._nb.add(tab_plot, text="Plot CSV")
        self._idx_tab_plot_csv = self._nb.index(tab_plot)

        plot_top = ttk.Frame(tab_plot)
        plot_top.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(
            plot_top,
            text="Log folder: "
            + RECORDED_DIR
            + " (created on first recording)",
            wraplength=760,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, fill=tk.X)
        row2 = ttk.Frame(tab_plot)
        row2.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(
            row2,
            text="Refresh list",
            command=self._on_refresh_csv_list,
            style="GuiSecondary.TButton",
        ).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        self._imu_csv_only_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            row2,
            text="Show only imu_log_*.csv",
            variable=self._imu_csv_only_var,
            command=self._on_refresh_csv_list,
        ).pack(side=tk.LEFT, padx=(0, 10))
        self._plot_show_seq_gaps_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            row2,
            text="Mark missing samples (gap)",
            variable=self._plot_show_seq_gaps_var,
        ).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(
            row2,
            text="Choose file…",
            command=self._on_plot_choose_file,
            style="GuiSecondary.TButton",
        ).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(
            row2,
            text="Delete selected file",
            command=self._on_plot_delete_selected,
            style="GuiStop.TButton",
        ).pack(side=tk.LEFT)
        ttk.Label(
            tab_plot,
            text=(
                "Double-click: open plot. Select a row then \"Delete selected file\" "
                "or press Delete to remove the file (with confirmation).",
            ),
            foreground="gray",
            wraplength=760,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(0, 4))

        lf_list = ttk.Labelframe(tab_plot, text="File CSV", padding=4)
        lf_list.pack(fill=tk.BOTH, expand=True)
        self._list_csv = tk.Listbox(
            lf_list,
            font=("Consolas", 10),
            selectmode=tk.SINGLE,
            activestyle=tk.DOTBOX,
        )
        sb_csv = ttk.Scrollbar(lf_list, orient=tk.VERTICAL, command=self._list_csv.yview)
        self._list_csv.configure(yscrollcommand=sb_csv.set)
        self._list_csv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb_csv.pack(side=tk.RIGHT, fill=tk.Y)
        self._list_csv.bind("<Double-1>", self._on_plot_listbox_double)
        self._list_csv.bind("<Delete>", self._on_plot_delete_selected)

        self._nb.bind("<<NotebookTabChanged>>", self._on_notebook_tab_changed)

        self._on_refresh_ports()
        self._on_refresh_csv_list()
        self.root.protocol("WM_DELETE_WINDOW", self._on_quit)
        self._sync_live_plot_tab_visibility()
        self._poll_ui()

    def _apply_ttk_button_styles(self) -> None:
        """Dùng theme `clam` + màu nền tùy chọn (ttk trên Windows/Linux)."""
        st = ttk.Style()
        if "clam" in st.theme_names():
            st.theme_use("clam")

        pad = (8, 4)
        # Nút phụ: xanh xám
        st.configure("GuiSecondary.TButton", background="#4A6FA5", foreground="#FFFFFF", padding=pad)
        st.map(
            "GuiSecondary.TButton",
            background=[
                ("active", "#3D5D8C"),
                ("pressed", "#2E4A70"),
                ("disabled", "#A8B5CC"),
            ],
            foreground=[("disabled", "#EEF2F8")],
        )
        # Bắt đầu ghi: teal
        st.configure("GuiPrimary.TButton", background="#0D9488", foreground="#FFFFFF", padding=pad)
        st.map(
            "GuiPrimary.TButton",
            background=[
                ("active", "#0F7668"),
                ("pressed", "#115E59"),
                ("disabled", "#94A3AF"),
            ],
            foreground=[("disabled", "#F3F4F6")],
        )
        # Dừng ghi: đỏ dịu
        st.configure("GuiStop.TButton", background="#D64545", foreground="#FFFFFF", padding=pad)
        st.map(
            "GuiStop.TButton",
            background=[
                ("active", "#C43333"),
                ("pressed", "#A62B2B"),
                ("disabled", "#BDBDBD"),
            ],
            foreground=[("disabled", "#5A5A5A")],
        )

    def _live_window_for_slave_count(self, n: int) -> int:
        if n >= LIVE_MANY_SLAVES_THRESHOLD:
            return LIVE_WINDOW_SAMPLES_MANY_SLAVES
        return LIVE_WINDOW_SAMPLES_DEFAULT

    def _maybe_sync_live_window_size(self) -> None:
        """≥5 IMU: buffer 1 s (100 mẫu) để vẽ nhanh hơn; ít hơn: giữ 2 s."""
        if self._live_host is None:
            return
        want = self._live_window_for_slave_count(len(self._mac_rows))
        if want == self._live_plot_window_samples:
            return
        self._live_plot_window_samples = want
        self._live_host.resize_window(want)

    def _reset_live_deque(self) -> None:
        if self._live_host is not None:
            self._live_plot_window_samples = LIVE_WINDOW_SAMPLES_DEFAULT
            self._live_host.resize_window(LIVE_WINDOW_SAMPLES_DEFAULT)
            self._live_host.store.reset()

    def _append_live_imu_samples(self, mac_csv: str, row: Dict[str, Any]) -> None:
        if self._recording.is_set() or self._live_host is None:
            return
        self._live_host.store.push_many(
            [
                {
                    "mac": mac_csv,
                    "ax": float(row["ax"]),
                    "ay": float(row["ay"]),
                    "az": float(row["az"]),
                    "gx": float(row["gx"]),
                    "gy": float(row["gy"]),
                    "gz": float(row["gz"]),
                }
            ]
        )

    def _build_live_tab(self, parent: ttk.Frame) -> None:
        hint = ttk.Label(
            parent,
            text=(
                "The plot updates only while this tab is active (other tabs: data still arrives, "
                "no drawing — saves CPU). With \"Connect (Live)\" only: plots when you open this tab. "
                "With \"Record CSV\" on: Live pauses while recording prioritizes disk. "
                "\"Stop recording / Disconnect\": if recording, closes the file and resumes Live; "
                "if Live only, disconnects COM."
            ),
            wraplength=820,
            justify=tk.LEFT,
        )
        hint.pack(fill=tk.X, pady=(0, 6))

        plot_frame = tk.Frame(parent, bg="white")
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self._live_plot_frame = plot_frame
        self._live_host = LiveImuTkHost(plot_frame)
        parent.winfo_toplevel().after(250, self._live_embed_when_ready)

    def _live_embed_when_ready(self) -> None:
        if self._live_host is None:
            return
        self._live_host.ensure_embedded()
        if self._live_imu_tab_selected and not self._live_plot_suppressed:
            self._live_host.show()

    def _on_notebook_tab_changed(self, event: tk.Event | None = None) -> None:
        self._sync_live_plot_tab_visibility()
        try:
            cur = self._nb.index(self._nb.select())
        except tk.TclError:
            return
        if cur == self._idx_tab_plot_csv:
            self._on_refresh_csv_list()

    def _sync_live_plot_tab_visibility(self) -> None:
        """Chỉ lên lịch vẽ Live khi tab «Live IMU» được chọn; tab khác: hủy after, giữ deque dữ liệu."""
        try:
            cur = self._nb.index(self._nb.select())
        except tk.TclError:
            return
        sel = cur == self._idx_tab_live_imu
        if sel == self._live_imu_tab_selected:
            return
        self._live_imu_tab_selected = sel
        if sel:
            if self._live_host is not None:
                self._live_host.ensure_embedded()
                self._live_host.show()
            if not self._live_plot_suppressed:
                self._schedule_live_plot_tick()
        else:
            if self._live_host is not None:
                self._live_host.hide()
            self._cancel_live_plot_tick()

    def _maybe_print_timing_log(self) -> None:
        """In một lần mỗi LIVE_TIMING_LOG_INTERVAL_SEC nếu IMU_GUI_TIMING_DEBUG bật."""
        if not LIVE_TIMING_DEBUG:
            return
        now = time.monotonic()
        if self._timing_log_deadline == 0.0:
            self._timing_log_deadline = now + LIVE_TIMING_LOG_INTERVAL_SEC
        if now < self._timing_log_deadline:
            return
        self._timing_log_deadline = now + LIVE_TIMING_LOG_INTERVAL_SEC
        win = LIVE_TIMING_LOG_INTERVAL_SEC

        if self._ti_live_gap_n > 0:
            mean_gap_ms = (self._ti_live_gap_sum / self._ti_live_gap_n) * 1000.0
            hz = 1000.0 / mean_gap_ms if mean_gap_ms > 1e-6 else 0.0
            extra = ""
            nu = self._ti_live_updates
            if nu > 0:
                subs = []
                for k in sorted(self._ti_live_sub_sum.keys()):
                    subs.append(f"{k}={self._ti_live_sub_sum[k] / nu:.2f}ms")
                subs.append(f"refresh={self._ti_live_drawidle_ms / nu:.2f}ms")
                subs.append(f"qt_events={self._ti_live_idletasks_ms / nu:.2f}ms")
                extra = " | " + " ".join(subs)
            print(
                f"[TIMING ~{win:.0f}s] live_plot_tick: Δt_mean={mean_gap_ms:.0f}ms (~{hz:.2f}Hz) "
                f"n_ticks={self._ti_live_gap_n} n_draw={nu}{extra}",
                flush=True,
            )

        if self._ti_poll_rounds > 0:
            pr = self._ti_poll_rounds
            mst = self._ti_poll_msg_by_type
            lo = int(mst.get("log", 0))
            print(
                f"[TIMING ~{win:.0f}s] poll_ui: rounds={pr} msgs={self._ti_poll_msg_total} "
                f"(mac_stats={mst.get('mac_stats', 0)} mac_telem={mst.get('mac_telem', 0)} "
                f"log={lo} gap={mst.get('gap', 0)}) | drain_avg={self._ti_poll_drain_ms / pr:.2f}ms "
                f"flush_mac_stats_avg={self._ti_poll_flush_stats_ms / pr:.3f}ms "
                f"flush_telem_avg={self._ti_poll_flush_telem_ms / pr:.3f}ms "
                f"log_gap_append_avg={self._ti_poll_append_ms / pr:.3f}ms",
                flush=True,
            )

        self._ti_live_gap_sum = 0.0
        self._ti_live_gap_n = 0
        self._ti_live_updates = 0
        self._ti_live_sub_sum.clear()
        self._ti_live_drawidle_ms = 0.0
        self._ti_live_idletasks_ms = 0.0
        self._ti_poll_rounds = 0
        self._ti_poll_msg_total = 0
        self._ti_poll_msg_by_type.clear()
        self._ti_poll_drain_ms = 0.0
        self._ti_poll_flush_stats_ms = 0.0
        self._ti_poll_flush_telem_ms = 0.0
        self._ti_poll_append_ms = 0.0

    def _cancel_live_plot_tick(self) -> None:
        tid = self._live_plot_after_id
        if tid is not None:
            try:
                self.root.after_cancel(tid)
            except (tk.TclError, ValueError):
                pass
            self._live_plot_after_id = None

    def _schedule_live_plot_tick(self) -> None:
        if self._live_plot_suppressed or not self._live_imu_tab_selected:
            return
        self._cancel_live_plot_tick()
        self._live_plot_after_id = self.root.after(
            LIVE_PLOT_INTERVAL_MS, self._live_plot_tick
        )

    def _live_plot_tick(self) -> None:
        """Vẽ lại đồ thị Live (pyqtgraph) trên luồng Tk."""
        self._live_plot_after_id = None
        if LIVE_TIMING_DEBUG:
            wall_now = time.monotonic()
            if self._ti_live_wall_prev is not None:
                self._ti_live_gap_sum += wall_now - self._ti_live_wall_prev
                self._ti_live_gap_n += 1
            self._ti_live_wall_prev = wall_now
        try:
            host = self._live_host
            if host is None or self._live_plot_suppressed:
                return
            if not self._live_imu_tab_selected:
                return
            if self._recording.is_set():
                return
            if LIVE_TIMING_DEBUG:
                st: Dict[str, float] = {}
                t0 = time.perf_counter()
                host.refresh()
                st["refresh"] = (time.perf_counter() - t0) * 1000.0
                t0 = time.perf_counter()
                host.process_events()
                st["qt_events"] = (time.perf_counter() - t0) * 1000.0
                self._ti_live_updates += 1
                for k, v in st.items():
                    self._ti_live_sub_sum[k] = self._ti_live_sub_sum.get(k, 0.0) + v
                self._ti_live_drawidle_ms += st["refresh"]
                self._ti_live_idletasks_ms += st["qt_events"]
            else:
                host.refresh()
                host.process_events()
        finally:
            if not self._live_plot_suppressed and self._live_imu_tab_selected:
                self._schedule_live_plot_tick()

    def _pause_live_animation(self) -> None:
        self._live_plot_suppressed = True
        self._cancel_live_plot_tick()

    def _resume_live_animation(self) -> None:
        self._live_plot_suppressed = False
        self._schedule_live_plot_tick()
        if self._live_host is not None:
            self._live_host.refresh()
            self._live_host.process_events()

    def _refresh_serial_toolbar_buttons(self) -> None:
        alive = bool(self._thread and self._thread.is_alive())
        rec = self._recording.is_set()
        self._btn_open.config(state=tk.DISABLED if alive else tk.NORMAL)
        self._btn_record.config(
            state=(tk.NORMAL if (alive and not rec) else tk.DISABLED)
        )
        self._btn_close.config(state=tk.NORMAL if alive else tk.DISABLED)
        self._combo_port.config(state=tk.DISABLED if alive else "readonly")

    def _on_start_recording(self) -> None:
        if not (self._thread and self._thread.is_alive()):
            messagebox.showwarning(
                "Record CSV",
                "Not connected. Click \"Connect (Live)\" first.",
                parent=self.root,
            )
            return
        if self._recording.is_set():
            return
        self._recording.set()
        self._pause_live_animation()
        self._refresh_serial_toolbar_buttons()

    def _append_log(self, s: str) -> None:
        self._txt_log.insert(tk.END, s + "\n")
        self._txt_log.see(tk.END)

    def _append_gap(self, s: str) -> None:
        self._txt_gap.insert(tk.END, s + "\n")
        self._txt_gap.see(tk.END)

    def _default_stats_row(self) -> dict[str, int]:
        return {
            "missed": 0,
            "filled": 0,
            "received": 0,
        }

    def _format_ts_diff_vs_node1(self, mac: str) -> str:
        """
        Trung bình chênh micros so với «Node 1» = MAC nhỏ nhất.
        Worker ghép từng mẫu với timestamp Node 1 gần nhất để tránh so nhầm 2 chu kỳ khác nhau.
        """
        if len(self._mac_last_micros) < 2:
            return "—"
        diff = self._mac_avg_ts_diff_us.get(mac)
        return str(diff) if diff is not None else "—"

    def _tree_row_values_for_mac(self, mac: str) -> Tuple[str, ...]:
        st = self._last_stats.get(mac, self._default_stats_row())
        t_bat, t_rssi = self._mac_telem.get(mac, (None, None))
        bat_s = f"{t_bat}%" if t_bat is not None else "—"
        rssi_s = f"{t_rssi}" if t_rssi is not None and t_rssi > -128 else "—"
        mus = self._mac_last_micros.get(mac)
        micros_s = str(mus) if mus is not None else "—"
        diff_s = self._format_ts_diff_vs_node1(mac)
        return (
            mac,
            micros_s,
            diff_s,
            bat_s,
            rssi_s,
            str(st["missed"]),
            str(st["filled"]),
            str(st["received"]),
        )

    def _redraw_all_mac_rows(self) -> None:
        """Cập nhật mọi dòng (Δµs phụ thuộc Node 1 — cần vẽ lại cả bảng)."""
        for m, iid in self._mac_rows.items():
            self._tree.item(iid, values=self._tree_row_values_for_mac(m))

    def _update_mac_row(
        self,
        mac: str,
        missed: int,
        filled: int,
        received: int,
        *,
        bat_pct: Optional[int] = None,
        rssi_dbm: Optional[int] = None,
        micros_timestamp_us: Optional[int] = None,
        ts_diff_avg_us: Optional[int] = None,
    ) -> None:
        self._last_stats[mac] = {
            "missed": missed,
            "filled": filled,
            "received": received,
        }
        if micros_timestamp_us is not None:
            self._mac_last_micros[mac] = int(micros_timestamp_us)
        if ts_diff_avg_us is not None:
            self._mac_avg_ts_diff_us[mac] = int(ts_diff_avg_us)
        t_bat, t_rssi = self._mac_telem.get(mac, (None, None))
        if bat_pct is not None:
            t_bat = bat_pct
        if rssi_dbm is not None:
            t_rssi = rssi_dbm
        self._mac_telem[mac] = (t_bat, t_rssi)

        if mac not in self._mac_rows:
            iid = self._tree.insert(
                "", tk.END, values=self._tree_row_values_for_mac(mac)
            )
            self._mac_rows[mac] = iid
            self._maybe_sync_live_window_size()
        self._redraw_all_mac_rows()

    def _on_refresh_ports(self) -> None:
        _refresh_ports(self._combo_port)

    def _on_refresh_csv_list(self) -> None:
        self._list_csv.delete(0, tk.END)
        self._plot_csv_paths.clear()
        _ensure_recorded_dir()
        pattern = os.path.join(RECORDED_DIR, "*.csv")
        files = glob.glob(pattern)
        if self._imu_csv_only_var.get():
            files = [
                p
                for p in files
                if fnmatch.fnmatch(os.path.basename(p).lower(), "imu_log_*.csv")
            ]
        files.sort(key=os.path.getmtime, reverse=True)
        for p in files:
            self._plot_csv_paths.append(p)
            try:
                sz = os.path.getsize(p)
                sz_kb = sz / 1024.0
                extra = f"  ({sz_kb:.1f} KB)" if sz_kb < 10240 else f"  ({sz_kb / 1024.0:.1f} MB)"
            except OSError:
                extra = ""
            self._list_csv.insert(tk.END, os.path.basename(p) + extra)

    def _on_plot_listbox_double(self, event: tk.Event | None = None) -> None:
        sel = self._list_csv.curselection()
        if sel:
            idx = int(sel[0])
        elif event is not None:
            idx = int(self._list_csv.nearest(event.y))
        else:
            return
        if idx < 0 or idx >= len(self._plot_csv_paths):
            return
        self._launch_imu_plotter(self._plot_csv_paths[idx])

    def _on_plot_choose_file(self) -> None:
        path = filedialog.askopenfilename(
            parent=self.root,
            title="Choose IMU file (imu_log_*.csv)",
            initialdir=RECORDED_DIR,
            filetypes=[
                ("IMU log", "*.csv"),
                ("All files", "*.*"),
            ],
        )
        if path:
            self._launch_imu_plotter(path)

    def _on_plot_delete_selected(self, event: tk.Event | None = None) -> None:
        """Xóa file tương ứng dòng đang chọn trong list (có xác nhận)."""
        sel = self._list_csv.curselection()
        if not sel:
            messagebox.showinfo(
                "Plot CSV",
                "Select a file in the list before deleting.",
                parent=self.root,
            )
            return
        idx = int(sel[0])
        if idx < 0 or idx >= len(self._plot_csv_paths):
            return
        path = self._plot_csv_paths[idx]
        base = os.path.basename(path)
        if not messagebox.askyesno(
            "Delete file",
            f"Permanently delete this file?\n\n{base}\n\n{path}",
            parent=self.root,
            icon=messagebox.WARNING,
        ):
            return
        try:
            os.remove(path)
        except OSError as e:
            messagebox.showerror(
                "Plot CSV",
                f"Could not delete file:\n{e}",
                parent=self.root,
            )
            return
        self._on_refresh_csv_list()

    def _launch_imu_plotter(self, csv_path: str) -> None:
        csv_path = os.path.normpath(os.path.abspath(csv_path))
        if not os.path.isfile(csv_path):
            messagebox.showerror("Plot CSV", f"File not found:\n{csv_path}")
            return
        base = os.path.basename(csv_path).lower()
        if not base.startswith("imu_log_"):
            messagebox.showwarning(
                "Plot CSV",
                "imu_log_plotter only plots IMU logs (filename imu_log_*.csv).\n"
                "VL53 or other CSV files cannot be used with this plotter.",
            )
            return

        # Matplotlib cần chạy trên main thread; plotter tự chọn Qt5Agg nếu tab Live IMU đã bật Qt.
        def _run_plot_on_main() -> None:
            try:
                from imu_log_plotter import plot_imu_csv_file

                err = plot_imu_csv_file(
                    csv_path, show_seq_gaps=self._plot_show_seq_gaps_var.get()
                )
                if err:
                    messagebox.showerror("Plot CSV", err)
            except Exception as e:
                messagebox.showerror(
                    "Plot CSV",
                    f"Plot error (matplotlib / plotter):\n{e}",
                )

        self.root.after(0, _run_plot_on_main)

    def _on_serial_thread_ended(self) -> None:
        self._recording.clear()
        self._resume_live_animation()
        self._reset_live_deque()
        self._refresh_serial_toolbar_buttons()

    def _parse_baud(self) -> int:
        try:
            b = int(self._baud_var.get().strip())
            if b <= 0:
                raise ValueError
            return b
        except ValueError:
            return DEFAULT_BAUD

    def _on_open(self) -> None:
        port = self._combo_port.get().strip()
        if not port:
            messagebox.showwarning("COM", "Select a COM port.")
            return
        if self._thread and self._thread.is_alive():
            return

        for iid in self._tree.get_children():
            self._tree.delete(iid)
        self._mac_rows.clear()
        self._mac_last_micros.clear()
        self._mac_avg_ts_diff_us.clear()
        self._txt_gap.delete("1.0", tk.END)

        self._stop.clear()
        self._recording.clear()
        self._reset_live_deque()
        self._resume_live_animation()
        self._thread = threading.Thread(
            target=self._serial_worker,
            args=(port, self._parse_baud()),
            daemon=True,
        )
        self._thread.start()
        self._refresh_serial_toolbar_buttons()

    def _on_close(self) -> None:
        if self._recording.is_set():
            self._recording.clear()
            self._resume_live_animation()
            self._append_log(
                "Stopped CSV recording (Live continues — PKT-INT stats print next). "
                "Press \"Stop recording / Disconnect\" again to disconnect COM."
            )
            self._refresh_serial_toolbar_buttons()
            return
        self._stop.set()
        with self._worker_serial_lock:
            ws = self._worker_serial
        if ws is not None:
            try:
                if getattr(ws, "is_open", True):
                    ws.close()
            except Exception:
                pass

    def _on_quit(self) -> None:
        self._live_plot_suppressed = True
        self._cancel_live_plot_tick()
        # Tắt Record trước: worker đóng CSV trong vòng lặp khi thấy rec==False.
        if self._recording.is_set():
            self._recording.clear()
        self._stop.set()
        th = self._thread
        if th is not None and th.is_alive():
            th.join(timeout=25.0)
            if th.is_alive():
                # Đang kẹt ở ser.read() hoặc rewrite CSV — đóng cổng để worker thoát và chạy finally.
                with self._worker_serial_lock:
                    ws = self._worker_serial
                if ws is not None:
                    try:
                        if getattr(ws, "is_open", True):
                            ws.close()
                    except Exception:
                        pass
                th.join(timeout=10.0)
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def _serial_worker(self, port: str, baud: int) -> None:
        imu_header = (
            "mac,micros_timestamp,sample_seq,ax,ay,az,gx,gy,gz,mx_uT,my_uT,mz_uT,temp_c"
        )
        vl53_header = (
            "mac,micros_timestamp,timestamp_ms,sample_seq,"
            + ",".join(f"z{i}" for i in range(VL53_ZONE_COUNT))
        )

        try:
            ser = _open_serial_for_esp32(port, baud, timeout=0.05)
        except serial.SerialException as e:
            self._ui_q.put({"type": "error", "msg": f"Could not open {port}: {e}"})
            self._ui_q.put({"type": "stopped"})
            return

        try:
            ser.set_buffer_size(rx_size=SERIAL_RX_BUFFER_BYTES)
        except (AttributeError, OSError, serial.SerialException):
            pass
        with self._worker_serial_lock:
            self._worker_serial = ser

        connection_start = datetime.now()
        self._ui_q.put(
            {
                "type": "log",
                "text": (
                    f"Opened {port} @ {baud} | DTR=False, RTS=False | Live on. "
                    f"Press \"Record CSV\" to save IMU/VL53 (Live pauses while recording). "
                    f"Live serial (≤{SERIAL_LIVE_READ_MAX_BYTES} B/loop) + batch arrival timing "
                    f"(PC, ~50 ms) — stats printed when COM disconnects."
                ),
            }
        )

        buf = bytearray()
        f_imu = None
        f_vl53 = None
        out_path = ""
        vl53_path = ""
        count_imu = 0
        count_vl53 = 0
        total_imu_logged = 0
        total_vl53_logged = 0
        last_seq_by_mac: dict = {}
        pending_missing_by_mac: dict[str, set[int]] = defaultdict(set)
        stats_by_mac: dict = defaultdict(_default_mac_seq_stats)
        last_ui_stats_emit = 0.0
        mac_last_micros_worker: dict[str, int] = {}
        recent_micros_by_mac: dict[str, deque[int]] = defaultdict(
            lambda: deque(maxlen=TS_DIFF_AVG_WINDOW_SAMPLES)
        )
        ts_diff_hist_by_mac: dict[str, deque[int]] = defaultdict(
            lambda: deque(maxlen=TS_DIFF_AVG_WINDOW_SAMPLES)
        )
        current_ref_mac: Optional[str] = None
        last_csv_flush = time.monotonic()
        pkt_interval_state: dict[str, dict[str, Any]] = {}
        pkt_intervals_by_mac: dict[str, list[float]] = {}
        pkt_interval_last_progress: dict[str, int] = defaultdict(int)

        try:
            while not self._stop.is_set():
                rec = self._recording.is_set()

                if rec and f_imu is None:
                    out_name = f"imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                    _ensure_recorded_dir()
                    out_path = os.path.join(RECORDED_DIR, out_name)
                    vl53_path = _default_vl53_path(out_path)
                    try:
                        f_imu = open(
                            out_path, "w", encoding="utf-8", buffering=1024 * 1024
                        )
                        f_vl53 = open(
                            vl53_path, "w", encoding="utf-8", buffering=1024 * 1024
                        )
                        f_imu.write(imu_header + "\n")
                        f_vl53.write(vl53_header + "\n")
                        count_imu = 0
                        count_vl53 = 0
                        last_csv_flush = time.monotonic()
                        last_seq_by_mac.clear()
                        pending_missing_by_mac.clear()
                        stats_by_mac.clear()
                        last_ui_stats_emit = 0.0
                        mac_last_micros_worker.clear()
                        recent_micros_by_mac.clear()
                        ts_diff_hist_by_mac.clear()
                        current_ref_mac = None
                        self._ui_q.put({"type": "recording_session_reset"})
                        self._ui_q.put(
                            {
                                "type": "log",
                                "text": (
                                    f"Started recording | IMU: {out_path} | VL53: {vl53_path}"
                                ),
                            }
                        )
                    except OSError as e:
                        self._ui_q.put(
                            {
                                "type": "error",
                                "msg": f"Could not create/write CSV: {e}",
                            }
                        )
                        self._recording.clear()
                        self._ui_q.put({"type": "recording_aborted"})
                        self._ui_q.put(
                            {
                                "type": "log",
                                "text": "Recording turned off after file error — Live active again.",
                            }
                        )
                        continue

                if (not rec) and f_imu is not None:
                    imu_n = count_imu
                    vl53_n = count_vl53
                    try:
                        f_imu.flush()
                        f_imu.close()
                    except Exception:
                        pass
                    try:
                        f_vl53.flush()
                        f_vl53.close()
                    except Exception:
                        pass
                    f_imu = None
                    f_vl53 = None
                    count_imu = 0
                    count_vl53 = 0
                    self._ui_q.put(
                        {
                            "type": "log",
                            "text": (
                                f"Closed CSV files (last session: IMU={imu_n} rows, "
                                f"VL53={vl53_n} rows)."
                            ),
                        }
                    )
                    self._ui_q.put(
                        _ui_packet_interval_stats_msg(
                            pkt_intervals_by_mac,
                            label_suffix=" (after stopping CSV recording)",
                        )
                    )

                chunk = _serial_read_live(ser)
                if not chunk:
                    continue

                rows, buf = feed_imu_serial(buf, chunk)
                imu_write_lines: list[str] = []
                vl53_write_lines: list[str] = []
                macs_stats_dirty: set[str] = set()
                for row in rows:
                    kind = row.get("kind", "imu")
                    if kind == "sync_gui":
                        m = _norm_mac(str(row.get("mac", "")))
                        if m:
                            self._ui_q.put(
                                {
                                    "type": "mac_telem",
                                    "mac": m,
                                    "bat_pct": int(row["bat_pct"]),
                                    "rssi_dbm": int(row["rssi_dbm"]),
                                }
                            )
                        continue
                    if kind == "imu_ctrl":
                        cmd = str(row.get("cmd", ""))
                        if cmd == "LOST_REQ":
                            m = _norm_mac(row["mac"])
                            lo = int(row["seq_first"])
                            c = int(row["seq_count"])
                            hi = lo + c - 1 if c > 0 else lo
                            self._ui_q.put(
                                {
                                    "type": "log",
                                    "text": (
                                        f"[RETX] Master requests retransmit: mac={m} "
                                        f"seq {lo}..{hi} (n={c})"
                                    ),
                                }
                            )
                        elif cmd == "LOST_RTX":
                            m = _norm_mac(row["mac"])
                            self._ui_q.put(
                                {
                                    "type": "log",
                                    "text": (
                                        f"[RETX] Master sent retransmit batch on UART "
                                        f"(from IMU_RETXB): mac={m} "
                                        f"node={row['node_id']} count={row['count']} "
                                        f"seq0={row['seq0']}"
                                    ),
                                }
                            )
                        elif cmd == "RETX_DROP":
                            m = _norm_mac(row["mac"])
                            self._ui_q.put(
                                {
                                    "type": "log",
                                    "text": (
                                        f"[RETX] Master dropped pending after 3× LOST: mac={m} "
                                        f"node={row['node_id']} "
                                        f"seq0={row['seq0']} n={row['n']}"
                                    ),
                                }
                            )
                        continue
                    if kind == "vl53":
                        if (not self._recording.is_set()) or f_vl53 is None:
                            continue
                        mac_csv = _norm_mac(row["mac"])
                        zones = row["zones"]
                        z_csv = ",".join(str(z) for z in zones)
                        line_csv = (
                            f"{mac_csv},{row['micros']},{row['timestamp_ms']},"
                            f"{row['sample_seq']},{z_csv}\n"
                        )
                        vl53_write_lines.append(line_csv)
                        count_vl53 += 1
                        total_vl53_logged += 1
                        if count_vl53 % 5000 == 0:
                            self._ui_q.put(
                                {
                                    "type": "log",
                                    "text": (
                                        f"VL53: wrote {count_vl53} rows → "
                                        f"{os.path.basename(vl53_path)}"
                                    ),
                                }
                            )
                        continue

                    gx_rad = row["gx"] * DEG_TO_RAD
                    gy_rad = row["gy"] * DEG_TO_RAD
                    gz_rad = row["gz"] * DEG_TO_RAD
                    mac_csv = _norm_mac(row["mac"])
                    rec_row = self._recording.is_set() and f_imu is not None
                    csv_file_line = (
                        count_imu + 2
                        if rec_row
                        else (stats_by_mac[mac_csv]["received"] + 2)
                    )
                    seq = row.get("sample_seq")
                    if seq is None:
                        seq_i = 0
                        write_row = True
                    else:
                        try:
                            seq_i = int(seq)
                        except (TypeError, ValueError):
                            seq_i = 0
                            write_row = True
                        else:
                            write_row, _ = _check_sample_seq_gap_ui(
                                mac_csv,
                                seq_i,
                                last_seq_by_mac,
                                csv_file_line,
                                stats_by_mac,
                                self._ui_q,
                                pending_missing_by_mac,
                            )

                    if not write_row:
                        s = stats_by_mac[mac_csv]
                        macs_stats_dirty.add(mac_csv)
                        continue

                    tc = row.get("temp_c")
                    temp_out = float(tc) if tc is not None else 0.0
                    mx_u = row.get("mx_uT")
                    my_u = row.get("my_uT")
                    mz_u = row.get("mz_uT")
                    mx_out = float(mx_u) if mx_u is not None else 0.0
                    my_out = float(my_u) if my_u is not None else 0.0
                    mz_out = float(mz_u) if mz_u is not None else 0.0
                    line_csv = (
                        f"{mac_csv},{row['micros']},{seq_i},"
                        f"{row['ax']},{row['ay']},{row['az']},"
                        f"{gx_rad},{gy_rad},{gz_rad},"
                        f"{mx_out},{my_out},{mz_out},{temp_out}\n"
                    )

                    if rec_row:
                        # Không rewrite/sort cả file khi retx tới trễ: thao tác đó chặn
                        # thread đọc serial và dễ làm tràn RX buffer ở tốc độ cao.
                        imu_write_lines.append(line_csv)
                        count_imu += 1
                        total_imu_logged += 1
                        if count_imu % 5000 == 0:
                            self._ui_q.put(
                                {
                                    "type": "log",
                                    "text": (
                                        f"IMU: wrote {count_imu} rows → "
                                        f"{os.path.basename(out_path)}"
                                    ),
                                }
                            )
                    else:
                        self._append_live_imu_samples(mac_csv, row)

                    micros_i = int(row["micros"])
                    mac_last_micros_worker[mac_csv] = micros_i
                    if not rec:
                        recent_micros_by_mac[mac_csv].append(micros_i)
                        ref_mac = min(recent_micros_by_mac.keys())
                        if ref_mac != current_ref_mac:
                            current_ref_mac = ref_mac
                            ts_diff_hist_by_mac.clear()
                        if mac_csv == ref_mac:
                            ts_diff_hist_by_mac[mac_csv].append(0)
                            for other_mac, other_samples in recent_micros_by_mac.items():
                                if other_mac == ref_mac:
                                    continue
                                delta = _nearest_delta_us(micros_i, other_samples)
                                if delta is not None:
                                    ts_diff_hist_by_mac[other_mac].append(-delta)
                        else:
                            delta = _nearest_delta_us(
                                micros_i, recent_micros_by_mac[ref_mac]
                            )
                            if delta is not None:
                                ts_diff_hist_by_mac[mac_csv].append(delta)

                    stats_by_mac[mac_csv]["received"] += 1
                    macs_stats_dirty.add(mac_csv)
                    if seq is not None:
                        try:
                            seq_pkt = int(seq)
                        except (TypeError, ValueError):
                            pass
                        else:
                            mac_intervals = pkt_intervals_by_mac.setdefault(mac_csv, [])
                            n_before = len(mac_intervals)
                            full = _feed_imu_batch_pc_interval(
                                mac_csv,
                                seq_pkt,
                                pkt_interval_state,
                                pkt_intervals_by_mac,
                            )
                            n_mac = len(mac_intervals)
                            if n_mac > n_before:
                                last_prog = pkt_interval_last_progress[mac_csv]
                                if (
                                    n_mac >= last_prog + PACKET_INTERVAL_PROGRESS_EVERY
                                    and n_mac < PACKET_INTERVAL_MAX_SAMPLES
                                ):
                                    pkt_interval_last_progress[mac_csv] = (
                                        n_mac // PACKET_INTERVAL_PROGRESS_EVERY
                                    ) * PACKET_INTERVAL_PROGRESS_EVERY
                                    self._ui_q.put(
                                        {
                                            "type": "log",
                                            "text": (
                                                f"[PKT-INT] mac={mac_csv} "
                                                f"captured {n_mac}/"
                                                f"{PACKET_INTERVAL_MAX_SAMPLES} intervals "
                                                f"(seq={seq_pkt})"
                                            ),
                                        }
                                    )
                            if full:
                                self._ui_q.put(
                                    _ui_packet_interval_stats_msg(
                                        pkt_intervals_by_mac,
                                        reset=True,
                                        label_suffix=(
                                            f" ({PACKET_INTERVAL_MAX_SAMPLES} intervals/MAC)"
                                        ),
                                    )
                                )
                                pkt_intervals_by_mac.clear()
                                pkt_interval_state.clear()
                                pkt_interval_last_progress.clear()
                now_ui = time.monotonic()
                if macs_stats_dirty and (
                    rec
                    or now_ui - last_ui_stats_emit >= UI_STATS_UPDATE_INTERVAL_SEC
                ):
                    last_ui_stats_emit = now_ui
                    for mac_csv in sorted(macs_stats_dirty):
                        s = stats_by_mac[mac_csv]
                        msg: dict[str, Any] = {
                            "type": "mac_stats",
                            "mac": mac_csv,
                            "missed": s["missed"],
                            "filled": s.get("filled", 0),
                            "received": s["received"],
                            "micros": mac_last_micros_worker.get(mac_csv),
                        }
                        if not rec:
                            msg["ts_diff_avg"] = _mean_i64(
                                ts_diff_hist_by_mac[mac_csv]
                            )
                        self._ui_q.put(msg)

                if imu_write_lines and f_imu is not None:
                    f_imu.writelines(imu_write_lines)
                if vl53_write_lines and f_vl53 is not None:
                    f_vl53.writelines(vl53_write_lines)
                if (f_imu is not None or f_vl53 is not None) and (
                    time.monotonic() - last_csv_flush >= CSV_FLUSH_INTERVAL_SEC
                ):
                    if f_imu is not None:
                        f_imu.flush()
                    if f_vl53 is not None:
                        f_vl53.flush()
                    last_csv_flush = time.monotonic()
        except Exception as e:
            self._ui_q.put({"type": "error", "msg": f"Serial/CSV thread error: {e}"})
        finally:
            if f_imu is not None:
                try:
                    f_imu.flush()
                    f_imu.close()
                except Exception:
                    pass
            if f_vl53 is not None:
                try:
                    f_vl53.flush()
                    f_vl53.close()
                except Exception:
                    pass
            try:
                ser.close()
            except Exception:
                pass
            with self._worker_serial_lock:
                self._worker_serial = None

            dur = (datetime.now() - connection_start).total_seconds()
            self._ui_q.put(
                _ui_packet_interval_stats_msg(
                    pkt_intervals_by_mac,
                    label_suffix=f" (session {dur:.1f} s)",
                )
            )
            self._ui_q.put(
                {
                    "type": "log",
                    "text": (
                        f"Disconnected COM (session {dur:.1f} s) | "
                        f"Total rows written: IMU={total_imu_logged}, VL53={total_vl53_logged}"
                    ),
                }
            )
            self._ui_q.put({"type": "stopped"})

    def _poll_ui(self) -> None:
        pending_stats: dict[str, Dict[str, Any]] = {}
        pending_telem: dict[str, Dict[str, Any]] = {}

        dbg = LIVE_TIMING_DEBUG
        if dbg:
            t_poll0 = time.perf_counter()
            n_msgs = 0
            counts: dict[str, int] = defaultdict(int)

        try:
            while True:
                msg = self._ui_q.get_nowait()
                if dbg:
                    n_msgs += 1
                    counts[str(msg.get("type", "?"))] += 1
                t = msg.get("type")
                if t == "log":
                    if dbg:
                        ta = time.perf_counter()
                    self._append_log(msg["text"])
                    if dbg:
                        self._ti_poll_append_ms += (time.perf_counter() - ta) * 1000.0
                elif t == "gap":
                    if dbg:
                        ta = time.perf_counter()
                    ts = datetime.now().strftime("%H:%M:%S")
                    line = (
                        f"[{ts}] [GAP] csv_line={msg['csv_line']} mac={msg['mac']} "
                        f"missing ~{msg['missing']} samples (seq {msg['last']} -> {msg['seq']})"
                    )
                    self._append_gap(line)
                    if dbg:
                        self._ti_poll_append_ms += (time.perf_counter() - ta) * 1000.0
                elif t == "recording_session_reset":
                    self._last_stats.clear()
                    self._mac_avg_ts_diff_us.clear()
                    self._txt_gap.delete("1.0", tk.END)
                    self._redraw_all_mac_rows()
                elif t == "mac_stats":
                    pending_stats[str(msg["mac"])] = msg
                elif t == "mac_telem":
                    pending_telem[str(msg["mac"])] = msg
                elif t == "packet_interval_stats":
                    by_mac = msg.get("by_mac") or {}
                    suffix = str(msg.get("label_suffix", ""))
                    if not by_mac:
                        self._append_log(
                            f"[PKT-INT] No batch intervals collected{suffix}."
                        )
                    else:
                        for mac in sorted(by_mac.keys()):
                            entry = by_mac[mac]
                            if isinstance(entry, dict) and "filtered" in entry:
                                pairs = (
                                    ("filtered", "outlier-filtered"),
                                    ("unfiltered", "raw"),
                                )
                                for key, tag in pairs:
                                    self._append_log(
                                        _format_packet_interval_stats(
                                            entry[key],
                                            mac=mac,
                                            label=f"Inter-batch interval ({tag})"
                                            + suffix,
                                        )
                                    )
                            else:
                                self._append_log(
                                    _format_packet_interval_stats(
                                        entry,
                                        mac=mac,
                                        label="Inter-batch interval" + suffix,
                                    )
                                )
                    if msg.get("reset"):
                        self._append_log(
                            "[PKT-INT] Printed stats — buffer cleared, capturing again."
                        )
                elif t == "recording_aborted":
                    self._resume_live_animation()
                    self._refresh_serial_toolbar_buttons()
                elif t == "error":
                    messagebox.showerror("Error", msg["msg"])
                    if dbg:
                        ta = time.perf_counter()
                    self._append_log(msg["msg"])
                    if dbg:
                        self._ti_poll_append_ms += (time.perf_counter() - ta) * 1000.0
                elif t == "stopped":
                    self._on_serial_thread_ended()
                    self._on_refresh_csv_list()
        except queue.Empty:
            pass

        if dbg:
            self._ti_poll_drain_ms += (time.perf_counter() - t_poll0) * 1000.0
            self._ti_poll_msg_total += n_msgs
            self._ti_poll_rounds += 1
            for kk, vv in counts.items():
                self._ti_poll_msg_by_type[kk] += vv

        if dbg:
            tf0 = time.perf_counter()
        for mac in sorted(pending_stats.keys()):
            m = pending_stats[mac]
            mus = m.get("micros")
            self._update_mac_row(
                mac,
                int(m["missed"]),
                int(m.get("filled", 0)),
                int(m["received"]),
                micros_timestamp_us=int(mus) if mus is not None else None,
                ts_diff_avg_us=(
                    int(m["ts_diff_avg"]) if m.get("ts_diff_avg") is not None else None
                ),
            )
        if dbg:
            self._ti_poll_flush_stats_ms += (time.perf_counter() - tf0) * 1000.0
            tt0 = time.perf_counter()
        for mac in sorted(pending_telem.keys()):
            m = pending_telem[mac]
            z = self._last_stats.get(mac, self._default_stats_row())
            self._update_mac_row(
                mac,
                z["missed"],
                z["filled"],
                z["received"],
                bat_pct=m.get("bat_pct"),
                rssi_dbm=m.get("rssi_dbm"),
            )
        if dbg:
            self._ti_poll_flush_telem_ms += (time.perf_counter() - tt0) * 1000.0
            self._maybe_print_timing_log()

        self.root.after(80, self._poll_ui)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    ImuLoggerGuiApp().run()


if __name__ == "__main__":
    main()
