#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Core live IMU plot: ring buffer numpy + pyqtgraph (dùng chung imu_live_plot.py và imu_logger_gui.py)."""

from __future__ import annotations

import math
import sys
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from imu_serial_codec import DEG_TO_RAD

LIVE_WINDOW_SAMPLES = 200
LIVE_PLOT_INTERVAL_MS = 33
LINE_WIDTH = 1.5
# Trục X cố định cho imu_live_plot.py (Matplotlib blit).
X_FIXED = np.arange(LIVE_WINDOW_SAMPLES, dtype=np.float32)

AXIS_COLORS = {"x": "#1f77b4", "y": "#ff7f0e", "z": "#2ca02c"}
SLAVE_PEN_STYLES = [
    QtCore.Qt.PenStyle.SolidLine,
    QtCore.Qt.PenStyle.DashLine,
    QtCore.Qt.PenStyle.DotLine,
    QtCore.Qt.PenStyle.DashDotLine,
]
NORM_COLOR = "#800080"
_GYRO_RAD = np.float32(DEG_TO_RAD)


def _live_x_axis(window: int) -> tuple[np.ndarray, np.ndarray]:
    x = np.arange(window, dtype=np.float64)
    return x, np.full(window, np.nan, dtype=np.float64)

_pg_configured_mode: str | None = None


def ensure_qt_application() -> QtWidgets.QApplication:
    """Luôn gọi trước khi tạo bất kỳ QWidget/pyqtgraph widget nào."""
    app = QtWidgets.QApplication.instance()
    if app is None:
        # argv rỗng: tránh lỗi khi chạy qua debugpy (-c / launcher args).
        app = QtWidgets.QApplication([])
    return app


def configure_pyqtgraph(*, embedded: bool = False) -> None:
    """Cấu hình pyqtgraph sau QApplication. embedded=True: nhúng Tk (tắt OpenGL)."""
    global _pg_configured_mode
    ensure_qt_application()
    mode = "embed" if embedded else "standalone"
    if _pg_configured_mode == mode:
        return
    pg.setConfigOptions(
        antialias=True,
        background="w",
        foreground="k",
        useOpenGL=not embedded,
    )
    _pg_configured_mode = mode


class MacRing:
    __slots__ = ("accel", "count", "gyro", "head", "norm", "w", "_view_a", "_view_g", "_view_n")

    def __init__(self, window: int) -> None:
        self.w = window
        self.head = 0
        self.count = 0
        self.accel = np.full((3, window), np.nan, dtype=np.float32)
        self.gyro = np.full((3, window), np.nan, dtype=np.float32)
        self.norm = np.full(window, np.nan, dtype=np.float32)
        self._view_a = np.full((3, window), np.nan, dtype=np.float32)
        self._view_g = np.full((3, window), np.nan, dtype=np.float32)
        self._view_n = np.full(window, np.nan, dtype=np.float32)

    def push(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        i = self.head
        self.accel[0, i] = ax
        self.accel[1, i] = ay
        self.accel[2, i] = az
        self.gyro[0, i] = gx * _GYRO_RAD
        self.gyro[1, i] = gy * _GYRO_RAD
        self.gyro[2, i] = gz * _GYRO_RAD
        self.norm[i] = math.sqrt(ax * ax + ay * ay + az * az)
        self.head = (i + 1) % self.w
        if self.count < self.w:
            self.count += 1

    def view(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        c, h, w = self.count, self.head, self.w
        va, vg, vn = self._view_a, self._view_g, self._view_n
        va.fill(np.nan)
        vg.fill(np.nan)
        vn.fill(np.nan)
        if c == 0:
            return va, vg, vn
        if c < w:
            sl = slice(w - c, w)
            va[:, sl] = self.accel[:, :c]
            vg[:, sl] = self.gyro[:, :c]
            vn[sl] = self.norm[:c]
            return va, vg, vn
        idx = (h + np.arange(w, dtype=np.intp)) % w
        va[:] = self.accel[:, idx]
        vg[:] = self.gyro[:, idx]
        vn[:] = self.norm[idx]
        return va, vg, vn


class LiveImuStreamStore:
    def __init__(self, window: int = LIVE_WINDOW_SAMPLES) -> None:
        self._window = window
        self._lock = threading.Lock()
        self._rings: Dict[str, MacRing] = {}
        self.revision = 0

    @property
    def window(self) -> int:
        return self._window

    def reset(self) -> None:
        with self._lock:
            self._rings.clear()
            self.revision = 0

    def resize_window(self, window: int) -> bool:
        """Đổi cửa sổ ring buffer; xóa dữ liệu cũ. Trả về True nếu có thay đổi."""
        w = max(1, int(window))
        with self._lock:
            if w == self._window:
                return False
            self._window = w
            self._rings.clear()
            self.revision += 1
            return True

    def push_many(self, samples: List[Dict[str, Any]]) -> None:
        if not samples:
            return
        with self._lock:
            for s in samples:
                mac = s["mac"]
                ring = self._rings.get(mac)
                if ring is None:
                    ring = MacRing(self._window)
                    self._rings[mac] = ring
                ring.push(s["ax"], s["ay"], s["az"], s["gx"], s["gy"], s["gz"])
            self.revision += 1

    def snapshot_views(
        self,
    ) -> Tuple[int, Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]]]:
        with self._lock:
            views = {mac: ring.view() for mac, ring in self._rings.items()}
            return self.revision, views


@dataclass
class SlaveCurves:
    accel: List[pg.PlotDataItem] = field(default_factory=list)
    gyro: List[pg.PlotDataItem] = field(default_factory=list)
    norm: pg.PlotDataItem | None = None


class LiveImuPyqtGraphPanel:
    """Ba panel accel/gyro/norm; caller gọi refresh() theo timer."""

    def __init__(self, store: LiveImuStreamStore, *, embedded: bool = False) -> None:
        configure_pyqtgraph(embedded=embedded)
        self.store = store
        self.widget = pg.GraphicsLayoutWidget()
        self._mac_order: List[str] = []
        self._curves: Dict[str, SlaveCurves] = {}
        self._window = store.window
        self._x, self._nan_y = _live_x_axis(self._window)
        self._build_plots()

    def _pen(self, color: str, slave_idx: int) -> Any:
        style = SLAVE_PEN_STYLES[slave_idx % len(SLAVE_PEN_STYLES)]
        return pg.mkPen(color, width=LINE_WIDTH, style=style, cosmetic=True)

    def _build_plots(self) -> None:
        self._p_acc = self.widget.addPlot(row=0, col=0, title="Accel (m/s²)")
        self._p_gyro = self.widget.addPlot(row=1, col=0, title="Gyro (rad/s)")
        self._p_norm = self.widget.addPlot(row=2, col=0, title="‖Acc‖ (m/s²)")
        self._p_gyro.setXLink(self._p_acc)
        self._p_norm.setXLink(self._p_acc)

        for p in (self._p_acc, self._p_gyro, self._p_norm):
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setXRange(0, self._window - 1, padding=0)
            p.enableAutoRange(axis="x", enable=False)
            p.enableAutoRange(axis="y", enable=False)

        self._p_acc.setYRange(-80, 80)
        self._p_gyro.setYRange(-40, 40)
        self._p_norm.setYRange(0, 60)
        self._p_norm.addLine(y=9.81, pen=pg.mkPen("#cc3333", style=QtCore.Qt.PenStyle.DashLine))

        self._p_acc.addLegend(offset=(10, 10), labelTextSize="7pt", colCount=2)
        self._p_gyro.addLegend(offset=(10, 10), labelTextSize="7pt", colCount=2)
        self._p_norm.addLegend(offset=(10, 10), labelTextSize="7pt", colCount=1)

    def _ensure_mac(self, mac: str) -> SlaveCurves:
        if mac in self._curves:
            return self._curves[mac]
        idx = len(self._mac_order)
        self._mac_order.append(mac)
        label = f"S{idx + 1}"
        sc = SlaveCurves()
        for axis in ("x", "y", "z"):
            acc_item = self._p_acc.plot(
                self._x,
                self._nan_y,
                pen=self._pen(AXIS_COLORS[axis], idx),
                name=f"{label} a{axis}",
                connect="finite",
            )
            acc_item.setClipToView(True)
            sc.accel.append(acc_item)
            gyro_item = self._p_gyro.plot(
                self._x,
                self._nan_y,
                pen=self._pen(AXIS_COLORS[axis], idx),
                name=f"{label} g{axis}",
                connect="finite",
            )
            gyro_item.setClipToView(True)
            sc.gyro.append(gyro_item)
        sc.norm = self._p_norm.plot(
            self._x,
            self._nan_y,
            pen=self._pen(NORM_COLOR, idx),
            name=f"{label} ‖a‖",
            connect="finite",
        )
        sc.norm.setClipToView(True)
        self._curves[mac] = sc
        return sc

    def _apply_window_to_plots(self) -> None:
        for p in (self._p_acc, self._p_gyro, self._p_norm):
            p.setXRange(0, self._window - 1, padding=0)
        for sc in self._curves.values():
            for item in sc.accel + sc.gyro:
                item.setData(self._x, self._nan_y, connect="finite")
            if sc.norm is not None:
                sc.norm.setData(self._x, self._nan_y, connect="finite")

    def resize_window(self, window: int) -> bool:
        """Đồng bộ trục X/curve với cửa sổ mới (store đã resize trước đó)."""
        w = max(1, int(window))
        if w == self._window:
            return False
        self._window = w
        self._x, self._nan_y = _live_x_axis(w)
        self._apply_window_to_plots()
        return True

    def refresh(self) -> int:
        """Cập nhật curve từ store; trả về số slave có dữ liệu."""
        if self._window != self.store.window:
            self.resize_window(self.store.window)
        _rev, views = self.store.snapshot_views()
        if not views:
            return 0

        seen = [m for m in self._mac_order if m in views]
        for mac in views:
            if mac not in seen:
                seen.append(mac)

        for mac in seen:
            sc = self._ensure_mac(mac)
            va, vg, vn = views[mac]
            for j in range(3):
                sc.accel[j].setData(self._x, va[j], connect="finite")
                sc.gyro[j].setData(self._x, vg[j], connect="finite")
            if sc.norm is not None:
                sc.norm.setData(self._x, vn, connect="finite")
        return len(seen)


def _win32_hwnd(value: int) -> int:
    """Chuẩn hóa WId/HWND cho Win32 API (64-bit an toàn)."""
    return int(value) & 0xFFFFFFFFFFFFFFFF


def _win32_user32():
    import ctypes
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    if getattr(user32, "_imu_embed_typed", False):
        return user32

    user32.SetParent.argtypes = [wintypes.HWND, wintypes.HWND]
    user32.SetParent.restype = wintypes.HWND
    user32.MoveWindow.argtypes = [
        wintypes.HWND,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        wintypes.BOOL,
    ]
    user32.MoveWindow.restype = wintypes.BOOL

    if sys.maxsize > 2**32:
        user32.GetWindowLongPtrW.argtypes = [wintypes.HWND, ctypes.c_int]
        user32.GetWindowLongPtrW.restype = ctypes.c_longlong
        user32.SetWindowLongPtrW.argtypes = [
            wintypes.HWND,
            ctypes.c_int,
            ctypes.c_longlong,
        ]
        user32.SetWindowLongPtrW.restype = ctypes.c_longlong
        user32._imu_get_window_long = user32.GetWindowLongPtrW
        user32._imu_set_window_long = user32.SetWindowLongPtrW
    else:
        user32.GetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int]
        user32.GetWindowLongW.restype = ctypes.c_long
        user32.SetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_long]
        user32.SetWindowLongW.restype = ctypes.c_long
        user32._imu_get_window_long = user32.GetWindowLongW
        user32._imu_set_window_long = user32.SetWindowLongW

    user32._imu_embed_typed = True
    return user32


def _embed_qt_widget_in_tk(qt_widget: QtWidgets.QWidget, tk_hwnd: int) -> None:
    """Gắn native Qt widget vào HWND của tk.Frame (Windows: SetParent + WS_CHILD)."""
    qt_widget.setWindowFlags(QtCore.Qt.WindowType.Widget)
    qt_widget.setAttribute(QtCore.Qt.WidgetAttribute.WA_NativeWindow, True)
    qt_widget.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    qt_hwnd = _win32_hwnd(qt_widget.winId())
    parent_hwnd = _win32_hwnd(tk_hwnd)

    if sys.platform == "win32":
        user32 = _win32_user32()
        gwl_style = -16
        ws_child = 0x40000000
        ws_popup = 0x80000000
        ws_caption = 0x00C00000
        ws_thickframe = 0x00040000
        ws_visible = 0x10000000

        prev = user32.SetParent(qt_hwnd, parent_hwnd)
        if not prev:
            raise OSError("SetParent failed")

        style = int(user32._imu_get_window_long(qt_hwnd, gwl_style))
        style &= ~(ws_popup | ws_caption | ws_thickframe)
        style |= ws_child | ws_visible
        user32._imu_set_window_long(qt_hwnd, gwl_style, style)
    else:
        qt_widget.setParent(parent_hwnd)

    qt_widget.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, False)
    qt_widget.show()


def _move_embedded_qt(qt_widget: QtWidgets.QWidget, w: int, h: int) -> None:
    qt_widget.setGeometry(0, 0, w, h)
    if sys.platform == "win32":
        user32 = _win32_user32()
        user32.MoveWindow(_win32_hwnd(qt_widget.winId()), 0, 0, w, h, True)


class LiveImuTkHost:
    """Nhúng pyqtgraph vào tk.Frame — Windows dùng SetParent (HWND)."""

    def __init__(self, tk_frame: Any) -> None:
        self._tk_frame = tk_frame
        self._qapp = ensure_qt_application()
        self._embedded = False
        self._store = LiveImuStreamStore(LIVE_WINDOW_SAMPLES)

        self._container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self._container)
        layout.setContentsMargins(0, 0, 0, 0)
        self._panel = LiveImuPyqtGraphPanel(self._store, embedded=True)
        layout.addWidget(self._panel.widget)

        tk_frame.bind("<Configure>", self._on_configure, add="+")
        tk_frame.bind("<Map>", self._on_map, add="+")
        tk_frame.bind("<Unmap>", self._on_unmap, add="+")
        tk_frame.after(100, self.ensure_embedded)

    def ensure_embedded(self) -> None:
        if self._embedded:
            return
        self._tk_frame.update_idletasks()
        w = int(self._tk_frame.winfo_width())
        h = int(self._tk_frame.winfo_height())
        if w < 10 or h < 10:
            self._tk_frame.after(100, self.ensure_embedded)
            return
        try:
            _embed_qt_widget_in_tk(self._container, int(self._tk_frame.winfo_id()))
            self._embedded = True
            self._on_configure()
        except Exception:
            self._tk_frame.after(200, self.ensure_embedded)

    def _on_map(self, _event: Any = None) -> None:
        self.ensure_embedded()
        self.show()

    def _on_unmap(self, _event: Any = None) -> None:
        self.hide()

    def _on_configure(self, _event: Any = None) -> None:
        if not self._embedded:
            return
        self._tk_frame.update_idletasks()
        w = max(int(self._tk_frame.winfo_width()), 50)
        h = max(int(self._tk_frame.winfo_height()), 50)
        _move_embedded_qt(self._container, w, h)

    def show(self) -> None:
        if self._embedded:
            self._container.show()
            self._on_configure()

    def hide(self) -> None:
        if self._embedded:
            self._container.hide()

    @property
    def store(self) -> LiveImuStreamStore:
        return self._store

    def resize_window(self, window: int) -> bool:
        if not self._store.resize_window(window):
            return False
        self._panel.resize_window(window)
        return True

    def refresh(self) -> int:
        return self._panel.refresh()

    def process_events(self) -> None:
        self._qapp.processEvents()
