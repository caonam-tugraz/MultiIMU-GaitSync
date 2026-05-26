#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read logged IMU CSV and plot accel, gyro (rad/s), ‖acc‖.

CSV (imu_logger.py):
  - mac,micros_timestamp,sample_seq,ax,ay,az,gx,gy,gz[,mx_uT,my_uT,mz_uT][,temp_c]
  - Optional magnetometer columns (µT); when present the plot adds an mx/my/mz panel.
  - Optional leading type column: type,mac,... (only IMU rows; VL53 skipped)
  - Multiple interleaved slaves; group by MAC, sort by micros_timestamp.
  - Before plotting: drop exact duplicate rows (MAC + micros + seq + IMU values), then
    duplicate (mac, sample_seq): keep first row.
  - Optional (GUI / plot_imu_csv_file(show_seq_gaps=True)): missing seq (gap between samples)
    shown as hollow edge-only markers per series (ax/ay/az/…); y linearly interpolated.

X-axis: sample_seq; each MAC subtracts min(sample_seq) → starts at 0 (per-node offset).
  Sort by (sample_seq, micros). Heavy datasets: subsample for display only.

Run:
  python imu_log_plotter.py [file.csv]
  (Requires imu_log_*.csv — not vl53_log_*.csv; the script warns if wrong format.)

No file argument: CSV file dialog; Cancel → newest imu_log_*.csv in the script folder.
"""

import sys
import os
import glob
import csv
import math
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

# Select matplotlib backend before pyplot; TkAgg conflicts if imu_logger_gui live plot (Qt) is open.
import matplotlib


def _qt_application_running() -> bool:
    probes = (
        lambda: __import__(
            "pyqtgraph.Qt", fromlist=["QtWidgets"]
        ).QtWidgets.QApplication.instance(),
        lambda: __import__(
            "PyQt5.QtWidgets", fromlist=["QApplication"]
        ).QApplication.instance(),
        lambda: __import__(
            "PySide6.QtWidgets", fromlist=["QApplication"]
        ).QApplication.instance(),
        lambda: __import__(
            "PyQt6.QtWidgets", fromlist=["QApplication"]
        ).QApplication.instance(),
    )
    for probe in probes:
        try:
            if probe() is not None:
                return True
        except Exception:
            continue
    return False


def _configure_matplotlib_backend() -> None:
    if _qt_application_running():
        for backend in ("Qt5Agg", "QtAgg"):
            try:
                matplotlib.use(backend, force=True)
                return
            except Exception:
                continue
        raise ImportError(
            "Qt is running (Live IMU tab) but matplotlib could not load Qt5Agg/QtAgg."
        )
    matplotlib.use("TkAgg")


_configure_matplotlib_backend()
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog

from imu_logger import _SEQ_RESET_THRESHOLD

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
AXIS_COLORS = {"x": "#e74c3c", "y": "#27ae60", "z": "#3498db"}
SLAVE_LINESTYLES = ["-", "--", ":", "-."]
# Max displayed points per series (display only — avoids overcrowded traces)
PLOT_MAX_POINTS_PER_SERIES = 5000
# Limit gap markers per MAC (avoid heavy scatter)
PLOT_MAX_GAP_MARKERS_PER_MAC = 4000

# Distinct marker shape per series (color matches line).
_GAP_MARKER_STYLE: Dict[str, Tuple[str, float]] = {
    "ax": ("o", 5.0),
    "ay": ("s", 5.0),
    "az": ("^", 6.0),
    "gx": ("D", 5.0),
    "gy": ("v", 6.0),
    "gz": ("P", 6.0),
    "norm": ("X", 6.5),
    "mx_uT": ("*", 7.0),
    "my_uT": ("h", 6.0),
    "mz_uT": ("H", 5.5),
    "temp_c": ("8", 5.5),
}

_IMU_REQUIRED_9 = (
    "mac",
    "micros_timestamp",
    "sample_seq",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
)


def _imu_csv_header_ok(names):
    """Return (ok, has_type_col): first nine IMU columns required; optional mag and temp_c."""
    n = [h.strip() for h in names if h]
    if not n:
        return False, False
    has_type = n[0].lower() == "type"
    rest = n[1:] if has_type else n
    if len(rest) < 9:
        return False, has_type
    if tuple(rest[:9]) != _IMU_REQUIRED_9:
        return False, has_type
    return True, has_type


def _fieldname_map(fieldnames):
    """lower -> original CSV header name for DictReader keys."""
    return {str(h).strip().lower(): h for h in (fieldnames or []) if h}


def _mag_keys_from_fieldmap(fmap: dict) -> Optional[Tuple[str, str, str]]:
    """Return (key_mx, key_my, key_mz) if CSV has three mag columns."""
    if all(k in fmap for k in ("mx_ut", "my_ut", "mz_ut")):
        return fmap["mx_ut"], fmap["my_ut"], fmap["mz_ut"]
    if all(k in fmap for k in ("mx", "my", "mz")):
        return fmap["mx"], fmap["my"], fmap["mz"]
    return None


def _parse_micros_timestamp_cell(s):
    """Parse micros: prefer full int unless string has decimal point."""
    s = str(s).strip()
    if not s:
        raise ValueError("empty")
    try:
        return int(s)
    except ValueError:
        return int(float(s))


def _parse_sample_seq_cell(s):
    """sample_seq as int; Excel may export as float."""
    s = str(s).strip()
    if not s:
        raise ValueError("empty")
    try:
        return int(s)
    except ValueError:
        return int(float(s))


def _norm_mac(mac: str) -> str:
    return str(mac).strip().upper()


def _looks_like_vl53_log_header(names):
    """Detect vl53_log_*.csv: has timestamp_ms, z0 — no ax..gz."""
    n = [h.strip().lower() for h in names if h]
    if not n:
        return False
    return "timestamp_ms" in n and "z0" in n and "ax" not in n


def find_latest_log():
    pattern = os.path.join(SCRIPT_DIR, "imu_log_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def ask_csv_path():
    """File open dialog; empty return → caller uses find_latest_log()."""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        path = filedialog.askopenfilename(
            title="Select IMU log (imu_log_*.csv) — not vl53_log_*.csv",
            initialdir=SCRIPT_DIR,
            filetypes=[
                ("IMU log", "imu_log_*.csv"),
                ("CSV", "*.csv"),
                ("All files", "*.*"),
            ],
        )
    finally:
        root.destroy()
    return path if path else None


def _short_mac_label(mac: str) -> str:
    if len(mac) <= 20:
        return mac
    return mac[:8] + "…" + mac[-4:]


def _subsample_indices(n: int, max_n: int) -> list[int]:
    if n <= 0 or max_n <= 0:
        return []
    if n <= max_n or max_n < 2:
        return list(range(n))
    ix = np.unique(np.rint(np.linspace(0, n - 1, max_n)).astype(np.intp))
    return [int(i) for i in ix.tolist()]


def _gather_gap_points_per_series(
    sub: List[dict], s0: int
) -> Dict[str, Tuple[List[float], List[float]]]:
    """
    Detect missing sample_seq (uint32 increment, delta > 1, not wrap — matches imu_logger).
    Each missing seq: x = seq - s0; y linearly interpolated between neighbors.
    """
    seqs = [int(r["sample_seq"]) for r in sub]
    if len(seqs) < 2:
        return {}

    gap_specs: List[Tuple[float, int, int, float]] = []
    for i in range(len(seqs) - 1):
        a, b = seqs[i], seqs[i + 1]
        delta = (b - a) & 0xFFFFFFFF
        if delta <= 1 or delta > 0x7FFFFFFF or delta > _SEQ_RESET_THRESHOLD:
            continue
        for k in range(1, int(delta)):
            if len(gap_specs) >= PLOT_MAX_GAP_MARKERS_PER_MAC:
                break
            miss = (a + k) & 0xFFFFFFFF
            xf = float(miss - s0)
            t = float(k) / float(delta)
            gap_specs.append((xf, i, i + 1, t))
        if len(gap_specs) >= PLOT_MAX_GAP_MARKERS_PER_MAC:
            break

    if not gap_specs:
        return {}

    out: Dict[str, Tuple[List[float], List[float]]] = {}
    for key in ("ax", "ay", "az", "gx", "gy", "gz"):
        xs = [g[0] for g in gap_specs]
        ys = [
            float(sub[il][key]) * (1.0 - tt) + float(sub[ir][key]) * tt
            for _, il, ir, tt in gap_specs
        ]
        out[key] = (xs, ys)

    xs_n = [g[0] for g in gap_specs]
    yn: List[float] = []
    for _, il, ir, t in gap_specs:
        ax_ = float(sub[il]["ax"]) * (1.0 - t) + float(sub[ir]["ax"]) * t
        ay_ = float(sub[il]["ay"]) * (1.0 - t) + float(sub[ir]["ay"]) * t
        az_ = float(sub[il]["az"]) * (1.0 - t) + float(sub[ir]["az"]) * t
        yn.append(math.sqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_))
    out["norm"] = (xs_n, yn)

    if sub and "mx_uT" in sub[0]:
        for key in ("mx_uT", "my_uT", "mz_uT"):
            ys = [
                float(sub[il][key]) * (1.0 - tt) + float(sub[ir][key]) * tt
                for _, il, ir, tt in gap_specs
            ]
            out[key] = (xs_n, ys)
    if sub and "temp_c" in sub[0]:
        ys = [
            float(sub[il]["temp_c"]) * (1.0 - tt) + float(sub[ir]["temp_c"]) * tt
            for _, il, ir, tt in gap_specs
        ]
        out["temp_c"] = (xs_n, ys)
    return out


def _scatter_gap_markers(
    ax,
    gaps: Dict[str, Tuple[List[float], List[float]]],
    series_colors: List[Tuple[str, str]],
) -> None:
    """Draw markers at interpolated gap points (edge-only, series color)."""
    for skey, color in series_colors:
        if skey not in gaps:
            continue
        gx, gy = gaps[skey]
        if not gx:
            continue
        mk, msize = _GAP_MARKER_STYLE.get(skey, ("o", 5.0))
        ax.scatter(
            gx,
            gy,
            s=max(14.0, msize * msize * 0.55),
            marker=mk,
            facecolors="none",
            edgecolors=color,
            linewidths=0.9,
            zorder=6,
        )


def dedupe_identical_rows(rows):
    seen = set()
    out = []
    for r in rows:
        key = [
            r["mac"],
            r["micros_timestamp"],
            r["sample_seq"],
            r["ax"],
            r["ay"],
            r["az"],
            r["gx"],
            r["gy"],
            r["gz"],
        ]
        if "mx_uT" in r:
            key.extend([r["mx_uT"], r["my_uT"], r["mz_uT"]])
        if "temp_c" in r:
            key.append(r["temp_c"])
        key = tuple(key)
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def micros_rows_to_monotonic_sec(rows):
    """
    Convert per-MAC sorted micros_timestamp to monotone time (s) from zero; if micros steps
    backward or jumps huge, assume nominal 10 ms step.
    `rows`: list of dict with micros_timestamp (int, µs).
    """
    if not rows:
        return []
    micros = [int(r["micros_timestamp"]) for r in rows]
    if len(micros) == 1:
        return [0.0]
    t_sec = [0.0]
    nominal_us = 10000
    max_plausible_jump_us = 86_400_000_000  # 1 day
    for i in range(1, len(micros)):
        delta = micros[i] - micros[i - 1]
        if delta < 0:
            delta = nominal_us
        elif delta > max_plausible_jump_us:
            delta = nominal_us
        t_sec.append(t_sec[-1] + delta * 1e-6)
    return t_sec


def plot_imu_csv_file(
    csv_path: str, *, show_seq_gaps: bool = True
) -> Optional[str]:
    """
    Load IMU CSV and plot (plt.show() until window closed).
    show_seq_gaps: markers at missing sample_seq per MAC, distinct marker per series.
    Returns None on success; error string if plot cannot run.
    """
    if not os.path.exists(csv_path):
        return f"File not found: {csv_path}"

    rows = []
    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return "CSV has no header row."
        names = [h.strip() for h in reader.fieldnames if h]
        if _looks_like_vl53_log_header(names):
            return (
                "This file is a VL53 log (vl53_log_*.csv), not an IMU log (imu_log_*.csv).\n"
                f"  Open path: {csv_path}\n"
                "imu_log_plotter only plots IMU accel/gyro. Choose imu_log_*.csv."
            )

        header_ok, has_type_col = _imu_csv_header_ok(reader.fieldnames)
        if not header_ok:
            return (
                "CSV header does not match imu_logger (nine required columns: "
                "mac,micros_timestamp,sample_seq,ax,ay,az,gx,gy,gz; "
                "optional leading type; then optional mx_uT,my_uT,mz_uT and/or temp_c).\n"
                f"  Got: {reader.fieldnames}"
            )
        fmap = _fieldname_map(reader.fieldnames)
        mag_keys = _mag_keys_from_fieldmap(fmap)
        temp_key = fmap.get("temp_c")
        for row in reader:
            try:
                if has_type_col:
                    t = str(row.get("type", "")).strip().upper()
                    if t and t != "IMU":
                        continue
                rec = {
                    "mac": _norm_mac(row["mac"]),
                    "micros_timestamp": _parse_micros_timestamp_cell(
                        row["micros_timestamp"]
                    ),
                    "sample_seq": _parse_sample_seq_cell(row["sample_seq"]),
                    "ax": float(row["ax"]),
                    "ay": float(row["ay"]),
                    "az": float(row["az"]),
                    "gx": float(row["gx"]),
                    "gy": float(row["gy"]),
                    "gz": float(row["gz"]),
                }
                if mag_keys:
                    kx, ky, kz = mag_keys
                    rec["mx_uT"] = float(row[kx])
                    rec["my_uT"] = float(row[ky])
                    rec["mz_uT"] = float(row[kz])
                if temp_key and str(row.get(temp_key, "")).strip() != "":
                    rec["temp_c"] = float(row[temp_key])
                rows.append(rec)
            except (ValueError, KeyError, TypeError):
                continue

    if not rows:
        return "CSV is empty or has no valid rows."

    n_loaded = len(rows)
    rows = dedupe_identical_rows(rows)
    n_dup_identical = n_loaded - len(rows)
    if n_dup_identical:
        print(
            f"Dropped {n_dup_identical} exact duplicate rows (same MAC, micros, seq, ax..gz"
            f"{' + mag' if any('mx_uT' in r for r in rows) else ''})."
        )

    # Duplicate (mac, sample_seq): keep first occurrence in file.
    by_key = {}
    for r in rows:
        k = (r["mac"], r["sample_seq"])
        if k not in by_key:
            by_key[k] = r
    rows_after_seq = list(by_key.values())
    n_dup_seq = len(rows) - len(rows_after_seq)
    if n_dup_seq:
        print(
            f"Dropped {n_dup_seq} duplicate rows (mac, sample_seq), kept first in file."
        )
    rows = rows_after_seq

    if not rows:
        return "No samples left after deduplication."

    by_mac = defaultdict(list)
    for r in rows:
        by_mac[r["mac"]].append(r)
    macs = sorted(by_mac.keys())

    plot_mag = any("mx_uT" in r for r in rows)
    plot_temp = any("temp_c" in r for r in rows)
    nrows = 3 + (1 if plot_mag else 0) + (1 if plot_temp else 0)
    fig_h = min(22.0, 6.2 + 2.45 * (nrows - 1))
    fig, axes = plt.subplots(nrows, 1, sharex=True, figsize=(10, fig_h))
    ax_list = np.atleast_1d(axes).ravel().tolist()
    ax1, ax2, ax3 = ax_list[0], ax_list[1], ax_list[2]
    i_next = 3
    ax_mag = ax_list[i_next] if plot_mag else None
    if plot_mag:
        i_next += 1
    ax_temp = ax_list[i_next] if plot_temp else None

    n_mac = len(macs)

    ax1.set_ylabel("Accel (m/s²)")
    ax2.set_ylabel("Gyro (rad/s)")
    ax3.set_ylabel("‖Acc‖ (m/s²)")
    if ax_mag is not None:
        ax_mag.set_ylabel("Mag (µT)")
    if ax_temp is not None:
        ax_temp.set_ylabel("T_die (°C)")
    for a in ax_list[:-1]:
        a.set_xlabel("")
    ax_list[-1].set_xlabel(
        "sample_seq − min (each MAC shifted to 0) — shared x-axis for all nodes"
    )

    base_title = f"IMU Log: {os.path.basename(csv_path)} ({len(rows)} samples, {n_mac} MAC)"
    gap_marker_total = 0

    for idx, mac in enumerate(macs):
        sub = sorted(
            by_mac[mac],
            key=lambda r: (r["sample_seq"], r["micros_timestamp"]),
        )
        seqs = [int(r["sample_seq"]) for r in sub]
        s0 = min(seqs)
        x_full = [s - s0 for s in seqs]
        si = _subsample_indices(len(sub), PLOT_MAX_POINTS_PER_SERIES)
        subp = [sub[i] for i in si]
        x_plot = [x_full[i] for i in si]
        ls = SLAVE_LINESTYLES[idx % len(SLAVE_LINESTYLES)]
        lab = _short_mac_label(mac)
        prefix = f"{lab} " if n_mac > 1 else ""
        plot_kw = {"lw": 1.0, "linestyle": ls}
        if len(sub) > PLOT_MAX_POINTS_PER_SERIES:
            plot_kw["rasterized"] = True

        ax1.plot(
            x_plot,
            [r["ax"] for r in subp],
            color=AXIS_COLORS["x"],
            **plot_kw,
            label=f"{prefix}ax",
        )
        ax1.plot(
            x_plot,
            [r["ay"] for r in subp],
            color=AXIS_COLORS["y"],
            **plot_kw,
            label=f"{prefix}ay",
        )
        ax1.plot(
            x_plot,
            [r["az"] for r in subp],
            color=AXIS_COLORS["z"],
            **plot_kw,
            label=f"{prefix}az",
        )

        ax2.plot(
            x_plot,
            [r["gx"] for r in subp],
            color=AXIS_COLORS["x"],
            **plot_kw,
            label=f"{prefix}gx",
        )
        ax2.plot(
            x_plot,
            [r["gy"] for r in subp],
            color=AXIS_COLORS["y"],
            **plot_kw,
            label=f"{prefix}gy",
        )
        ax2.plot(
            x_plot,
            [r["gz"] for r in subp],
            color=AXIS_COLORS["z"],
            **plot_kw,
            label=f"{prefix}gz",
        )

        norm_ = [
            math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2) for r in subp
        ]
        ax3.plot(
            x_plot,
            norm_,
            color="purple",
            **plot_kw,
            label=f"{prefix}‖acc‖",
        )

        if ax_mag is not None and plot_mag:
            ax_mag.plot(
                x_plot,
                [r["mx_uT"] for r in subp],
                color=AXIS_COLORS["x"],
                **plot_kw,
                label=f"{prefix}mx",
            )
            ax_mag.plot(
                x_plot,
                [r["my_uT"] for r in subp],
                color=AXIS_COLORS["y"],
                **plot_kw,
                label=f"{prefix}my",
            )
            ax_mag.plot(
                x_plot,
                [r["mz_uT"] for r in subp],
                color=AXIS_COLORS["z"],
                **plot_kw,
                label=f"{prefix}mz",
            )

        if ax_temp is not None and plot_temp:
            ax_temp.plot(
                x_plot,
                [r.get("temp_c", float("nan")) for r in subp],
                color="#9b59b6",
                **plot_kw,
                label=f"{prefix}T",
            )

        if show_seq_gaps:
            gaps = _gather_gap_points_per_series(sub, s0)
            if gaps:
                gap_marker_total += len(gaps["ax"][0])
                _scatter_gap_markers(
                    ax1,
                    gaps,
                    [
                        ("ax", AXIS_COLORS["x"]),
                        ("ay", AXIS_COLORS["y"]),
                        ("az", AXIS_COLORS["z"]),
                    ],
                )
                _scatter_gap_markers(
                    ax2,
                    gaps,
                    [
                        ("gx", AXIS_COLORS["x"]),
                        ("gy", AXIS_COLORS["y"]),
                        ("gz", AXIS_COLORS["z"]),
                    ],
                )
                _scatter_gap_markers(ax3, gaps, [("norm", "purple")])
                if ax_mag is not None and plot_mag:
                    _scatter_gap_markers(
                        ax_mag,
                        gaps,
                        [
                            ("mx_uT", AXIS_COLORS["x"]),
                            ("my_uT", AXIS_COLORS["y"]),
                            ("mz_uT", AXIS_COLORS["z"]),
                        ],
                    )
                if ax_temp is not None and plot_temp:
                    _scatter_gap_markers(
                        ax_temp, gaps, [("temp_c", "#9b59b6")]
                    )

    title_extra = ""
    if show_seq_gaps and gap_marker_total > 0:
        title_extra = (
            f"\n(hollow markers = missing samples, interpolated; {gap_marker_total} pts / all MAC)"
        )
    fig.suptitle(base_title + title_extra)

    for a in ax_list:
        a.grid(True, alpha=0.3)
        a.legend(loc="upper right", fontsize=6 if n_mac > 1 else 7, ncol=2 if n_mac > 1 else 1)

    plt.tight_layout()
    plt.show()
    return None


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(SCRIPT_DIR, csv_path)
    else:
        csv_path = ask_csv_path()
        if not csv_path:
            csv_path = find_latest_log()
            if not csv_path:
                print("No imu_log_*.csv found. Pass a file path:")
                print("  python imu_log_plotter.py imu_log_20260219_124026.csv")
                sys.exit(1)
            print(f"Using latest file: {os.path.basename(csv_path)}")

    err = plot_imu_csv_file(csv_path)
    if err:
        print(err)
        sys.exit(1)


if __name__ == "__main__":
    main()
