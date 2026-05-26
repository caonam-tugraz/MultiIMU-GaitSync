#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nhận dữ liệu IMU từ Master (qua Serial) và ghi vào file CSV.

Master (IMU_SERIAL_BINARY=1): gửi khung nhị phân
  [0xA5,0x5A,0xA5,0x5A] + 58 byte imu_packet_raw_t + 2 byte CRC-16/CCITT-FALSE (LE trên payload)
  [0xB5,0x5B,0xB5,0x5B] + 66 byte vl53_packet_t + 2 byte CRC (LE)
  (Firmware cũ không CRC: chỉ 58 / 66 byte payload — imu_serial_codec vẫn đọc.)
Hoặc dòng văn bản IMU,MAC,micros[,master_micros],seq,ax..gz[,mx_uT,my_uT,mz_uT][,temp].

Nếu Master in VL53 (text hoặc khung B5+66 byte; raw int16/mm, có sample_seq) — ghi file riêng
(vl53_log_*.csv) với cùng micros_timestamp (timeline thống nhất với IMU).

Đơn vị CSV IMU:
  - accel: m/s^2
  - gyro: rad/s (đổi từ deg/s sau khi giải scale)
  - mx_uT,my_uT,mz_uT: µT (0 nếu nguồn không có từ)

Chạy:
  python imu_logger.py [COM_PORT] [imu.csv] [vl53.csv] [--duration-hours H]
  - Mặc định: thu liên tục 3 giờ rồi tự dừng.
  - --duration-hours 0: không giới hạn (chỉ dừng bằng Ctrl+C).
  - vl53.csv: tùy chọn; mặc định cùng thư mục, tên vl53_log_YYYYMMDD_HHMMSS.csv
    (trùng mốc thời gian với imu.csv nếu không chỉ định file IMU).

Kiểm tra nhanh sample_seq theo từng MAC: in [GAP]/[SEQ_BACK]/[seq reset] kèm số dòng (lẽ ra ghi).
[SEQ_BACK]: seq lùi ít (bù/retx/trễ); vẫn ghi CSV, không cập nhật mốc last (tiến tới theo mẫu mới hơn).
[seq reset]: seq nhảy lớn hoặc lùi nhiều (slave reboot / reset) — cập nhật last, ghi CSV bình thường.
Ctrl+C: thời gian PC, tổng giây, bảng theo MAC (gồm cột ước Hz = N/T và Δ=N/100−T so với 100Hz),
  mẫu thiếu (gap), trùng, SEQ_BACK.
"""

import argparse
import sys
import os
import re
import serial
import time
from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from imu_serial_codec import (
    DEG_TO_RAD,
    feed_imu_serial,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_BAUD = 921600
DEFAULT_PORT = "COM18"
# Thu mặc định 3 giờ rồi tự dừng (--duration-hours 0 = không giới hạn).
DEFAULT_DURATION_HOURS = 3.0

# Chênh lệch seq lớn hơn ngưỡng này coi là reset/reconnect, không báo "thiếu" hàng loạt.
_SEQ_RESET_THRESHOLD = 1_000_000
# Nếu seq giảm so với last nhưng chênh lệch <= ngưỡng này: coi là gói trễ / lệch thứ tự ([SEQ_BACK]).
# Nếu chênh lệch lớn hơn: coi là slave reset (seq mới sau reboot), cập nhật last và vẫn ghi CSV.
_SEQ_REORDER_BACK_MAX = 256

VL53_ZONE_COUNT = 16


def _norm_mac(mac: str) -> str:
    return str(mac).strip().upper()


def _imu_data_line_sort_key(line: str) -> Tuple[str, int, int]:
    """Cột: mac, micros, sample_seq — sắp theo (mac, seq, micros)."""
    p = line.rstrip("\n").split(",")
    if len(p) < 3:
        return (_norm_mac(p[0]) if p else "", 0, 0)
    try:
        return (_norm_mac(p[0]), int(p[2]), int(p[1]))
    except ValueError:
        return (_norm_mac(p[0]), 0, 0)


def _default_vl53_path(imu_path: str) -> str:
    """Từ imu_log_*.csv → vl53_log_*.csv cùng thư mục, cùng phần timestamp."""
    base = os.path.basename(imu_path)
    dirn = os.path.dirname(imu_path)
    if base.startswith("imu_log_") and base.lower().endswith(".csv"):
        stem = "vl53_log_" + base[8:]  # sau 'imu_log_'
        return os.path.join(dirn, stem) if dirn else stem
    m = re.match(r"^(.*)imu(.*)\.csv$", base, re.IGNORECASE)
    if m:
        return os.path.join(dirn, f"{m.group(1)}vl53{m.group(2)}.csv") if dirn else f"{m.group(1)}vl53{m.group(2)}.csv"
    root, ext = os.path.splitext(imu_path)
    return root + "_vl53" + ext


def _default_mac_seq_stats():
    return {
        "received": 0,
        "missed": 0,
        "filled": 0,  # mẫu bù (retx / seq lùi) đã ghi
        "duplicate": 0,
        "seq_back": 0,
    }


def _check_sample_seq_gap(
    mac: str,
    seq: int,
    last_by_mac: dict,
    csv_file_line: int,
    stats_by_mac: dict,
) -> Tuple[bool, bool]:
    """
    (ghi_csv, mẫu_bù_để_sắp_lại).
    Seq lùi ít: vẫn ghi; mẫu_bù_để_sắp_lại=True → ghi lại cả file theo thứ tự seq.
    """
    st = stats_by_mac[mac]
    update_last = True
    late_fill = False

    last = last_by_mac.get(mac)
    if last is None:
        last_by_mac[mac] = seq
        return True, False

    delta = (seq - last) & 0xFFFFFFFF
    if delta == 0:
        st["duplicate"] += 1
        print(
            f"[WARN] trùng sample_seq? csv_dòng={csv_file_line} mac={mac} seq={seq}",
            flush=True,
        )
    elif delta == 1:
        pass
    elif delta > 0x7FFFFFFF:
        # Trên vòng uint32: seq nhỏ hơn last. Có thể (1) bù/retx (2) gói trễ (3) slave reset lớn.
        backward = (last - seq) & 0xFFFFFFFF
        if backward <= _SEQ_REORDER_BACK_MAX:
            st["seq_back"] += 1
            update_last = False
            late_fill = True
            print(
                f"[SEQ_BACK] ghi + sắp lại theo sample_seq | csv_dòng_lẽ_ra={csv_file_line} "
                f"mac={mac} seq={seq} (last mốc vẫn={last})",
                flush=True,
            )
        else:
            print(
                f"[seq reset] csv_dòng={csv_file_line} mac={mac} seq giảm nhiều (coi slave reset): "
                f"last={last} -> seq={seq} (lùi ~{backward})",
                flush=True,
            )
    elif delta > _SEQ_RESET_THRESHOLD:
        print(
            f"[seq reset] csv_dòng={csv_file_line} mac={mac} nhảy lớn (coi reset): "
            f"last={last} -> seq={seq}",
            flush=True,
        )
    elif delta > 1:
        missing = int(delta - 1)
        st["missed"] += missing
        print(
            f"[GAP] csv_dòng={csv_file_line} mac={mac} thiếu ~{missing} mẫu "
            f"(seq {last} -> {seq})",
            flush=True,
        )
    if update_last:
        last_by_mac[mac] = seq
    return True, late_fill


def _print_seq_stats_summary(
    stats_by_mac: dict,
    *,
    start_dt: Optional[datetime] = None,
    end_dt: Optional[datetime] = None,
    vl53_by_mac: Optional[Dict[str, int]] = None,
    count_imu_total: int = 0,
    count_vl53_total: int = 0,
) -> None:
    """In tổng hợp sau Ctrl+C: thời gian phiên, thiếu seq, trùng, SEQ_BACK, theo MAC."""
    dur_s: Optional[float] = None
    if start_dt is not None and end_dt is not None:
        dur_s = (end_dt - start_dt).total_seconds()
        print("\n=== Phiên ghi (đồng hồ PC) ===")
        print(f"Bắt đầu:        {start_dt.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Kết thúc:       {end_dt.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Tổng thời gian: {dur_s:.3f} s")
        print(
            f"Dòng đã ghi:    IMU={count_imu_total}  |  VL53={count_vl53_total}"
        )

    v53map = vl53_by_mac or {}
    has_vl53 = bool(v53map) and sum(v53map.values()) > 0
    if not stats_by_mac and not has_vl53:
        if start_dt is None:
            print("\n(Chưa có thống kê sample_seq — chưa nhận gói IMU có seq.)")
        else:
            print(
                "\n(Chưa có dòng IMU/VL53 — chỉ có mốc thời gian phiên ở trên.)"
            )
        return

    total_missed = sum(s.get("missed", 0) for s in stats_by_mac.values())
    total_filled = sum(s.get("filled", 0) for s in stats_by_mac.values())
    total_dup = sum(s.get("duplicate", 0) for s in stats_by_mac.values())
    total_seq_back = sum(s.get("seq_back", 0) for s in stats_by_mac.values())
    total_recv = sum(s.get("received", 0) for s in stats_by_mac.values())

    print("\n=== Thống kê sample_seq (IMU) ===")
    print(f"Tổng mẫu ước lượng bị thiếu (gap seq):     {total_missed}")
    print(f"Tổng mẫu bù (đã ghi, retx/seq lùi):     {total_filled}")
    print(f"Tổng lần trùng sample_seq:                {total_dup}")
    print(f"Tổng gói bỏ qua [SEQ_BACK]:              {total_seq_back}")
    print(f"Tổng dòng IMU đã ghi (mọi MAC):          {total_recv}")

    macs = sorted(set(stats_by_mac.keys()) | set(v53map.keys()))
    if macs:
        _print_mac_stats_table(stats_by_mac, v53map, macs, session_duration_s=dur_s)


def _imu_rate_vs_session_cell(n_imu: int, session_duration_s: Optional[float]) -> str:
    """
    So sánh với 100 Hz: N/100 (s) vs thời gian phiên T (s); f TB = N/T.
    Một cột: hiển thị f TB và Δ = N/100 − T.
    """
    if (
        session_duration_s is None
        or session_duration_s <= 0.0
        or n_imu <= 0
    ):
        return "—"
    implied_s = n_imu / 100.0
    delta_s = implied_s - session_duration_s
    hz_avg = n_imu / session_duration_s
    return f"{hz_avg:.2f} Hz  Δ{delta_s:+.4f}s"


def _print_mac_stats_table(
    stats_by_mac: dict,
    v53map: Dict[str, int],
    macs: List[str],
    *,
    session_duration_s: Optional[float] = None,
) -> None:
    """In bảng theo từng MAC: IMU, VL53, thiếu, đã bù, trùng, SEQ_BACK, ước Hz vs 100Hz."""
    headers = (
        "MAC",
        "IMU (dòng)",
        "VL53 (dòng)",
        "Thiếu (~gap)",
        "Đã bù",
        "Trùng seq",
        "SEQ_BACK",
        "Ước Hz vs 100Hz (N/T, Δ=N/100−T)",
    )
    rows_data = []
    for mac in macs:
        s = stats_by_mac.get(mac)
        r = s.get("received", 0) if s else 0
        m = s.get("missed", 0) if s else 0
        filled = s.get("filled", 0) if s else 0
        d = s.get("duplicate", 0) if s else 0
        sb = s.get("seq_back", 0) if s else 0
        v53 = v53map.get(mac, 0)
        rate_cell = _imu_rate_vs_session_cell(r, session_duration_s)
        rows_data.append((mac, r, v53, m, filled, d, sb, rate_cell))

    ncols = len(headers)
    str_rows: List[Tuple[str, ...]] = [tuple(headers)]
    for tup in rows_data:
        str_rows.append(tuple(str(x) for x in tup))

    widths = [0] * ncols
    for row in str_rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))

    def hline() -> str:
        return "+" + "+".join("-" * (w + 2) for w in widths) + "+"

    def fmt_row(row: tuple, header: bool = False) -> str:
        parts = []
        for i, cell in enumerate(row):
            w = widths[i]
            if i == 0 or header or i == ncols - 1:
                parts.append(" " + str(cell).ljust(w) + " ")
            else:
                parts.append(" " + str(cell).rjust(w) + " ")
        return "|" + "|".join(parts) + "|"

    print("\nTheo từng MAC:")
    if session_duration_s is not None and session_duration_s > 0:
        print(
            "  (Δ = IMU_dòng/100 − T_phiên; f TB = IMU_dòng/T; "
            "Δ≈0 và f TB≈100 Hz ⇔ trung bình đúng 100 Hz trong phiên.)"
        )
    print(hline())
    print(fmt_row(str_rows[0], header=True))
    print(hline())
    for row in str_rows[1:]:
        print(fmt_row(row))
    print(hline())


def main():
    ap = argparse.ArgumentParser(
        description="Ghi log IMU/VL53 từ Master (Serial); tùy chọn giới hạn thời gian thu."
    )
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT, help="Cổng COM")
    ap.add_argument(
        "out_csv",
        nargs="?",
        default=None,
        help="File CSV IMU (mặc định imu_log_YYYYMMDD_HHMMSS.csv trong thư mục script)",
    )
    ap.add_argument(
        "vl53_csv",
        nargs="?",
        default=None,
        help="File CSV VL53 (mặc định cặp với file IMU)",
    )
    ap.add_argument(
        "--duration-hours",
        type=float,
        default=DEFAULT_DURATION_HOURS,
        metavar="H",
        help=(
            "Thu liên tục H giờ rồi tự dừng (mặc định %(default)s). "
            "0 = không giới hạn, chỉ dừng bằng Ctrl+C."
        ),
    )
    args = ap.parse_args()

    port = args.port
    out_name = args.out_csv or f"imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    out_path = os.path.join(SCRIPT_DIR, out_name) if not os.path.isabs(out_name) else out_name

    if args.vl53_csv is not None:
        vl53_path = args.vl53_csv
        if not os.path.isabs(vl53_path):
            vl53_path = os.path.join(SCRIPT_DIR, vl53_path)
    else:
        vl53_path = _default_vl53_path(out_path)

    session_limit_s: Optional[float] = None
    if args.duration_hours > 0:
        session_limit_s = float(args.duration_hours) * 3600.0

    imu_header = (
        "mac,micros_timestamp,sample_seq,ax,ay,az,gx,gy,gz,mx_uT,my_uT,mz_uT,temp_c,master_micros"
    )
    vl53_header = (
        "mac,micros_timestamp,timestamp_ms,sample_seq,"
        + ",".join(f"z{i}" for i in range(VL53_ZONE_COUNT))
    )

    print(f"Cổng Serial: {port}, Baud: {DEFAULT_BAUD}")
    print(f"File IMU:  {out_path}")
    print(f"File VL53: {vl53_path}")
    if session_limit_s is None:
        print("Thời gian thu: không giới hạn (Ctrl+C để dừng).")
    else:
        print(
            f"Thời gian thu: tối đa {args.duration_hours:g} h (~{session_limit_s:.0f} s), sau đó tự dừng."
        )
    print("Đang đọc dữ liệu từ Master...")

    # USB-UART (CP210x/CH340…) thường nối DTR→EN, RTS→IO0: mở COM có thể pulse → ESP reset.
    # Gán DTR/RTS = False trước open (pyserial 3.x) và sau open để tránh reset khi logger kết nối.
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = DEFAULT_BAUD
        ser.timeout = 0.05
        try:
            ser.dtr = False
            ser.rts = False
        except AttributeError:
            pass
        ser.open()
    except serial.SerialException as e:
        print(f"Lỗi mở cổng {port}: {e}")
        sys.exit(1)
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except OSError:
        pass

    buf = bytearray()
    count_imu = 0
    count_vl53 = 0
    last_seq_by_mac = {}
    stats_by_mac: dict = defaultdict(_default_mac_seq_stats)
    vl53_by_mac: Dict[str, int] = defaultdict(int)
    session_start_dt: Optional[datetime] = None
    stop_reason: Optional[str] = None  # "keyboard" | "duration"

    try:
        with open(out_path, "w", encoding="utf-8") as f_imu, open(
            vl53_path, "w", encoding="utf-8"
        ) as f_vl53:
            f_imu.write(imu_header + "\n")
            f_vl53.write(vl53_header + "\n")
            imu_data_lines: List[str] = []
            session_start_dt = datetime.now()
            while True:
                if session_limit_s is not None and session_start_dt is not None:
                    elapsed = (datetime.now() - session_start_dt).total_seconds()
                    if elapsed >= session_limit_s:
                        stop_reason = "duration"
                        print(
                            f"\nĐã đủ thời gian thu ({args.duration_hours:g} h) — dừng tự động.",
                            flush=True,
                        )
                        break

                chunk = ser.read(ser.in_waiting or 4096)
                if not chunk:
                    time.sleep(0.005)
                    continue
                rows, buf = feed_imu_serial(buf, chunk)
                for row in rows:
                    kind = row.get("kind", "imu")
                    if kind == "vl53":
                        mac_csv = _norm_mac(row["mac"])
                        zones = row["zones"]
                        z_csv = ",".join(str(z) for z in zones)
                        line_csv = (
                            f"{mac_csv},{row['micros']},{row['timestamp_ms']},"
                            f"{row['sample_seq']},{z_csv}\n"
                        )
                        f_vl53.write(line_csv)
                        f_vl53.flush()
                        count_vl53 += 1
                        vl53_by_mac[mac_csv] += 1
                        if count_vl53 % 100 == 0:
                            print(
                                f"Đã ghi {count_vl53} dòng VL53 → {os.path.basename(vl53_path)}",
                                flush=True,
                            )
                        continue

                    gx_rad = row["gx"] * DEG_TO_RAD
                    gy_rad = row["gy"] * DEG_TO_RAD
                    gz_rad = row["gz"] * DEG_TO_RAD
                    mac_csv = _norm_mac(row["mac"])
                    csv_file_line = count_imu + 2
                    seq = row.get("sample_seq")
                    late_fill = False
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
                            write_row, late_fill = _check_sample_seq_gap(
                                mac_csv,
                                seq_i,
                                last_seq_by_mac,
                                csv_file_line,
                                stats_by_mac,
                            )
                    if not write_row:
                        continue
                    tc = row.get("temp_c")
                    temp_out = float(tc) if tc is not None else 0.0
                    mx_u = row.get("mx_uT")
                    my_u = row.get("my_uT")
                    mz_u = row.get("mz_uT")
                    mx_out = float(mx_u) if mx_u is not None else 0.0
                    my_out = float(my_u) if my_u is not None else 0.0
                    mz_out = float(mz_u) if mz_u is not None else 0.0
                    mm = row.get("master_micros")
                    master_out = int(mm) if mm is not None else 0
                    line_csv = (
                        f"{mac_csv},{row['micros']},{seq_i},"
                        f"{row['ax']},{row['ay']},{row['az']},"
                        f"{gx_rad},{gy_rad},{gz_rad},"
                        f"{mx_out},{my_out},{mz_out},{temp_out},{master_out}\n"
                    )
                    imu_data_lines.append(line_csv)
                    if late_fill:
                        imu_data_lines.sort(key=_imu_data_line_sort_key)
                        f_imu.seek(0)
                        f_imu.write(imu_header + "\n" + "".join(imu_data_lines))
                        f_imu.truncate()
                    else:
                        f_imu.write(line_csv)
                    f_imu.flush()
                    count_imu += 1
                    stats_by_mac[mac_csv]["received"] += 1
                    if late_fill:
                        stats_by_mac[mac_csv]["filled"] = (
                            stats_by_mac[mac_csv].get("filled", 0) + 1
                        )
                    if count_imu % 100 == 0:
                        print(
                            f"Đã ghi {count_imu} dòng IMU → {os.path.basename(out_path)}",
                            flush=True,
                        )
    except KeyboardInterrupt:
        stop_reason = "keyboard"
        print("", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass

        if session_start_dt is None:
            return

        session_end_dt = datetime.now()
        if stop_reason == "keyboard":
            print(
                f"\nDừng (Ctrl+C). IMU: {count_imu} dòng → {out_path}"
                f"\n       VL53: {count_vl53} dòng → {vl53_path}"
            )
        elif stop_reason == "duration":
            print(
                f"\nIMU: {count_imu} dòng → {out_path}"
                f"\nVL53: {count_vl53} dòng → {vl53_path}"
            )
        else:
            # ví dụ lỗi trước khi vào vòng lặp — vẫn in tóm tắt nếu có mốc thời gian
            print(
                f"\nKết thúc ghi. IMU: {count_imu} dòng → {out_path}"
                f"\n       VL53: {count_vl53} dòng → {vl53_path}"
            )

        _print_seq_stats_summary(
            stats_by_mac,
            start_dt=session_start_dt,
            end_dt=session_end_dt,
            vl53_by_mac=dict(vl53_by_mac),
            count_imu_total=count_imu,
            count_vl53_total=count_vl53,
        )


if __name__ == "__main__":
    main()
