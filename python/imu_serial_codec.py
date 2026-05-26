#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Giải mã luồng Serial từ Master: khung nhị phân IMU / VL53 (raw) hoặc dòng văn bản IMU, VL53,...

Khung nhị phân IMU (IMU_SERIAL_BINARY=1):
  [0xA5,0x5A,0xA5,0x5A] + payload 58 byte (imu_packet_raw_t) + 2 byte CRC-16/CCITT-FALSE (LE)
  — CRC tính trên đúng 58 byte payload; firmware cũ: chỉ 58 byte (không CRC) vẫn đọc được.
  — Legacy 50 byte payload (không CRC) vẫn hỗ trợ.

Khung nhị phân VL53 (cùng IMU_SERIAL_BINARY=1):
  [0xB5,0x5B,0xB5,0x5B] + payload 66 byte (packed vl53_packet_t) + 2 byte CRC-16 LE
  — Firmware cũ: 66 byte không CRC vẫn đọc được.

Scale (khớp firmware):
  - accel int16 -> m/s^2: chia / ACC_SCALE
  - gyro int16 -> deg/s: chia / GYRO_SCALE (32767/IMU_GYRO_FS_DPS); logger chuyển rad/s như cũ
"""

import struct
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import serial

# Phải trùng #define trên Master/Slave
IMU_FRAME_SYNC = bytes([0xA5, 0x5A, 0xA5, 0x5A])
IMU_RAW_PAYLOAD_SIZE = 58
IMU_RAW_PAYLOAD_SIZE_LEGACY = 50
# CRC-16/CCITT-FALSE (LE) sau payload; Master mới gửi, PC bỏ khung nếu sai CRC.
IMU_FRAME_CRC_LEN = 2

VL53_FRAME_SYNC = bytes([0xB5, 0x5B, 0xB5, 0x5B])
VL53_RAW_PAYLOAD_SIZE = 66
_VL53_RAW_STRUCT = struct.Struct("<8s6sIIQI16h")
ACC_SCALE = 512.0
# Khớp IMU_GYRO_FS_DPS / IMU_RAW_GYRO_SCALE trên Slave & Master (int16 đóng gói gyro °/s)
IMU_GYRO_FS_DPS = 2000.0
GYRO_SCALE = 32767.0 / IMU_GYRO_FS_DPS
DEG_TO_RAD = math.pi / 180.0
MAG_SCALE = 128.0  # Slave BNO055 IMU_RAW_MAG_SCALE (µT)

# '<' packed — 58 byte: ... + 10h + Q master_micros_at_tx; 50 byte (legacy): chỉ 10h
_IMU_RAW_STRUCT = struct.Struct("<8s6sIQI10hQ")
_IMU_RAW_STRUCT_LEGACY = struct.Struct("<8s6sIQI10h")


def _crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF — khớp imu_serial_crc16_ccitt_false() trên Master."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_imu_line_text(line: str) -> Optional[Dict[str, Any]]:
    """Parse IMU,MAC,micros[,seq],ax..gz [,mx,my,mz µT] [,temp].

    Chuẩn slave BNO055 + mag + temp: len==14 (có seq).
    Legacy: 9 / 10 / 11 field (không mag).
    """
    line = line.strip()
    if not line.startswith("IMU,"):
        return None
    parts = line.split(",")
    try:
        if len(parts) == 15:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "master_micros": int(parts[3]),
                "sample_seq": int(parts[4]),
                "ax": float(parts[5]),
                "ay": float(parts[6]),
                "az": float(parts[7]),
                "gx": float(parts[8]),
                "gy": float(parts[9]),
                "gz": float(parts[10]),
                "mx_uT": float(parts[11]),
                "my_uT": float(parts[12]),
                "mz_uT": float(parts[13]),
                "temp_c": float(parts[14]),
            }
        if len(parts) == 14:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "sample_seq": int(parts[3]),
                "ax": float(parts[4]),
                "ay": float(parts[5]),
                "az": float(parts[6]),
                "gx": float(parts[7]),
                "gy": float(parts[8]),
                "gz": float(parts[9]),
                "mx_uT": float(parts[10]),
                "my_uT": float(parts[11]),
                "mz_uT": float(parts[12]),
                "temp_c": float(parts[13]),
            }
        if len(parts) == 13:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "sample_seq": int(parts[3]),
                "ax": float(parts[4]),
                "ay": float(parts[5]),
                "az": float(parts[6]),
                "gx": float(parts[7]),
                "gy": float(parts[8]),
                "gz": float(parts[9]),
                "mx_uT": float(parts[10]),
                "my_uT": float(parts[11]),
                "mz_uT": float(parts[12]),
                "temp_c": None,
            }
        if len(parts) == 9:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "sample_seq": None,
                "ax": float(parts[3]),
                "ay": float(parts[4]),
                "az": float(parts[5]),
                "gx": float(parts[6]),
                "gy": float(parts[7]),
                "gz": float(parts[8]),
                "temp_c": None,
            }
        if len(parts) == 10:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "sample_seq": int(parts[3]),
                "ax": float(parts[4]),
                "ay": float(parts[5]),
                "az": float(parts[6]),
                "gx": float(parts[7]),
                "gy": float(parts[8]),
                "gz": float(parts[9]),
                "temp_c": None,
            }
        if len(parts) == 11:
            return {
                "kind": "imu",
                "mac": parts[1].strip(),
                "micros": int(parts[2]),
                "sample_seq": int(parts[3]),
                "ax": float(parts[4]),
                "ay": float(parts[5]),
                "az": float(parts[6]),
                "gx": float(parts[7]),
                "gy": float(parts[8]),
                "gz": float(parts[9]),
                "temp_c": float(parts[10]),
            }
    except (ValueError, IndexError):
        return None
    return None


def parse_imu_log_control_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Dòng do Master in khi bù mẫu thiếu (ESP-NOW), ví dụ:
      IMU_LOG,LOST_REQ,AA:BB:..:FF,<node_id>,<seq_first>,<seq_count>
      IMU_LOG,LOST_RTX,AA:BB:..:FF,<node_id>,<count>,<seq0>
      IMU_LOG,RETX_DROP,AA:BB:..:FF,<node_id>,<seq0>,<n>
    Trả về kind == 'imu_ctrl' (không ghi CSV).
    """
    line = line.strip()
    if not line.startswith("IMU_LOG,"):
        return None
    rest = line[8:]
    if rest.startswith("LOST_REQ,"):
        body = rest[9:]
        parts = body.rsplit(",", 3)
        if len(parts) != 4:
            return None
        mac_s, n_s, f_s, c_s = parts[0], parts[1], parts[2], parts[3]
        mac_u = mac_s.strip().upper()
        return {
            "kind": "imu_ctrl",
            "cmd": "LOST_REQ",
            "mac": mac_u,
            "node_id": int(n_s),
            "seq_first": int(f_s),
            "seq_count": int(c_s),
        }
    if rest.startswith("LOST_RTX,"):
        body = rest[9:]
        parts = body.rsplit(",", 3)
        if len(parts) != 4:
            return None
        mac_s, n_s, c_s, s0 = parts[0], parts[1], parts[2], parts[3]
        mac_u = mac_s.strip().upper()
        return {
            "kind": "imu_ctrl",
            "cmd": "LOST_RTX",
            "mac": mac_u,
            "node_id": int(n_s),
            "count": int(c_s),
            "seq0": int(s0),
        }
    if rest.startswith("RETX_DROP,"):
        body = rest[10:]
        parts = body.rsplit(",", 3)
        if len(parts) != 4:
            return None
        mac_s, n_s, s0, nch = parts[0], parts[1], parts[2], parts[3]
        mac_u = mac_s.strip().upper()
        return {
            "kind": "imu_ctrl",
            "cmd": "RETX_DROP",
            "mac": mac_u,
            "node_id": int(n_s),
            "seq0": int(s0),
            "n": int(nch),
        }
    return None


def parse_sync_gui_status_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Một dòng từ Master kèm SYNC: pin % (slave) + RSSI khung vừa nhận.
    SYNC,GUI,mac=AA:BB:CC:DD:EE:FF,pct=85,rssi_dbm=-65
    (rssi_dbm=-128: không có rx_ctrl; GUI có thể hiện "—")
    """
    s = line.strip()
    if not s.startswith("SYNC,GUI,mac="):
        return None
    parts = s.split(",")
    if len(parts) < 5 or parts[0] != "SYNC" or parts[1] != "GUI" or not parts[2].startswith(
        "mac="
    ):
        return None
    try:
        mac = parts[2][4:].strip()
        if not parts[3].startswith("pct=") or not parts[4].startswith("rssi_dbm="):
            return None
        pct = int(parts[3][4:])
        rssi = int(parts[4][9:])
    except (ValueError, IndexError):
        return None
    return {
        "kind": "sync_gui",
        "mac": mac,
        "bat_pct": pct,
        "rssi_dbm": rssi,
    }


def parse_vl53_line_text(line: str) -> Optional[Dict[str, Any]]:
    """Parse VL53,MAC,timestamp_ms,micros[,sample_seq],z0..z15 (raw int16/mm). Có hoặc không sample_seq."""
    line = line.strip()
    if not line.startswith("VL53,"):
        return None
    parts = line.split(",")
    try:
        mac = parts[1].strip()
        ts_ms = int(parts[2])
        micros = int(parts[3])
        if len(parts) >= 21:
            sample_seq = int(parts[4])
            z0 = 5
        elif len(parts) >= 20:
            sample_seq = 0
            z0 = 4
        else:
            return None
        zones = [int(parts[z0 + i]) for i in range(16)]
    except (ValueError, IndexError):
        return None
    return {
        "kind": "vl53",
        "mac": mac,
        "micros": micros,
        "timestamp_ms": ts_ms,
        "sample_seq": sample_seq,
        "zones": zones,
    }


def decode_vl53_raw_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    """66 byte vl53_packet_t -> cùng dict như parse_vl53_line_text."""
    if len(payload) != VL53_RAW_PAYLOAD_SIZE:
        return None
    try:
        tup = _VL53_RAW_STRUCT.unpack(payload)
    except struct.error:
        return None
    typ, mac_b, _node_id, ts_ms, micros, sample_seq = tup[:6]
    zones = list(tup[6:])
    if typ[:4] != b"VL53":
        return None
    if len(zones) != 16:
        return None
    mac = ":".join("%02X" % b for b in mac_b)
    return {
        "kind": "vl53",
        "mac": mac,
        "micros": micros,
        "timestamp_ms": ts_ms,
        "sample_seq": sample_seq,
        "zones": zones,
    }


def decode_imu_raw_payload(payload: bytes) -> Optional[Dict[str, Any]]:
    """imu_packet_raw_t -> dict (gyro deg/s trước khi nhân DEG_TO_RAD). Hỗ trợ 58 byte (có master_micros) hoặc 50 byte (cũ)."""
    if len(payload) not in (IMU_RAW_PAYLOAD_SIZE, IMU_RAW_PAYLOAD_SIZE_LEGACY):
        return None
    try:
        if len(payload) == IMU_RAW_PAYLOAD_SIZE:
            (
                typ,
                mac_b,
                _node_id,
                micros,
                sample_seq,
                ax,
                ay,
                az,
                gx,
                gy,
                gz,
                mx,
                my,
                mz,
                temp_centi_c,
                master_micros,
            ) = _IMU_RAW_STRUCT.unpack(payload)
        else:
            (
                typ,
                mac_b,
                _node_id,
                micros,
                sample_seq,
                ax,
                ay,
                az,
                gx,
                gy,
                gz,
                mx,
                my,
                mz,
                temp_centi_c,
            ) = _IMU_RAW_STRUCT_LEGACY.unpack(payload)
            master_micros = None
    except struct.error:
        return None
    if typ[:7] != b"IMU_RAW":
        return None
    mac = ":".join("%02X" % b for b in mac_b)
    out: Dict[str, Any] = {
        "kind": "imu",
        "mac": mac,
        "micros": micros,
        "sample_seq": sample_seq,
        "ax": ax / ACC_SCALE,
        "ay": ay / ACC_SCALE,
        "az": az / ACC_SCALE,
        "gx": gx / GYRO_SCALE,
        "gy": gy / GYRO_SCALE,
        "gz": gz / GYRO_SCALE,
        "mx_uT": mx / MAG_SCALE,
        "my_uT": my / MAG_SCALE,
        "mz_uT": mz / MAG_SCALE,
        "temp_c": temp_centi_c / 100.0,
    }
    if master_micros is not None:
        out["master_micros"] = int(master_micros)
    return out


def feed_imu_serial(
    buf: bytearray,
    chunk: bytes,
    *,
    include_unparsed_lines: bool = False,
) -> Tuple[List[Dict[str, Any]], bytearray]:
    """
    Thêm chunk vào buf, trích mẫu IMU (nhị phân / dòng), VL53 (nhị phân B5... / dòng text).
    Khung nhị phân: chỉ bỏ đủ byte khi decode thành công; sync giả (4 byte trùng ngẫu nhiên)
    → bỏ 1 byte để tránh trượt và micros/gyro sai trong CSV.
    Trả về (danh_sách_row, buf_đã_cắt).
    include_unparsed_lines: True → mỗi dòng text không phải IMU/VL53 thành {"kind":"log","text":...}
      (dùng cho imu_calib: CALIB_ACK, v.v.).
    """
    buf.extend(chunk)
    rows: List[Dict[str, Any]] = []
    # Giới hạn RAM; khi vượt, cắt phần đầu. Luồng chủ yếu nhị phân: có thể cắt giữa khung —
    # nhánh IMU/VL53 bên dưới chỉ consume đủ byte khi decode OK; nếu sync giả chỉ tịnh tiến 1 byte.
    max_buf = 384 * 1024
    keep_tail = 256 * 1024
    while True:
        if len(buf) > max_buf:
            cut = len(buf) - keep_tail
            if cut > 0:
                # Cắt bỏ đầu buffer; nếu gần mốc cắt có newline thì cắt sau dòng đó (ít xé câu log ASCII).
                lo = max(0, cut - 5000)
                nl = buf.rfind(b"\n", lo, cut)
                if nl >= 0:
                    del buf[: nl + 1]
                else:
                    del buf[:cut]

        imu_idx = buf.find(IMU_FRAME_SYNC)
        vl53_idx = buf.find(VL53_FRAME_SYNC)
        nl_idx = buf.find(b"\n")

        candidates: List[Tuple[int, str]] = []
        if imu_idx >= 0:
            candidates.append((imu_idx, "imu"))
        if vl53_idx >= 0:
            candidates.append((vl53_idx, "vl53"))
        if nl_idx >= 0:
            candidates.append((nl_idx, "nl"))

        if not candidates:
            break

        pos, kind = min(candidates, key=lambda x: x[0])

        if kind == "imu":
            base = pos + 4
            if len(buf) < base + IMU_RAW_PAYLOAD_SIZE_LEGACY:
                break
            avail = len(buf) - base

            if avail >= IMU_RAW_PAYLOAD_SIZE + IMU_FRAME_CRC_LEN:
                payload = bytes(buf[base : base + IMU_RAW_PAYLOAD_SIZE])
                crc_rx = struct.unpack_from("<H", buf, base + IMU_RAW_PAYLOAD_SIZE)[0]
                if _crc16_ccitt_false(payload) != crc_rx:
                    del buf[: pos + 1]
                    continue
                row = decode_imu_raw_payload(payload)
                if row is not None:
                    del buf[: base + IMU_RAW_PAYLOAD_SIZE + IMU_FRAME_CRC_LEN]
                    rows.append(row)
                else:
                    del buf[: pos + 1]
                continue

            if avail == IMU_RAW_PAYLOAD_SIZE + 1:
                break

            plen = (
                IMU_RAW_PAYLOAD_SIZE
                if avail >= IMU_RAW_PAYLOAD_SIZE
                else IMU_RAW_PAYLOAD_SIZE_LEGACY
            )
            payload = bytes(buf[base : base + plen])
            row = decode_imu_raw_payload(payload)
            if row is not None:
                del buf[: base + plen]
                rows.append(row)
            else:
                del buf[: pos + 1]
            continue

        if kind == "vl53":
            base = pos + 4
            if len(buf) < base + VL53_RAW_PAYLOAD_SIZE:
                break
            avail = len(buf) - base

            if avail >= VL53_RAW_PAYLOAD_SIZE + IMU_FRAME_CRC_LEN:
                payload = bytes(buf[base : base + VL53_RAW_PAYLOAD_SIZE])
                crc_rx = struct.unpack_from("<H", buf, base + VL53_RAW_PAYLOAD_SIZE)[0]
                if _crc16_ccitt_false(payload) != crc_rx:
                    del buf[: pos + 1]
                    continue
                row = decode_vl53_raw_payload(payload)
                if row is not None:
                    del buf[: base + VL53_RAW_PAYLOAD_SIZE + IMU_FRAME_CRC_LEN]
                    rows.append(row)
                else:
                    del buf[: pos + 1]
                continue

            if avail == VL53_RAW_PAYLOAD_SIZE + 1:
                break

            payload = bytes(buf[base : base + VL53_RAW_PAYLOAD_SIZE])
            row = decode_vl53_raw_payload(payload)
            if row is not None:
                del buf[: base + VL53_RAW_PAYLOAD_SIZE]
                rows.append(row)
            else:
                del buf[: pos + 1]
            continue

        line_bytes = buf[:pos]
        del buf[: pos + 1]
        try:
            line = line_bytes.decode("utf-8", errors="ignore")
        except Exception:
            continue
        row_vl = parse_vl53_line_text(line)
        if row_vl is not None:
            rows.append(row_vl)
            continue
        row = parse_imu_line_text(line)
        if row is not None:
            rows.append(row)
            continue
        row_ctrl = parse_imu_log_control_line(line)
        if row_ctrl is not None:
            rows.append(row_ctrl)
            continue
        row_sync = parse_sync_gui_status_line(line)
        if row_sync is not None:
            rows.append(row_sync)
            continue
        if include_unparsed_lines:
            s = line.strip()
            if s:
                rows.append({"kind": "log", "text": s})

    return rows, buf


def configure_serial_for_esp32(ser: serial.Serial) -> None:
    """Khớp imu_live_plot.py: DTR/RTS=False sau open để tránh reset/boot ESP32."""
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except (AttributeError, OSError, serial.SerialException):
        pass
    time.sleep(0.15)


def open_serial_for_esp32(port: str, baud: int, timeout: float = 0.02) -> serial.Serial:
    """Mở COM rồi cấu hình DTR/RTS — giống imu_live_plot.py."""
    ser = serial.Serial(port, baud, timeout=timeout)
    configure_serial_for_esp32(ser)
    return ser


def list_serial_port_devices() -> List[str]:
    """Danh sách tên cổng (COMx) trên PC — ưu tiên USB-UART thật hơn com0com."""
    from serial.tools import list_ports

    def _rank(p) -> tuple:
        desc = (p.description or "").lower()
        bonus = 0
        if "com0com" in desc or "emulator" in desc:
            bonus -= 100
        if any(k in desc for k in ("usb", "serial", "ch340", "cp210", "ftdi", "esp", "uart", "jtag")):
            bonus += 10
        return (bonus, p.device)

    ports = list(list_ports.comports())
    ports.sort(key=_rank, reverse=True)
    return [p.device for p in ports]


def serial_port_descriptions() -> Dict[str, str]:
    from serial.tools import list_ports

    out: Dict[str, str] = {}
    for p in list_ports.comports():
        out[p.device] = (p.description or "").strip()
    return out


def probe_serial_imu_port(
    port: str,
    baud: int = 921600,
    *,
    probe_timeout_s: float = 0.85,
    min_imu_frames: int = 1,
) -> int:
    """
    Mở tạm port, decode luồng Master (A5…/IMU text).
    Trả về số khung IMU hợp lệ (0 nếu không mở được hoặc không khớp).
    """
    try:
        ser = open_serial_for_esp32(port, baud, timeout=0.05)
    except serial.SerialException:
        return 0

    buf = bytearray()
    imu_count = 0
    deadline = time.monotonic() + probe_timeout_s
    try:
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(min(waiting, 4096))
            else:
                chunk = ser.read(256)
            if not chunk:
                time.sleep(0.008)
                continue
            rows, buf = feed_imu_serial(buf, chunk, include_unparsed_lines=False)
            imu_count += sum(1 for r in rows if r.get("kind") == "imu")
            if imu_count >= min_imu_frames:
                break
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return imu_count


def find_serial_port_with_imu(
    baud: int = 921600,
    *,
    ports: Optional[List[str]] = None,
    probe_timeout_s: float = 0.85,
    min_imu_frames: int = 2,
) -> Tuple[Optional[str], Dict[str, int]]:
    """
    Quét từng COM; chọn port có nhiều khung IMU nhất (≥ min_imu_frames nếu có thể).
    Trả về (port hay None, dict port → số khung IMU đếm được).
    """
    candidates = ports if ports is not None else list_serial_port_devices()
    counts: Dict[str, int] = {}
    best_port: Optional[str] = None
    best_n = 0
    for port in candidates:
        n = probe_serial_imu_port(
            port, baud, probe_timeout_s=probe_timeout_s, min_imu_frames=1
        )
        counts[port] = n
        if n > best_n:
            best_n = n
            best_port = port
    if best_n >= min_imu_frames:
        return best_port, counts
    if best_n >= 1:
        return best_port, counts
    return None, counts
