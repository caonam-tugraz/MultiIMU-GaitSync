#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) trên toàn bộ payload nhị phân
 * trước khi gửi UART; PC khớp imu_serial_codec.py (2 byte LE sau payload).
 */
static inline uint16_t imu_serial_crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc = (uint16_t)(crc ^ ((uint16_t)data[i] << 8));
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

// Dùng cho Arduino: prototype tự sinh nằm sau khối #include — giữ mọi typedef ở header
// được include làm dòng #include cuối trong sketch.

/** Master Serial: khớp imu_logger.py — lùi seq ≤ ngưỡng → không gửi Serial. */
#define MASTER_SEQ_RESET_THRESHOLD 1000000U
#define MASTER_SEQ_REORDER_BACK_MAX 256U
/** Tối đa MAC theo dõi (≥ MAX_SLAVES). */
#define MASTER_MAC_SEQ_TRACK 16

/** Master cấp TDMA: 10 slot × 5 ms = 50 ms/khung. */
#define MASTER_IMU_TX_SLOT_COUNT 10U
#define MASTER_SYNC_SLOT_COUNT 11U
#define MASTER_SLOT_UNASSIGNED 0xFFU
#define PACKET_T_SLOT_FLAG_VALID 0x01U

typedef struct {
  uint8_t mac[6];
  bool imu_inited;
  uint32_t imu_last;
  bool vl53_inited;
  uint32_t vl53_last;
} MacSeqTrack;

typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint32_t timestamp;
  uint64_t micros_timestamp;
  int timezone_offset;
  uint64_t request_time;
  uint64_t response_time;
  /** Slave: giá trị ADC (0..4095) chân pin chia áp; gửi trong gói SYNC. */
  uint16_t slave_adc_raw;
  /**
   * Slave: RTT vòng đồng bộ (NTP) từ vòng TIME trước, µs — (t4−t1)−(t3−t2).
   * 0 = chưa có / firmware cũ. Master TFT dùng trường này thay cho (t3−t2) chỉ ~vài trăm µs.
   */
  uint32_t last_sync_rtt_us;
  uint8_t imu_tx_slot;
  uint8_t sync_slot;
  uint8_t imu_tx_slot_count;
  uint8_t slot_flags;
} packet_t;

#define PACKET_T_LEGACY_SIZE ((size_t)offsetof(packet_t, imu_tx_slot))

typedef struct {
  char type[8];
  uint8_t mac[6];
  uint16_t _pad;
  uint32_t node_id;
  uint64_t micros_timestamp;
  float ax, ay, az;
  float gx, gy, gz;
} imu_packet_t;

#pragma pack(push, 1)
typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint64_t micros_timestamp;
  uint32_t sample_seq;
  int16_t ax, ay, az, gx, gy, gz;
  /**
   * Từ trường (BNO055: µT; int16 = round(µT × IMU_RAW_MAG_SCALE), scale 128 trên Slave BNO055).
   * Slave không có mag (ICM): gửi 0.
   */
  int16_t mx, my, mz;
  /** Nhiệt độ chip (ICM42688 die / BNO055): °C × 100 (int16), ví dụ 2534 = 25,34 °C */
  int16_t temp_centi_c;
  /**
   * Master: esp_timer_get_time() (µs) tại lúc gửi khung UART tới PC —
   * so sánh với micros_timestamp (timeline slave sau đồng bộ).
   */
  uint64_t master_micros_at_tx;
} imu_packet_raw_t;

/* Nhiều mẫu IMU trong một gói ESP-NOW (giảm số lần phát). Tối đa 15 mẫu. */
#define IMU_BATCH_MAX_SAMPLES 15
typedef struct {
  uint16_t delta_us_from_prev;
  int16_t ax, ay, az, gx, gy, gz;
  int16_t mx, my, mz;
} imu_batch_delta_t;

typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint8_t count;
  uint8_t reserved[3];
  uint32_t sample_seq0;
  uint64_t micros_t0;
  int16_t ax0, ay0, az0, gx0, gy0, gz0;
  int16_t mx0, my0, mz0;
  /**
   * Một giá trị nhiệt độ chip cho cả lô (đổi chậm): lấy từ mẫu cuối khi đóng gói (ICM/BNO055).
   * Master tách Serial gán cùng temp cho từng mẫu trong lô.
   */
  int16_t temp_centi_c;
  imu_batch_delta_t rest[IMU_BATCH_MAX_SAMPLES - 1];
} imu_batch_packet_t;
#pragma pack(pop)

/* Kích thước header + mẫu đầu (không có rest). */
#define IMU_BATCH_HEADER_SIZE 54

/**
 * Master → Slave: yêu cầu gửi lại một dải mẫu (theo sample_seq) bị thiếu.
 * type: "IMU_LOST" (8 byte), không \0 ở giữa; peer ESP-NOW = target_mac.
 */
#pragma pack(push, 1)
typedef struct {
  char type[8];
  uint8_t target_mac[6];
  uint32_t node_id;
  uint32_t seq_first;
  uint32_t seq_count;
} imu_lost_request_t;
#pragma pack(pop)
#define IMU_LOST_REQUEST_SIZE 26U

#define VL53_ZONE_COUNT 16
/* Raw int16 trên air (1 LSB = 1 mm), clamp từ uint16 driver; sample_seq như IMU. */
#pragma pack(push, 1)
typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint32_t timestamp;
  uint64_t micros_timestamp;
  uint32_t sample_seq;
  int16_t distance_raw[VL53_ZONE_COUNT];
} vl53_packet_t;

/* Nhiều mẫu VL53 (int16 raw / zone) trong một gói ESP-NOW — cùng ý tưởng IMU_RAWB. */
#define VL53_BATCH_MAX_SAMPLES 6
typedef struct {
  uint16_t delta_us_from_prev;
  int16_t distance_raw[VL53_ZONE_COUNT];
} vl53_batch_delta_t;

typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint8_t count;
  uint8_t reserved[3];
  uint32_t sample_seq0;
  uint64_t micros_t0;
  int16_t d0[VL53_ZONE_COUNT];
  vl53_batch_delta_t rest[VL53_BATCH_MAX_SAMPLES - 1];
} vl53_batch_packet_t;

#define VL53_BATCH_HEADER_SIZE (8 + 6 + 4 + 1 + 3 + 4 + 8 + VL53_ZONE_COUNT * 2)
#pragma pack(pop)

/**
 * type: "CALIB" = Master→Slave áp dụng bias/scale (+ NVS nếu save_to_nvs);
 *       "CALGET" = Master→Slave yêu cầu đọc thông số hiện tại;
 *       "CALREP" = Slave→Master trả bias/scale/global/enabled (không ghi NVS).
 * gyro_bias: offset gyro (°/s), trừ trước khi gói IMU — firmware cũ chỉ gửi 64 byte (không có 3 float này).
 */
typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint32_t timestamp;
  uint64_t micros_timestamp;
  float bias[3];
  float scale[3];
  float global_scale;
  uint8_t save_to_nvs;
  uint8_t enabled;
  uint8_t reserved[2];
  float gyro_bias[3];
} accel_calib_packet_t;

/** Kích thước gói calib trước khi có gyro_bias (tương thích Master/Tool cũ). */
#define ACCEL_CALIB_PACKET_LEGACY_SIZE 64
