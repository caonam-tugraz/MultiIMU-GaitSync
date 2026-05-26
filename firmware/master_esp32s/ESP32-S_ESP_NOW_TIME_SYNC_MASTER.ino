#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <cstddef>
#include "esp_bt.h"
#include "esp_now_time_sync_types.h"

/** Master ESP32-S (không TFT): chỉ WiFi STA + ESP-NOW + Serial logger. */

#define CHANNEL 1
#define MAX_SLAVES 10
/** Phải > HELLO_RECONNECT_INTERVAL slave (3 s) và thời gian chờ SYNC/TBURST. */
#define SLAVE_RX_TIMEOUT_MS 5000UL
#define SLAVE_RSSI_MA_N 10
/** Chu kỳ in RSSI trung bình slave ra Serial (thay cập nhật TFT). */
#define SLAVE_RSSI_LOG_INTERVAL_MS 3000UL
#define USE_LONG_RANGE 1
/** esp_wifi_set_max_tx_power: đơn vị 0,25 dBm; ESP32 thường 8…84 (≈2…21 dBm). Tối thiểu = 8. */
#define MAX_TX_POWER 8
/** Tăng công suất phát khi RSSI slave (MA) thấp; giảm khi link ổn (hysteresis). */
#ifndef MASTER_WIFI_TX_POWER_ADAPTIVE
#define MASTER_WIFI_TX_POWER_ADAPTIVE 1
#endif
#ifndef MASTER_WIFI_TX_POWER_NORM_QDB
#define MASTER_WIFI_TX_POWER_NORM_QDB MAX_TX_POWER
#endif
#ifndef MASTER_WIFI_TX_POWER_BOOST_QDB
#define MASTER_WIFI_TX_POWER_BOOST_QDB 84
#endif
#ifndef MASTER_WIFI_TX_RSSI_BOOST_DB
#define MASTER_WIFI_TX_RSSI_BOOST_DB (-75)
#endif
#ifndef MASTER_WIFI_TX_RSSI_RESTORE_DB
#define MASTER_WIFI_TX_RSSI_RESTORE_DB (-65)
#endif
#ifndef MASTER_WIFI_TX_POWER_ADJ_INTERVAL_MS
#define MASTER_WIFI_TX_POWER_ADJ_INTERVAL_MS 2000U
#endif

/** Số mẫu tối đa mỗi gói IMU_LOST (chia nhỏ nếu thiếu dài). */
#ifndef MASTER_IMU_LOST_MAX_CHUNK
#define MASTER_IMU_LOST_MAX_CHUNK 32U
#endif
/**
 * 1 = phát hiện lỗ hổng seq IMU → gửi IMU_LOST cho slave truyền lại (+ timeout poll).
 * 0 = tắt tạm, không request retx (lọc Serial: MASTER_SERIAL_SEQ_FILTER).
 */
#ifndef MASTER_IMU_LOST_REQUEST_ENABLE
#define MASTER_IMU_LOST_REQUEST_ENABLE 0
#endif

/**
 * 1 = lọc trùng / reorder sample_seq (theo MAC) trước khi gửi UART tới PC.
 * 0 = gửi hết mọi mẫu nhận được (IMU + VL53), không bỏ mẫu theo seq.
 */
#ifndef MASTER_SERIAL_SEQ_FILTER
#define MASTER_SERIAL_SEQ_FILTER 0
#endif

/** Pin 1 cell Li-ion: đầy / cạn (V tại cell). % = nội suy theo Vcell (Vadc sau cầu × 2). */
#define VBAT_CELL_FULL_V 4.1f
#define VBAT_CELL_EMPTY_V 3.2f
/** Cầu chia: Vadc tại chân = Vcell × scale (0.5 → Vcell = Vadc / scale). */
#define VBAT_ADC_PIN_VCELL_SCALE 0.496f
#define VBAT_ADC_PIN_AT_CELL_FULL_V (VBAT_CELL_FULL_V * VBAT_ADC_PIN_VCELL_SCALE)
#define VBAT_ADC_REF_V 3.3f
#define VBAT_VCELL_PER_VADC (VBAT_CELL_FULL_V / VBAT_ADC_PIN_AT_CELL_FULL_V)

/**
 * Slave HELLO_KEEPALIVE=0 → chỉ HELLO khi chưa connected (reconnect).
 * Master luôn ACK mọi HELLO (đủ cho kết nối lại; không còn spam 2s khi đã stream IMU).
 */

// 1: Serial gửi khung nhị phân (sync 4 byte + payload raw + CRC16 LE) cho PC/logger; 0: dòng chữ IMU,...
#define IMU_SERIAL_BINARY 1

// Khớp scale trên Slave (imu_logger / imu_serial_codec.py)
#define IMU_RAW_ACC_SCALE 512.0f
#define IMU_GYRO_FS_DPS 2000.0f
#define IMU_RAW_GYRO_SCALE (32767.0f / IMU_GYRO_FS_DPS)
#define IMU_TEMP_CENTI_SCALE 100.0f

static MacSeqTrack g_macSeqTrack[MASTER_MAC_SEQ_TRACK];
static uint8_t g_macSeqTrackCount = 0;

static MacSeqTrack *masterMacSeqFindOrAdd(const uint8_t *mac) {
  for (uint8_t i = 0; i < g_macSeqTrackCount; i++) {
    if (memcmp(g_macSeqTrack[i].mac, mac, 6) == 0) {
      return &g_macSeqTrack[i];
    }
  }
  if (g_macSeqTrackCount < MASTER_MAC_SEQ_TRACK) {
    MacSeqTrack *p = &g_macSeqTrack[g_macSeqTrackCount++];
    memcpy(p->mac, mac, 6);
    p->imu_inited = false;
    p->imu_last = 0U;
    p->vl53_inited = false;
    p->vl53_last = 0U;
    return p;
  }
  return NULL;
}

static bool masterSeqStreamShouldForward(bool *inited, uint32_t *last, uint32_t seq) {
  if (!*inited) {
    *inited = true;
    *last = seq;
    return true;
  }
  uint32_t delta = (seq - *last) & 0xFFFFFFFFU;
  if (delta == 0) {
    return false;
  }
  if (delta == 1) {
    *last = seq;
    return true;
  }
  if (delta > 0x7FFFFFFFU) {
    uint32_t backward = (*last - seq) & 0xFFFFFFFFU;
    if (backward <= MASTER_SEQ_REORDER_BACK_MAX) {
      return false;
    }
    *last = seq;
    return true;
  }
  if (delta > MASTER_SEQ_RESET_THRESHOLD) {
    *last = seq;
    return true;
  }
  *last = seq;
  return true;
}

static void masterSendImuLostForGap(MacSeqTrack *t, uint32_t seq_first, uint32_t nmiss, uint32_t node_id);
static void masterImuRetxMarkSample(const uint8_t *mac, uint32_t seq);
static void masterImuRetxTimeoutPoll(void);

static bool imuStreamAllowForwardAndRequestLost(MacSeqTrack *t, uint32_t node_id, uint32_t seq) {
  if (!t) {
    return true;
  }
  if (t->imu_inited) {
    uint32_t last = t->imu_last;
    uint32_t delta = (seq - last) & 0xFFFFFFFFU;
#if MASTER_IMU_LOST_REQUEST_ENABLE
    if (delta > 1U && delta < 0x7FFFFFFFU && delta <= MASTER_SEQ_RESET_THRESHOLD) {
      masterSendImuLostForGap(t, last + 1U, delta - 1U, node_id);
    }
#endif
  }
  return masterSeqStreamShouldForward(&t->imu_inited, &t->imu_last, seq);
}

static bool imuSerialShouldForwardWithNode(const uint8_t *mac, uint32_t node_id, uint32_t seq) {
  MacSeqTrack *t = masterMacSeqFindOrAdd(mac);
  return imuStreamAllowForwardAndRequestLost(t, node_id, seq);
}

static bool vl53SerialShouldForward(const uint8_t *mac, uint32_t seq) {
  MacSeqTrack *t = masterMacSeqFindOrAdd(mac);
  if (!t) {
    return true;
  }
  return masterSeqStreamShouldForward(&t->vl53_inited, &t->vl53_last, seq);
}

static inline int16_t masterFloatToI16(float v, float scale) {
  float x = roundf(v * scale);
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)x;
}

static void serialWriteImuRawFrameImpl(imu_packet_raw_t raw) {
  raw.master_micros_at_tx = nowUs();
  static const uint8_t kImuSync[] = {0xA5, 0x5A, 0xA5, 0x5A};
  Serial.write(kImuSync, sizeof(kImuSync));
  Serial.write((const uint8_t *)&raw, sizeof(raw));
  uint16_t crc = imu_serial_crc16_ccitt_false((const uint8_t *)&raw, sizeof(raw));
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)((crc >> 8) & 0xFF));
}

/** Gửi IMU ra PC; tuỳ MASTER_SERIAL_SEQ_FILTER có lọc trùng seq hay gửi hết. */
static void serialWriteImuRawFrame(const imu_packet_raw_t &raw) {
#if MASTER_SERIAL_SEQ_FILTER
  if (!imuSerialShouldForwardWithNode(raw.mac, raw.node_id, raw.sample_seq)) {
    return;
  }
#endif
  serialWriteImuRawFrameImpl(raw);
}

// UART raw VL53: sync khác IMU để PC tách khung; payload = vl53_packet_t (packed, khớp esp_now_time_sync_types.h)
static void serialWriteVl53RawFrame(const vl53_packet_t &v) {
#if MASTER_SERIAL_SEQ_FILTER
  if (!vl53SerialShouldForward(v.mac, v.sample_seq)) {
    return;
  }
#endif
  static const uint8_t kVl53Sync[] = {0xB5, 0x5B, 0xB5, 0x5B};
  Serial.write(kVl53Sync, sizeof(kVl53Sync));
  Serial.write((const uint8_t *)&v, sizeof(v));
  uint16_t crc = imu_serial_crc16_ccitt_false((const uint8_t *)&v, sizeof(v));
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)((crc >> 8) & 0xFF));
}

static imu_packet_raw_t floatImuPacketToRaw(const imu_packet_t &imu) {
  imu_packet_raw_t raw = {};
  memcpy(raw.type, "IMU_RAW\0", 8);
  memcpy(raw.mac, imu.mac, 6);
  raw.node_id = imu.node_id;
  raw.micros_timestamp = imu.micros_timestamp;
  raw.sample_seq = 0;
  raw.ax = masterFloatToI16(imu.ax, IMU_RAW_ACC_SCALE);
  raw.ay = masterFloatToI16(imu.ay, IMU_RAW_ACC_SCALE);
  raw.az = masterFloatToI16(imu.az, IMU_RAW_ACC_SCALE);
  raw.gx = masterFloatToI16(imu.gx, IMU_RAW_GYRO_SCALE);
  raw.gy = masterFloatToI16(imu.gy, IMU_RAW_GYRO_SCALE);
  raw.gz = masterFloatToI16(imu.gz, IMU_RAW_GYRO_SCALE);
  raw.mx = 0;
  raw.my = 0;
  raw.mz = 0;
  raw.temp_centi_c = 0;
  return raw;
}

static uint64_t masterImuBatchEndMicros(const imu_batch_packet_t &pkt) {
  uint64_t t = pkt.micros_t0;
  for (uint32_t i = 1; i < pkt.count; i++) {
    t += pkt.rest[i - 1].delta_us_from_prev;
  }
  return t;
}

static uint64_t masterVl53BatchEndMicros(const vl53_batch_packet_t &pkt) {
  uint64_t t = pkt.micros_t0;
  for (uint32_t i = 1; i < pkt.count; i++) {
    t += pkt.rest[i - 1].delta_us_from_prev;
  }
  return t;
}

// Slave gửi IMU_RAWB: nhiều mẫu trong một gói ESP-NOW → tách ra nhiều khung Serial (IMU_RAW + imu_packet_raw_t).
static bool tryExpandImuBatchToSerial(const uint8_t *data, int len, uint64_t rxUs) {
  if (len < (int)IMU_BATCH_HEADER_SIZE) {
    return false;
  }
  imu_batch_packet_t pkt = {};
  size_t copy_len = (size_t)len;
  if (copy_len > sizeof(pkt)) {
    copy_len = sizeof(pkt);
  }
  memcpy(&pkt, data, copy_len);
  if (memcmp(pkt.type, "IMU_RAWB", 8) != 0) {
    return false;
  }
  if (pkt.count < 1 || pkt.count > IMU_BATCH_MAX_SAMPLES) {
    return false;
  }
  int expected =
      (int)(IMU_BATCH_HEADER_SIZE +
            (size_t)(pkt.count - 1U) * sizeof(imu_batch_delta_t));
  if (len != expected) {
    return false;
  }

  imu_packet_raw_t raw = {};
  memcpy(raw.type, "IMU_RAW\0", 8);
  memcpy(raw.mac, pkt.mac, 6);
  raw.node_id = pkt.node_id;
  raw.micros_timestamp = pkt.micros_t0;
  raw.sample_seq = pkt.sample_seq0;
  raw.ax = pkt.ax0;
  raw.ay = pkt.ay0;
  raw.az = pkt.az0;
  raw.gx = pkt.gx0;
  raw.gy = pkt.gy0;
  raw.gz = pkt.gz0;
  raw.mx = pkt.mx0;
  raw.my = pkt.my0;
  raw.mz = pkt.mz0;
  raw.temp_centi_c = pkt.temp_centi_c;
  masterImuRetxMarkSample(raw.mac, raw.sample_seq);
  serialWriteImuRawFrame(raw);

  uint64_t t = pkt.micros_t0;
  for (uint32_t i = 1; i < pkt.count; i++) {
    t += pkt.rest[i - 1].delta_us_from_prev;
    raw.micros_timestamp = t;
    raw.sample_seq = pkt.sample_seq0 + i;
    raw.ax = pkt.rest[i - 1].ax;
    raw.ay = pkt.rest[i - 1].ay;
    raw.az = pkt.rest[i - 1].az;
    raw.gx = pkt.rest[i - 1].gx;
    raw.gy = pkt.rest[i - 1].gy;
    raw.gz = pkt.rest[i - 1].gz;
    raw.mx = pkt.rest[i - 1].mx;
    raw.my = pkt.rest[i - 1].my;
    raw.mz = pkt.rest[i - 1].mz;
    raw.temp_centi_c = pkt.temp_centi_c;
    masterImuRetxMarkSample(raw.mac, raw.sample_seq);
    serialWriteImuRawFrame(raw);
  }
  return true;
}

// Slave gửi IMU_RETXB: cùng layout IMU_RAWB — ghi Serial không lọc seq (gửi lại mẫu thiếu).
static bool tryExpandImuRtxBatchToSerial(const uint8_t *data, int len, uint64_t rxUs) {
  if (len < (int)IMU_BATCH_HEADER_SIZE) {
    return false;
  }
  imu_batch_packet_t pkt = {};
  size_t copy_len = (size_t)len;
  if (copy_len > sizeof(pkt)) {
    copy_len = sizeof(pkt);
  }
  memcpy(&pkt, data, copy_len);
  if (memcmp(pkt.type, "IMU_RTXB", 8) != 0) {
    return false;
  }
  if (pkt.count < 1 || pkt.count > IMU_BATCH_MAX_SAMPLES) {
    return false;
  }
  int expected =
      (int)(IMU_BATCH_HEADER_SIZE +
            (size_t)(pkt.count - 1U) * sizeof(imu_batch_delta_t));
  if (len != expected) {
    return false;
  }

  imu_packet_raw_t raw = {};
  memcpy(raw.type, "IMU_RAW\0", 8);
  memcpy(raw.mac, pkt.mac, 6);
  raw.node_id = pkt.node_id;
  raw.micros_timestamp = pkt.micros_t0;
  raw.sample_seq = pkt.sample_seq0;
  raw.ax = pkt.ax0;
  raw.ay = pkt.ay0;
  raw.az = pkt.az0;
  raw.gx = pkt.gx0;
  raw.gy = pkt.gy0;
  raw.gz = pkt.gz0;
  raw.mx = pkt.mx0;
  raw.my = pkt.my0;
  raw.mz = pkt.mz0;
  raw.temp_centi_c = pkt.temp_centi_c;
  masterImuRetxMarkSample(raw.mac, raw.sample_seq);
  serialWriteImuRawFrameImpl(raw);

  uint64_t t = pkt.micros_t0;
  for (uint32_t i = 1; i < pkt.count; i++) {
    t += pkt.rest[i - 1].delta_us_from_prev;
    raw.micros_timestamp = t;
    raw.sample_seq = pkt.sample_seq0 + i;
    raw.ax = pkt.rest[i - 1].ax;
    raw.ay = pkt.rest[i - 1].ay;
    raw.az = pkt.rest[i - 1].az;
    raw.gx = pkt.rest[i - 1].gx;
    raw.gy = pkt.rest[i - 1].gy;
    raw.gz = pkt.rest[i - 1].gz;
    raw.mx = pkt.rest[i - 1].mx;
    raw.my = pkt.rest[i - 1].my;
    raw.mz = pkt.rest[i - 1].mz;
    raw.temp_centi_c = pkt.temp_centi_c;
    masterImuRetxMarkSample(raw.mac, raw.sample_seq);
    serialWriteImuRawFrameImpl(raw);
  }
  Serial.printf(
      "IMU_LOG,LOST_RTX,%02X:%02X:%02X:%02X:%02X:%02X,%lu,%u,%lu\n",
      pkt.mac[0], pkt.mac[1], pkt.mac[2], pkt.mac[3], pkt.mac[4], pkt.mac[5],
      (unsigned long)pkt.node_id, (unsigned)pkt.count, (unsigned long)pkt.sample_seq0);
  return true;
}

// Slave gửi VL53RAWB: nhiều mẫu trong một gói → Serial từng mẫu (text hoặc khung B5+66 byte).
static bool tryExpandVl53BatchToSerial(const uint8_t *data, int len, uint64_t rxUs) {
  if (len < (int)VL53_BATCH_HEADER_SIZE) {
    return false;
  }
  vl53_batch_packet_t pkt = {};
  size_t copy_len = (size_t)len;
  if (copy_len > sizeof(pkt)) {
    copy_len = sizeof(pkt);
  }
  memcpy(&pkt, data, copy_len);
  if (memcmp(pkt.type, "VL53RAWB", 8) != 0) {
    return false;
  }
  if (pkt.count < 1 || pkt.count > VL53_BATCH_MAX_SAMPLES) {
    return false;
  }
  int expected =
      (int)(VL53_BATCH_HEADER_SIZE +
            (size_t)(pkt.count - 1U) * sizeof(vl53_batch_delta_t));
  if (len != expected) {
    return false;
  }

  uint64_t t = pkt.micros_t0;
  for (uint32_t i = 0; i < pkt.count; i++) {
    if (i > 0) {
      t += pkt.rest[i - 1].delta_us_from_prev;
    }
    uint32_t seq = pkt.sample_seq0 + i;
    const int16_t *zones =
        (i == 0) ? pkt.d0 : pkt.rest[i - 1].distance_raw;
#if IMU_SERIAL_BINARY
    vl53_packet_t v = {};
    memcpy(v.type, "VL53\0\0\0\0", 8);
    memcpy(v.mac, pkt.mac, 6);
    v.node_id = pkt.node_id;
    v.timestamp = (uint32_t)(t / 1000ULL);
    v.micros_timestamp = t;
    v.sample_seq = seq;
    memcpy(v.distance_raw, zones, sizeof(v.distance_raw));
    serialWriteVl53RawFrame(v);
#else
#if MASTER_SERIAL_SEQ_FILTER
    if (!vl53SerialShouldForward(pkt.mac, seq)) {
      continue;
    }
#endif
    Serial.printf("VL53,%02X:%02X:%02X:%02X:%02X:%02X,%lu,%llu,%lu",
                  pkt.mac[0], pkt.mac[1], pkt.mac[2], pkt.mac[3], pkt.mac[4],
                  pkt.mac[5], (unsigned long)(t / 1000ULL),
                  (unsigned long long)t, (unsigned long)seq);
    for (int z = 0; z < VL53_ZONE_COUNT; z++) {
      Serial.printf(",%d", (int)zones[z]);
    }
    Serial.println();
#endif
  }
  return true;
}

uint8_t slaveList[MAX_SLAVES][6];
/** Slot TDMA do Master cấp (thu hồi khi timeout / removeSlaveAt). */
static uint8_t slaveImuTxSlot[MAX_SLAVES];
static uint8_t slaveSyncSlot[MAX_SLAVES];
static bool slaveSlotAssigned[MAX_SLAVES];
static bool imuTxSlotPoolUsed[MASTER_IMU_TX_SLOT_COUNT];
static bool syncSlotPoolUsed[MASTER_SYNC_SLOT_COUNT];
uint32_t slaveLastSeenMs[MAX_SLAVES];
uint64_t slaveLastSyncUs[MAX_SLAVES];
uint32_t slaveLastSyncRttUs[MAX_SLAVES];
int8_t slaveLastRssi[MAX_SLAVES];
bool slaveRssiValid[MAX_SLAVES];
int8_t slaveRssiWin[MAX_SLAVES][SLAVE_RSSI_MA_N];
uint8_t slaveRssiWinN[MAX_SLAVES];
uint8_t slaveRssiWinPos[MAX_SLAVES];
uint8_t slaveLastBatPct[MAX_SLAVES];
bool slaveBatValid[MAX_SLAVES];
/** Ước lượng điện áp cell (V); đồng bộ logic SYNC/GUI serial. */
float slaveLastVcell[MAX_SLAVES];
int slaveCount = 0;

/** Điện áp tại chân ADC sau cầu (chia 2), từ raw 12-bit. */
static float adcRawToVadcPin(uint16_t raw12) {
  return (float)raw12 * (VBAT_ADC_REF_V / 4095.0f);
}

/** Điện áp cell = Vadc × (1/scale) = Vadc × VBAT_VCELL_PER_VADC. */
static float adcRawToVcellEst(uint16_t raw12) {
  return adcRawToVadcPin(raw12) * VBAT_VCELL_PER_VADC;
}

/** % pin theo điện áp cell 3.5V…4.1V (100% tại 4.1V). */
static uint8_t adcRawToBatteryPct(uint16_t raw12) {
  const float Vcell = adcRawToVcellEst(raw12);
  if (Vcell >= VBAT_CELL_FULL_V) {
    return 100;
  }
  if (Vcell <= VBAT_CELL_EMPTY_V) {
    return 0;
  }
  float pct =
      (Vcell - VBAT_CELL_EMPTY_V) / (VBAT_CELL_FULL_V - VBAT_CELL_EMPTY_V) * 100.0f;
  if (pct > 100.0f) {
    pct = 100.0f;
  }
  if (pct < 0.0f) {
    pct = 0.0f;
  }
  return (uint8_t)(pct + 0.5f);
}

static int slaveIndexForMac(const uint8_t *mac) {
  for (int i = 0; i < slaveCount; i++) {
    if (memcmp(mac, slaveList[i], 6) == 0) {
      return i;
    }
  }
  return -1;
}

static int masterAllocImuTxSlot() {
  for (uint8_t i = 0; i < MASTER_IMU_TX_SLOT_COUNT; i++) {
    if (!imuTxSlotPoolUsed[i]) {
      imuTxSlotPoolUsed[i] = true;
      return (int)i;
    }
  }
  return -1;
}

static int masterAllocSyncSlot() {
  for (uint8_t i = 0; i < MASTER_SYNC_SLOT_COUNT; i++) {
    if (!syncSlotPoolUsed[i]) {
      syncSlotPoolUsed[i] = true;
      return (int)i;
    }
  }
  return -1;
}

static void masterFreeImuTxSlot(uint8_t slot) {
  if (slot < MASTER_IMU_TX_SLOT_COUNT) {
    imuTxSlotPoolUsed[slot] = false;
  }
}

static void masterFreeSyncSlot(uint8_t slot) {
  if (slot < MASTER_SYNC_SLOT_COUNT) {
    syncSlotPoolUsed[slot] = false;
  }
}

static void masterReleaseSlaveSlots(int si) {
  if (si < 0 || si >= slaveCount || !slaveSlotAssigned[si]) {
    return;
  }
  masterFreeImuTxSlot(slaveImuTxSlot[si]);
  masterFreeSyncSlot(slaveSyncSlot[si]);
  slaveImuTxSlot[si] = MASTER_SLOT_UNASSIGNED;
  slaveSyncSlot[si] = MASTER_SLOT_UNASSIGNED;
  slaveSlotAssigned[si] = false;
}

static bool masterAssignSlaveSlots(int si) {
  if (si < 0 || si >= MAX_SLAVES) {
    return false;
  }
  int imu = masterAllocImuTxSlot();
  int sync = masterAllocSyncSlot();
  if (imu < 0 || sync < 0) {
    if (imu >= 0) {
      masterFreeImuTxSlot((uint8_t)imu);
    }
    if (sync >= 0) {
      masterFreeSyncSlot((uint8_t)sync);
    }
    Serial.println("❌ Het slot TDMA (IMU hoac SYNC)");
    return false;
  }
  slaveImuTxSlot[si] = (uint8_t)imu;
  slaveSyncSlot[si] = (uint8_t)sync;
  slaveSlotAssigned[si] = true;
  Serial.printf("   Slot cap phat: IMU=%u/%u, SYNC=%u/%u\n",
                (unsigned)slaveImuTxSlot[si], (unsigned)MASTER_IMU_TX_SLOT_COUNT,
                (unsigned)slaveSyncSlot[si], (unsigned)MASTER_SYNC_SLOT_COUNT);
  return true;
}

static void masterFillPacketSlots(packet_t *p, int si) {
  if (!p) {
    return;
  }
  p->imu_tx_slot = MASTER_SLOT_UNASSIGNED;
  p->sync_slot = MASTER_SLOT_UNASSIGNED;
  p->imu_tx_slot_count = MASTER_IMU_TX_SLOT_COUNT;
  p->slot_flags = 0;
  if (si < 0 || si >= slaveCount || !slaveSlotAssigned[si]) {
    return;
  }
  p->imu_tx_slot = slaveImuTxSlot[si];
  p->sync_slot = slaveSyncSlot[si];
  p->slot_flags = PACKET_T_SLOT_FLAG_VALID;
}

static void slaveRssiPushSample(int si, int8_t r) {
  if (si < 0 || si >= slaveCount) {
    return;
  }
  if (slaveRssiWinN[si] < SLAVE_RSSI_MA_N) {
    slaveRssiWin[si][slaveRssiWinN[si]++] = r;
  } else {
    slaveRssiWin[si][slaveRssiWinPos[si]] = r;
    slaveRssiWinPos[si] =
        (uint8_t)((slaveRssiWinPos[si] + 1U) % (uint8_t)SLAVE_RSSI_MA_N);
  }
}

/** Cập nhật slaveLastRssi / slaveRssiValid từ cửa sổ MA (log Serial định kỳ). */
static void updateSlaveRssiDisplayFromWindows(void) {
  for (int i = 0; i < slaveCount; i++) {
    const uint8_t n = slaveRssiWinN[i];
    if (n == 0U) {
      slaveRssiValid[i] = false;
      continue;
    }
    int32_t sum = 0;
    if (n < (uint8_t)SLAVE_RSSI_MA_N) {
      for (uint8_t k = 0; k < n; k++) {
        sum += (int32_t)slaveRssiWin[i][k];
      }
      slaveLastRssi[i] = (int8_t)((sum + (int32_t)n / 2) / (int32_t)n);
    } else {
      for (uint8_t k = 0; k < (uint8_t)SLAVE_RSSI_MA_N; k++) {
        sum += (int32_t)slaveRssiWin[i][k];
      }
      slaveLastRssi[i] = (int8_t)((sum + 5) / 10);
    }
    slaveRssiValid[i] = true;
  }
}

#if MASTER_WIFI_TX_POWER_ADAPTIVE
static int8_t s_masterWifiTxQdb = 0;
static bool s_masterWifiTxBoosted = false;
static uint32_t s_masterWifiTxLastAdjMs = 0;

static int8_t masterWifiClampTxQdb(int8_t qdb) {
  if (qdb < 8) {
    return 8;
  }
  if (qdb > 84) {
    return 84;
  }
  return qdb;
}

static bool masterWifiApplyTxPowerQdb(int8_t qdb) {
  qdb = masterWifiClampTxQdb(qdb);
  if (qdb == s_masterWifiTxQdb) {
    return false;
  }
  const esp_err_t e = esp_wifi_set_max_tx_power(qdb);
  if (e != ESP_OK) {
    return false;
  }
  s_masterWifiTxQdb = qdb;
  return true;
}

static void masterWifiTxPowerInit(void) {
  s_masterWifiTxBoosted = false;
  if (masterWifiApplyTxPowerQdb((int8_t)MASTER_WIFI_TX_POWER_NORM_QDB)) {
    Serial.printf("[LINK] Master TX init: quarter-dBm=%d (~%.1f dBm)\n",
                  (int)s_masterWifiTxQdb, (double)s_masterWifiTxQdb * 0.25);
  }
}

static void masterWifiTxPowerUpdateFromRssi(int8_t rssiDb) {
  const uint32_t now = millis();
  if (s_masterWifiTxLastAdjMs != 0U &&
      (uint32_t)(now - s_masterWifiTxLastAdjMs) < MASTER_WIFI_TX_POWER_ADJ_INTERVAL_MS) {
    return;
  }

  bool wantBoost;
  if (rssiDb < (int8_t)MASTER_WIFI_TX_RSSI_BOOST_DB) {
    wantBoost = true;
  } else if (rssiDb >= (int8_t)MASTER_WIFI_TX_RSSI_RESTORE_DB) {
    wantBoost = false;
  } else {
    return;
  }

  if (wantBoost == s_masterWifiTxBoosted) {
    return;
  }

  const int8_t target = wantBoost ? (int8_t)MASTER_WIFI_TX_POWER_BOOST_QDB
                                  : (int8_t)MASTER_WIFI_TX_POWER_NORM_QDB;
  if (!masterWifiApplyTxPowerQdb(target)) {
    return;
  }
  s_masterWifiTxBoosted = wantBoost;
  s_masterWifiTxLastAdjMs = now;
  Serial.printf("[LINK] Master TX %s: quarter-dBm=%d (~%.1f dBm), min RSSI=%d dBm\n",
                wantBoost ? "BOOST" : "NORM", (int)target, (double)target * 0.25,
                (int)rssiDb);
}

static void masterWifiTxPowerRecomputeFromSlaves(void) {
  if (slaveCount <= 0) {
    return;
  }
  updateSlaveRssiDisplayFromWindows();
  int8_t minRssi = 127;
  bool any = false;
  for (int i = 0; i < slaveCount; i++) {
    if (!slaveRssiValid[i]) {
      continue;
    }
    any = true;
    if (slaveLastRssi[i] < minRssi) {
      minRssi = slaveLastRssi[i];
    }
  }
  if (any) {
    masterWifiTxPowerUpdateFromRssi(minRssi);
  }
}
#endif

static void noteSlaveRssiIfKnown(const esp_now_recv_info *info, const uint8_t *src) {
  if (!info || !info->rx_ctrl) {
    return;
  }
  int si = slaveIndexForMac(src);
  if (si < 0) {
    return;
  }
  slaveRssiPushSample(si, (int8_t)info->rx_ctrl->rssi);
#if MASTER_WIFI_TX_POWER_ADAPTIVE
  masterWifiTxPowerRecomputeFromSlaves();
#endif
}

unsigned long lastPrint = 0;
int masterTimezone = 7; // GMT+7

char serialLineBuf[192];
size_t serialLineLen = 0;

uint64_t nowUs() {
  return (uint64_t)esp_timer_get_time();
}

uint32_t nowMs() {
  return (uint32_t)(nowUs() / 1000ULL);
}

void printMAC(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

String macToString(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

bool parseMacString(const char *text, uint8_t *mac) {
  unsigned int values[6];
  if (sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) {
    mac[i] = (uint8_t)values[i];
  }
  return true;
}

void printSlaveList() {
  Serial.println("\n📋 Danh sách slave hiện tại:");
  if (slaveCount == 0) {
    Serial.println("   (Chưa có thiết bị nào)");
    return;
  }

  for (int i = 0; i < slaveCount; i++) {
    Serial.printf("   #%d: ", i + 1);
    printMAC(slaveList[i]);
    if (slaveSlotAssigned[i]) {
      Serial.printf("  IMU_slot=%u  SYNC_slot=%u",
                    (unsigned)slaveImuTxSlot[i], (unsigned)slaveSyncSlot[i]);
    }
    Serial.println();
  }

  Serial.printf("→ Tổng số slave: %d\n", slaveCount);
  Serial.println("----------------------------------");
}

static void logSlaveRssiSummary(void) {
  if (slaveCount <= 0) {
    return;
  }
  Serial.println("--- RSSI/BAT (MA) ---");
  for (int i = 0; i < slaveCount; i++) {
    Serial.printf("  #%d %02X:%02X:%02X:%02X:%02X:%02X ", i + 1,
                  slaveList[i][0], slaveList[i][1], slaveList[i][2],
                  slaveList[i][3], slaveList[i][4], slaveList[i][5]);
    if (slaveRssiValid[i]) {
      Serial.printf("RSSI=%d dBm ", (int)slaveLastRssi[i]);
    } else {
      Serial.print("RSSI=-- ");
    }
    if (slaveBatValid[i]) {
      Serial.printf("BAT=%u%%\n", (unsigned)slaveLastBatPct[i]);
    } else {
      Serial.println("BAT=--");
    }
  }
}

void printMasterTime() {
  uint32_t currentMs = nowMs();

  int hours = (currentMs / 3600000UL) % 24;
  int minutes = (currentMs / 60000UL) % 60;
  int seconds = (currentMs / 1000UL) % 60;
  int milliseconds = currentMs % 1000UL;

  Serial.printf("🕐 Master Time: %02d:%02d:%02d.%03d\n",
                hours, minutes, seconds, milliseconds);
}

bool isSlaveKnown(const uint8_t *mac) {
  for (int i = 0; i < slaveCount; i++) {
    if (memcmp(mac, slaveList[i], 6) == 0) return true;
  }
  return false;
}

static unsigned long nowMsForSeen() {
  unsigned long t = millis();
  return (t == 0) ? 1UL : t;
}

static void markSlaveSeen(const uint8_t *mac) {
  for (int i = 0; i < slaveCount; i++) {
    if (memcmp(mac, slaveList[i], 6) == 0) {
      slaveLastSeenMs[i] = nowMsForSeen();
      return;
    }
  }
}

static void removeSlaveAt(int idx) {
  if (idx < 0 || idx >= slaveCount) {
    return;
  }
  esp_err_t del = esp_now_del_peer(slaveList[idx]);
  (void)del;
  Serial.print("⏱ Slave timeout (>");
  Serial.print((unsigned long)(SLAVE_RX_TIMEOUT_MS / 1000UL));
  Serial.print("s), removed: ");
  printMAC(slaveList[idx]);
  if (slaveSlotAssigned[idx]) {
    Serial.printf(" (thu hoi IMU slot %u, SYNC slot %u)",
                  (unsigned)slaveImuTxSlot[idx], (unsigned)slaveSyncSlot[idx]);
  }
  Serial.println();
  masterReleaseSlaveSlots(idx);
  for (int j = idx; j < slaveCount - 1; j++) {
    memcpy(slaveList[j], slaveList[j + 1], 6);
    slaveImuTxSlot[j] = slaveImuTxSlot[j + 1];
    slaveSyncSlot[j] = slaveSyncSlot[j + 1];
    slaveSlotAssigned[j] = slaveSlotAssigned[j + 1];
    slaveLastSeenMs[j] = slaveLastSeenMs[j + 1];
    slaveLastSyncUs[j] = slaveLastSyncUs[j + 1];
    slaveLastSyncRttUs[j] = slaveLastSyncRttUs[j + 1];
    slaveLastRssi[j] = slaveLastRssi[j + 1];
    slaveRssiValid[j] = slaveRssiValid[j + 1];
    slaveRssiWinN[j] = slaveRssiWinN[j + 1];
    slaveRssiWinPos[j] = slaveRssiWinPos[j + 1];
    memcpy(slaveRssiWin[j], slaveRssiWin[j + 1], sizeof(slaveRssiWin[0]));
    slaveLastBatPct[j] = slaveLastBatPct[j + 1];
    slaveBatValid[j] = slaveBatValid[j + 1];
    slaveLastVcell[j] = slaveLastVcell[j + 1];
  }
  slaveCount--;
}

static void checkSlaveTimeouts() {
  unsigned long now = millis();
  if (now == 0) {
    now = 1;
  }
  for (int i = slaveCount - 1; i >= 0; i--) {
    if (slaveLastSeenMs[i] == 0) {
      slaveLastSeenMs[i] = nowMsForSeen();
      continue;
    }
    if ((unsigned long)(now - slaveLastSeenMs[i]) >= SLAVE_RX_TIMEOUT_MS) {
      removeSlaveAt(i);
    }
  }
}

bool ensureSlavePeer(const uint8_t *mac) {
  if (isSlaveKnown(mac)) {
    markSlaveSeen(mac);
    return true;
  }
  if (slaveCount >= MAX_SLAVES) return false;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t addResult = esp_now_add_peer(&peerInfo);
  if (addResult == ESP_OK) {
    const int si = slaveCount;
    if (!masterAssignSlaveSlots(si)) {
      return false;
    }
    memcpy(slaveList[slaveCount], mac, 6);
    slaveLastSeenMs[slaveCount] = nowMsForSeen();
    slaveLastSyncUs[slaveCount] = 0ULL;
    slaveLastSyncRttUs[slaveCount] = 0U;
    slaveRssiValid[slaveCount] = false;
    slaveRssiWinN[slaveCount] = 0U;
    slaveRssiWinPos[slaveCount] = 0U;
    slaveBatValid[slaveCount] = false;
    slaveLastBatPct[slaveCount] = 0;
    slaveLastVcell[slaveCount] = 0.0f;
    slaveCount++;
    Serial.printf("✅ Đã thêm slave #%d: ", slaveCount);
    printMAC(mac);
    Serial.println();
    return true;
  } else if (addResult == ESP_ERR_ESPNOW_EXIST) {
    const int si = slaveCount;
    if (!masterAssignSlaveSlots(si)) {
      return false;
    }
    memcpy(slaveList[slaveCount], mac, 6);
    slaveLastSeenMs[slaveCount] = nowMsForSeen();
    slaveLastSyncUs[slaveCount] = 0ULL;
    slaveLastSyncRttUs[slaveCount] = 0U;
    slaveRssiValid[slaveCount] = false;
    slaveRssiWinN[slaveCount] = 0U;
    slaveRssiWinPos[slaveCount] = 0U;
    slaveBatValid[slaveCount] = false;
    slaveLastBatPct[slaveCount] = 0;
    slaveLastVcell[slaveCount] = 0.0f;
    slaveCount++;
    Serial.printf("ℹ️ Peer đã tồn tại, thêm vào danh sách slave #%d\n", slaveCount);
    return true;
  } else {
    Serial.printf("❌ Lỗi thêm slave peer (%d)\n", addResult);
    return false;
  }
}

/** Theo dõi LOST theo từng chunk (≤32 seq); 1s timeout trên master, tối đa 3 lần gửi. */
#define MASTER_IMU_RETX_PENDING_MAX 16
#define MASTER_IMU_RETX_TMO_MS 1000U
#define MASTER_IMU_RETX_MAX_SENDS 3U

typedef struct {
  bool active;
  uint8_t mac[6];
  uint32_t node_id;
  uint32_t seq0;
  uint32_t n;
  uint32_t got_mask;
  uint32_t last_send_ms;
  uint8_t sends_done;
} MasterImuRetxPending;

static MasterImuRetxPending g_masterImuRetxPend[MASTER_IMU_RETX_PENDING_MAX];

static uint32_t masterImuRetxChunkFullMask(uint32_t n) {
  if (n == 0U) {
    return 0U;
  }
  if (n >= 32U) {
    return 0xFFFFFFFFU;
  }
  return (1UL << n) - 1U;
}

static void masterImuRetxMarkSample(const uint8_t *mac, uint32_t seq) {
  for (int i = 0; i < MASTER_IMU_RETX_PENDING_MAX; i++) {
    MasterImuRetxPending *p = &g_masterImuRetxPend[i];
    if (!p->active) {
      continue;
    }
    if (memcmp(p->mac, mac, 6) != 0) {
      continue;
    }
    if (p->n == 0U || p->n > 32U) {
      continue;
    }
    if (seq < p->seq0) {
      continue;
    }
    uint32_t bit = seq - p->seq0;
    if (bit >= p->n) {
      continue;
    }
    p->got_mask |= (1U << bit);
  }
}

static void masterImuRetxSendLostPacket(const uint8_t *mac, uint32_t node_id, uint32_t seq_first,
                                       uint32_t seq_count) {
  imu_lost_request_t rq = {};
  memcpy(rq.type, "IMU_LOST", 8);
  memcpy(rq.target_mac, mac, 6);
  rq.node_id = node_id;
  rq.seq_first = seq_first;
  rq.seq_count = seq_count;
  (void)esp_now_send(mac, (uint8_t *)&rq, sizeof(rq));
  Serial.printf(
      "IMU_LOG,LOST_REQ,%02X:%02X:%02X:%02X:%02X:%02X,%lu,%lu,%lu\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
      (unsigned long)node_id, (unsigned long)seq_first, (unsigned long)seq_count);
}

static int masterImuRetxFindDup(const uint8_t *mac, uint32_t seq0, uint32_t n) {
  for (int i = 0; i < MASTER_IMU_RETX_PENDING_MAX; i++) {
    if (!g_masterImuRetxPend[i].active) {
      continue;
    }
    if (memcmp(g_masterImuRetxPend[i].mac, mac, 6) != 0) {
      continue;
    }
    if (g_masterImuRetxPend[i].seq0 == seq0 && g_masterImuRetxPend[i].n == n) {
      return i;
    }
  }
  return -1;
}

static int masterImuRetxFindFree(void) {
  for (int i = 0; i < MASTER_IMU_RETX_PENDING_MAX; i++) {
    if (!g_masterImuRetxPend[i].active) {
      return i;
    }
  }
  return -1;
}

static void masterImuRetxRegisterChunk(const uint8_t *mac, uint32_t node_id, uint32_t seq0, uint32_t n) {
  if (n == 0U || n > (uint32_t)MASTER_IMU_LOST_MAX_CHUNK) {
    return;
  }
  (void)ensureSlavePeer(mac);
  if (!esp_now_is_peer_exist(mac)) {
    return;
  }
  if (masterImuRetxFindDup(mac, seq0, n) >= 0) {
    return;
  }
  int idx = masterImuRetxFindFree();
  if (idx < 0) {
    Serial.println("IMU_LOG,RETX_PEND_FULL");
    return;
  }
  MasterImuRetxPending *p = &g_masterImuRetxPend[idx];
  memset(p, 0, sizeof(*p));
  p->active = true;
  memcpy(p->mac, mac, 6);
  p->node_id = node_id;
  p->seq0 = seq0;
  p->n = n;
  p->got_mask = 0U;
  p->last_send_ms = millis();
  p->sends_done = 1;
  masterImuRetxSendLostPacket(mac, node_id, seq0, n);
}

static void masterImuRetxTimeoutPoll(void) {
  const uint32_t now = millis();
  for (int i = 0; i < MASTER_IMU_RETX_PENDING_MAX; i++) {
    MasterImuRetxPending *p = &g_masterImuRetxPend[i];
    if (!p->active) {
      continue;
    }
    const uint32_t need = masterImuRetxChunkFullMask(p->n);
    if ((p->got_mask & need) == need) {
      p->active = false;
      continue;
    }
    if ((uint32_t)(now - p->last_send_ms) < MASTER_IMU_RETX_TMO_MS) {
      continue;
    }
    if (p->sends_done >= MASTER_IMU_RETX_MAX_SENDS) {
      Serial.printf(
          "IMU_LOG,RETX_DROP,%02X:%02X:%02X:%02X:%02X:%02X,%lu,%lu,%lu\n",
          p->mac[0], p->mac[1], p->mac[2], p->mac[3], p->mac[4], p->mac[5],
          (unsigned long)p->node_id, (unsigned long)p->seq0, (unsigned long)p->n);
      p->active = false;
      continue;
    }
    p->sends_done++;
    p->last_send_ms = now;
    masterImuRetxSendLostPacket(p->mac, p->node_id, p->seq0, p->n);
  }
}

static void masterSendImuLostForGap(MacSeqTrack *t, uint32_t seq_first, uint32_t nmiss, uint32_t node_id) {
  if (t == NULL || nmiss == 0U) {
    return;
  }
  for (uint32_t off = 0; off < nmiss; off += (uint32_t)MASTER_IMU_LOST_MAX_CHUNK) {
    uint32_t c = nmiss - off;
    if (c > (uint32_t)MASTER_IMU_LOST_MAX_CHUNK) {
      c = (uint32_t)MASTER_IMU_LOST_MAX_CHUNK;
    }
    masterImuRetxRegisterChunk(t->mac, node_id, seq_first + off, c);
  }
}

void sendAck(const uint8_t *dst) {
  packet_t ackPacket = {};
  strcpy(ackPacket.type, "ACK");
  WiFi.macAddress(ackPacket.mac);
  ackPacket.node_id = 0;
  ackPacket.timestamp = nowMs();
  ackPacket.micros_timestamp = nowUs();
  ackPacket.timezone_offset = masterTimezone;
  ackPacket.request_time = 0;
  ackPacket.response_time = 0;
  masterFillPacketSlots(&ackPacket, slaveIndexForMac(dst));

  esp_err_t result = esp_now_send(dst, (uint8_t *)&ackPacket, sizeof(ackPacket));
  if (result == ESP_OK) {
    Serial.println("✅ Đã gửi ACK xác nhận kết nối");
  } else {
    Serial.printf("❌ Lỗi gửi ACK (%d)\n", result);
  }
}

static void masterNoteSlaveBatteryFromSync(const uint8_t *src,
                                           const esp_now_recv_info *info,
                                           uint16_t adcRaw) {
  int si = slaveIndexForMac(src);
  if (si >= 0) {
    float Vadc = adcRawToVadcPin(adcRaw);
    float Vcell = adcRawToVcellEst(adcRaw);
    uint8_t pct = adcRawToBatteryPct(adcRaw);
    slaveLastVcell[si] = Vcell;
    slaveLastBatPct[si] = pct;
    slaveBatValid[si] = true;
    Serial.printf(
        "SYNC,BAT,raw=%u,Vadc=%.3f,Vcell=%.3f(x%.1f),pct=%u "
        "(cell %.2f-%.2fV)\n",
        (unsigned)adcRaw, (double)Vadc, (double)Vcell,
        (double)VBAT_VCELL_PER_VADC, (unsigned)pct,
        (double)VBAT_CELL_EMPTY_V, (double)VBAT_CELL_FULL_V);
    if (info && info->rx_ctrl) {
      int rssi_gui = (int)info->rx_ctrl->rssi;
      Serial.printf("SYNC,GUI,mac=%s,pct=%u,rssi_dbm=%d\n",
                    macToString(src).c_str(), (unsigned)pct, rssi_gui);
    }
  } else {
    Serial.printf("SYNC,BAT,raw=%u (no slave idx)\n", (unsigned)adcRaw);
  }
}

void sendTimeResponse(const uint8_t *dst, const packet_t &syncPkt, uint64_t masterReceiveUs) {
  packet_t timePacket = {};
  strcpy(timePacket.type, "TIME");
  WiFi.macAddress(timePacket.mac);
  timePacket.node_id = 0;

  // t2: master nhận SYNC
  timePacket.timestamp = (uint32_t)(masterReceiveUs / 1000ULL);
  timePacket.micros_timestamp = masterReceiveUs;
  timePacket.timezone_offset = masterTimezone;

  // giữ nguyên t1 từ slave
  timePacket.request_time = syncPkt.request_time;

  // t3: master gửi TIME
  uint64_t masterSendUs = nowUs();
  timePacket.response_time = masterSendUs;
  masterFillPacketSlots(&timePacket, slaveIndexForMac(dst));

  esp_err_t result = esp_now_send(dst, (uint8_t *)&timePacket, sizeof(timePacket));

  if (result == ESP_OK) {
    int si = slaveIndexForMac(dst);
    if (si >= 0) {
      slaveLastSyncUs[si] = masterSendUs;
      /* RTT thật (µs) do slave đo ở vòng trước, gửi kèm gói SYNC tới; không có t4 trên master. */
      if (syncPkt.last_sync_rtt_us != 0U) {
        slaveLastSyncRttUs[si] = syncPkt.last_sync_rtt_us;
      } else {
        slaveLastSyncRttUs[si] =
            (uint32_t)(masterSendUs - masterReceiveUs); /* fallback: chỉ t3−t2 (xử lý master) */
      }
    }
    Serial.printf("📤 TIME sent | t2=%llu us | t3=%llu us | proc(t3-t2)=%llu us | "
                  "slave_rtt_in_SYNC=%lu us\n",
                  (unsigned long long)masterReceiveUs,
                  (unsigned long long)masterSendUs,
                  (unsigned long long)(masterSendUs - masterReceiveUs),
                  (unsigned long)syncPkt.last_sync_rtt_us);
  } else {
    Serial.printf("❌ Lỗi gửi TIME (%d)\n", result);
  }
}

void sendCalibrationToSlave(const uint8_t *dst,
                            float bx, float by, float bz,
                            float sx, float sy, float sz,
                            float globalScale,
                            float gxb, float gyb, float gzb) {
  if (!ensureSlavePeer(dst)) {
    Serial.printf("CALIB_TX,%s,FAIL,peer\n", macToString(dst).c_str());
    return;
  }

  accel_calib_packet_t pkt = {};
  strcpy(pkt.type, "CALIB");
  WiFi.macAddress(pkt.mac);
  pkt.node_id = 0;
  pkt.timestamp = nowMs();
  pkt.micros_timestamp = nowUs();
  pkt.bias[0] = bx;
  pkt.bias[1] = by;
  pkt.bias[2] = bz;
  pkt.scale[0] = sx;
  pkt.scale[1] = sy;
  pkt.scale[2] = sz;
  pkt.global_scale = globalScale;
  pkt.save_to_nvs = 1;
  pkt.enabled = 1;
  pkt.gyro_bias[0] = gxb;
  pkt.gyro_bias[1] = gyb;
  pkt.gyro_bias[2] = gzb;

  esp_err_t result = esp_now_send(dst, (uint8_t *)&pkt, sizeof(pkt));
  if (result == ESP_OK) {
    Serial.printf("CALIB_TX,%s,OK,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                  macToString(dst).c_str(),
                  bx, by, bz, sx, sy, sz, globalScale, gxb, gyb, gzb);
  } else {
    Serial.printf("CALIB_TX,%s,FAIL,%d\n", macToString(dst).c_str(), result);
  }
}

/** Master → Slave: yêu cầu gửi lại bias/scale/global_scale hiện tại (CALREP). */
void requestCalibrationFromSlave(const uint8_t *dst) {
  if (!ensureSlavePeer(dst)) {
    Serial.printf("CALGET_TX,%s,FAIL,peer\n", macToString(dst).c_str());
    return;
  }

  accel_calib_packet_t pkt = {};
  strcpy(pkt.type, "CALGET");
  WiFi.macAddress(pkt.mac);
  pkt.node_id = 0;
  pkt.timestamp = nowMs();
  pkt.micros_timestamp = nowUs();
  pkt.bias[0] = pkt.bias[1] = pkt.bias[2] = 0.0f;
  pkt.scale[0] = pkt.scale[1] = pkt.scale[2] = 0.0f;
  pkt.global_scale = 0.0f;
  pkt.save_to_nvs = 0;
  pkt.enabled = 0;

  esp_err_t result = esp_now_send(dst, (uint8_t *)&pkt, sizeof(pkt));
  if (result == ESP_OK) {
    Serial.printf("CALGET_TX,%s,OK\n", macToString(dst).c_str());
  } else {
    Serial.printf("CALGET_TX,%s,FAIL,%d\n", macToString(dst).c_str(), result);
  }
}

void clearCalibrationOnSlave(const uint8_t *dst) {
  if (!ensureSlavePeer(dst)) {
    Serial.printf("CALIB_CLEAR_TX,%s,FAIL,peer\n", macToString(dst).c_str());
    return;
  }

  accel_calib_packet_t pkt = {};
  strcpy(pkt.type, "CALIB");
  WiFi.macAddress(pkt.mac);
  pkt.node_id = 0;
  pkt.timestamp = nowMs();
  pkt.micros_timestamp = nowUs();
  pkt.bias[0] = 0.0f;
  pkt.bias[1] = 0.0f;
  pkt.bias[2] = 0.0f;
  pkt.scale[0] = 1.0f;
  pkt.scale[1] = 1.0f;
  pkt.scale[2] = 1.0f;
  pkt.global_scale = 1.0f;
  pkt.save_to_nvs = 1;
  pkt.enabled = 0;
  pkt.gyro_bias[0] = 0.0f;
  pkt.gyro_bias[1] = 0.0f;
  pkt.gyro_bias[2] = 0.0f;

  esp_err_t result = esp_now_send(dst, (uint8_t *)&pkt, sizeof(pkt));
  if (result == ESP_OK) {
    Serial.printf("CALIB_CLEAR_TX,%s,OK\n", macToString(dst).c_str());
  } else {
    Serial.printf("CALIB_CLEAR_TX,%s,FAIL,%d\n", macToString(dst).c_str(), result);
  }
}

void processCalibrationCommand(const char *line) {
  char command[8] = {0};
  char macText[24] = {0};
  float bx, by, bz, sx, sy, sz, globalScale;
  float gxb = 0.0f, gyb = 0.0f, gzb = 0.0f;
  int parsed = sscanf(line, "%7[^,],%23[^,],%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                      command, macText,
                      &bx, &by, &bz,
                      &sx, &sy, &sz,
                      &globalScale,
                      &gxb, &gyb, &gzb);
  if ((parsed != 9 && parsed != 12) || strcmp(command, "CALIB") != 0) {
    Serial.println("CALIB_PARSE,FAIL,format");
    return;
  }

  uint8_t targetMac[6];
  if (!parseMacString(macText, targetMac)) {
    Serial.println("CALIB_PARSE,FAIL,mac");
    return;
  }

  sendCalibrationToSlave(targetMac, bx, by, bz, sx, sy, sz, globalScale, gxb, gyb, gzb);
}

void processClearCalibrationCommand(const char *line) {
  char command[8] = {0};
  char macText[24] = {0};
  int parsed = sscanf(line, "%7[^,],%23[^,\r\n]", command, macText);
  if (parsed != 2 || strcmp(command, "CLRCAL") != 0) {
    Serial.println("CALIB_CLEAR_PARSE,FAIL,format");
    return;
  }

  uint8_t targetMac[6];
  if (!parseMacString(macText, targetMac)) {
    Serial.println("CALIB_CLEAR_PARSE,FAIL,mac");
    return;
  }

  clearCalibrationOnSlave(targetMac);
}

void processGetCalibrationCommand(const char *line) {
  char command[12] = {0};
  char macText[24] = {0};
  int parsed = sscanf(line, "%11[^,],%23[^,\r\n]", command, macText);
  if (parsed != 2 || strcmp(command, "GETCALIB") != 0) {
    Serial.println("CALGET_PARSE,FAIL,format");
    return;
  }

  uint8_t targetMac[6];
  if (!parseMacString(macText, targetMac)) {
    Serial.println("CALGET_PARSE,FAIL,mac");
    return;
  }

  requestCalibrationFromSlave(targetMac);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      serialLineBuf[serialLineLen] = '\0';
      if (serialLineLen > 0 && strncmp(serialLineBuf, "CALIB,", 6) == 0) {
        processCalibrationCommand(serialLineBuf);
      } else if (serialLineLen > 0 && strncmp(serialLineBuf, "CLRCAL,", 7) == 0) {
        processClearCalibrationCommand(serialLineBuf);
      } else if (serialLineLen > 0 && strncmp(serialLineBuf, "GETCALIB,", 9) == 0) {
        processGetCalibrationCommand(serialLineBuf);
      }
      serialLineLen = 0;
      continue;
    }
    if (serialLineLen + 1 < sizeof(serialLineBuf)) {
      serialLineBuf[serialLineLen++] = ch;
    } else {
      serialLineLen = 0;
      Serial.println("CALIB_PARSE,FAIL,toolong");
    }
  }
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  uint64_t rxUs = nowUs();  // thời điểm nhận packet ở master
  const uint8_t *src = info->src_addr;

  markSlaveSeen(src);
  noteSlaveRssiIfKnown(info, src);

  // ---- Slave trả lời CALGET: thông số calib đang lưu / RAM ----
  if (len == (int)sizeof(accel_calib_packet_t) || len == ACCEL_CALIB_PACKET_LEGACY_SIZE) {
    accel_calib_packet_t calibIn = {};
    memcpy(&calibIn, incomingData, (size_t)len);
    if (strncmp(calibIn.type, "CALREP", 6) == 0) {
      const float gxb = (len == (int)sizeof(accel_calib_packet_t)) ? calibIn.gyro_bias[0] : 0.0f;
      const float gyb = (len == (int)sizeof(accel_calib_packet_t)) ? calibIn.gyro_bias[1] : 0.0f;
      const float gzb = (len == (int)sizeof(accel_calib_packet_t)) ? calibIn.gyro_bias[2] : 0.0f;
      Serial.printf(
          "CALIB_REPORT,%02X:%02X:%02X:%02X:%02X:%02X,%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
          src[0], src[1], src[2], src[3], src[4], src[5],
          calibIn.enabled ? "ENABLED" : "DISABLED",
          calibIn.bias[0], calibIn.bias[1], calibIn.bias[2],
          calibIn.scale[0], calibIn.scale[1], calibIn.scale[2],
          calibIn.global_scale,
          gxb, gyb, gzb);
      return;
    }
  }

  // ---- IMU batch (nhiều mẫu / gói ESP-NOW) → Serial vẫn là từng mẫu (A5 + imu_packet_raw_t) ----
  if (tryExpandImuBatchToSerial(incomingData, len, rxUs)) {
    return;
  }
  if (tryExpandImuRtxBatchToSerial(incomingData, len, rxUs)) {
    return;
  }

  // ---- VL53 batch (int16 raw / zone) → Serial từng dòng VL53,... ----
  if (tryExpandVl53BatchToSerial(incomingData, len, rxUs)) {
    return;
  }

  // ---- IMU packet (int16 thô + temp_centi_c) ----
  if (len == (int)sizeof(imu_packet_raw_t)) {
    imu_packet_raw_t raw;
    memcpy(&raw, incomingData, sizeof(raw));
    if (strncmp(raw.type, "IMU_RAW", 7) == 0) {
#if IMU_SERIAL_BINARY
      serialWriteImuRawFrame(raw);
#else
#if MASTER_SERIAL_SEQ_FILTER
      if (!imuSerialShouldForwardWithNode(raw.mac, raw.node_id, raw.sample_seq)) {
        return;
      }
#endif
      const float acc = 1.0f / 512.0f;
      const float gsc = 1.0f / IMU_RAW_GYRO_SCALE;
      const float temp_c = (float)raw.temp_centi_c / IMU_TEMP_CENTI_SCALE;
      Serial.printf(
          "IMU,%02X:%02X:%02X:%02X:%02X:%02X,%llu,%llu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f\n",
          src[0], src[1], src[2], src[3], src[4], src[5],
          (unsigned long long)raw.micros_timestamp,
          (unsigned long long)nowUs(),
          (unsigned long)raw.sample_seq,
          raw.ax * acc, raw.ay * acc, raw.az * acc,
          raw.gx * gsc, raw.gy * gsc, raw.gz * gsc, temp_c);
#endif
      return;
    }
  }

  // ---- IMU packet float (slave cũ) ----
  if (len == (int)sizeof(imu_packet_t)) {
    imu_packet_t imu;
    memcpy(&imu, incomingData, sizeof(imu));

    if (strncmp(imu.type, "IMU", 3) == 0) {
#if IMU_SERIAL_BINARY
      /* Gói float cũ: không có sample_seq hợp lệ (raw=0) — gửi không lọc seq. */
      serialWriteImuRawFrameImpl(floatImuPacketToRaw(imu));
#else
      Serial.printf(
        "IMU,%02X:%02X:%02X:%02X:%02X:%02X,%llu,%llu,0,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        src[0], src[1], src[2], src[3], src[4], src[5],
        (unsigned long long)imu.micros_timestamp,
        (unsigned long long)nowUs(),
        imu.ax, imu.ay, imu.az,
        imu.gx, imu.gy, imu.gz
      );
#endif
      return;
    }
  }

  // ---- VL53 packet ----
  if (len == (int)sizeof(vl53_packet_t)) {
    vl53_packet_t vl53;
    memcpy(&vl53, incomingData, sizeof(vl53));

    if (strncmp(vl53.type, "VL53", 4) == 0) {
#if IMU_SERIAL_BINARY
      serialWriteVl53RawFrame(vl53);
#else
#if MASTER_SERIAL_SEQ_FILTER
      if (!vl53SerialShouldForward(vl53.mac, vl53.sample_seq)) {
        return;
      }
#endif
      Serial.printf("VL53,%02X:%02X:%02X:%02X:%02X:%02X,%lu,%llu,%lu",
                    src[0], src[1], src[2], src[3], src[4], src[5],
                    (unsigned long)vl53.timestamp,
                    (unsigned long long)vl53.micros_timestamp,
                    (unsigned long)vl53.sample_seq);
      for (int i = 0; i < VL53_ZONE_COUNT; i++) {
        Serial.printf(",%d", (int)vl53.distance_raw[i]);
      }
      Serial.println();
#endif
      return;
    }
  }

  // ---- control packet ----
  if (len == (int)sizeof(packet_t)) {
    packet_t pkt;
    memcpy(&pkt, incomingData, sizeof(pkt));

    if (strcmp(pkt.type, "CACK") == 0) {
      Serial.printf("CALIB_ACK,%02X:%02X:%02X:%02X:%02X:%02X,OK\n",
                    src[0], src[1], src[2], src[3], src[4], src[5]);
      return;
    }

    if (strcmp(pkt.type, "CERR") == 0) {
      Serial.printf("CALIB_ACK,%02X:%02X:%02X:%02X:%02X:%02X,FAIL\n",
                    src[0], src[1], src[2], src[3], src[4], src[5]);
      return;
    }
  }

  packet_t pkt = {};
  if (len == (int)sizeof(packet_t)) {
    memcpy(&pkt, incomingData, sizeof(pkt));
  } else if (len >= (int)PACKET_T_LEGACY_SIZE) {
    memcpy(&pkt, incomingData, (size_t)len);
    if (len < (int)sizeof(packet_t)) {
      pkt.imu_tx_slot = MASTER_SLOT_UNASSIGNED;
      pkt.sync_slot = MASTER_SLOT_UNASSIGNED;
      pkt.imu_tx_slot_count = MASTER_IMU_TX_SLOT_COUNT;
      pkt.slot_flags = 0;
    }
    if (len < (int)offsetof(packet_t, last_sync_rtt_us)) {
      pkt.last_sync_rtt_us = 0;
    }
  } else {
    return;
  }

  if (strcmp(pkt.type, "HELLO") == 0) {
    Serial.print("📩 HELLO từ ");
    printMAC(src);
    Serial.println();

    ensureSlavePeer(src);
    noteSlaveRssiIfKnown(info, src);
    sendAck(src);
    return;
  }

  if (strcmp(pkt.type, "SYNC") == 0) {
    ensureSlavePeer(src);
    noteSlaveRssiIfKnown(info, src);
    masterNoteSlaveBatteryFromSync(src, info, pkt.slave_adc_raw);
    sendTimeResponse(src, pkt, rxUs);
    return;
  }
}

void printMasterMAC() {
  uint8_t mac[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
  Serial.print("🚀 Master MAC: ");
  printMAC(mac);
  Serial.println();
}

void setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                        WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
#else
  esp_wifi_set_protocol(WIFI_IF_STA,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                        WIFI_PROTOCOL_11N);
#endif

#if MASTER_WIFI_TX_POWER_ADAPTIVE
  masterWifiTxPowerInit();
#else
  esp_wifi_set_max_tx_power(MAX_TX_POWER);
#endif

  uint8_t ch;
  wifi_second_chan_t sc;
  esp_wifi_get_channel(&ch, &sc);
  Serial.printf("Wi-Fi channel locked at %u%s\n", ch, USE_LONG_RANGE ? " (LR ON)" : "");
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(921600);
  delay(200);

  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  Serial.println("\n=== ESP-NOW TIME SYNC MASTER (ESP32-S, no display) ===");

  setupRadio();

  printMasterMAC();
  Serial.println("----------------------------------");

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Lỗi khởi tạo ESP-NOW!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("✅ Master sẵn sàng, chờ slave kết nối...");
  printMasterTime();
}

void loop() {
  processSerialCommands();

  checkSlaveTimeouts();
#if MASTER_IMU_LOST_REQUEST_ENABLE
  masterImuRetxTimeoutPoll();
#endif
  static unsigned long s_lastRssiLogMs = 0;
  const unsigned long nowm = millis();
  if (slaveCount > 0 &&
      (nowm - s_lastRssiLogMs >= SLAVE_RSSI_LOG_INTERVAL_MS)) {
    s_lastRssiLogMs = nowm;
    updateSlaveRssiDisplayFromWindows();
    logSlaveRssiSummary();
  }

  if (millis() - lastPrint > 5000) {
#if !IMU_SERIAL_BINARY
    printSlaveList();
#endif
    lastPrint = millis();
  }

  delay(1);
}