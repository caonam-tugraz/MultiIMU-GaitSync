#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ICM42688.h"
#include "registers.h"
#include "esp_wifi.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ======================== CONFIG ========================
#define CHANNEL 1
#define USE_LONG_RANGE 1

#define PULSE_PIN 23
#define LED_PIN 2

#define LED_FLASH_MS 25
#define LED_RECONNECT_CYCLE 2000
#define LED_NO_MASTER_CYCLE 1500
#define NO_MASTER_THRESHOLD 120000

#define CONNECTION_TIMEOUT_MS 5000
/** Chưa có master (chưa kết nối): gửi lại HELLO tối đa mỗi khoảng này (ms). */
#define HELLO_RECONNECT_INTERVAL 3000
/** Sau N lần HELLO/kết nối thất bại liên tiếp (chưa connected) → esp_restart(). */
#define LINK_HELLO_FAIL_RESET_MAX 5U
/** Khi đã kết nối: HELLO định kỳ (ms); 0 = tắt — không spam RF / master ACK. */
#ifndef HELLO_KEEPALIVE_INTERVAL
#define HELLO_KEEPALIVE_INTERVAL 0
#endif

#define ESPNOW_SEND_WAIT_MS 250U
#ifndef ESPNOW_SYNC_SEND_WAIT_MS
#define ESPNOW_SYNC_SEND_WAIT_MS 80U
#endif

#define IMU_LOOP_PERIOD_US 10000u
/**
 * Khi đã timeSynced + IMU_SAMPLE_ALIGN_MASTER_10MS_GRID:
 * đọc IMU khi getGlobalTimeUs() % IMU_LOOP_PERIOD_US == IMU_SAMPLE_GRID_PHASE_US.
 * 0 = đúng biên lưới (mốc chia hết cho 10 ms), giống slave BNO055.
 * Khi chưa sync: vẫn vTaskDelayUntil 10 ms (pha ngẫu nhiên giữa các slave).
 */
#ifndef IMU_SAMPLE_ALIGN_MASTER_10MS_GRID
#define IMU_SAMPLE_ALIGN_MASTER_10MS_GRID 1
#endif
#if IMU_SAMPLE_ALIGN_MASTER_10MS_GRID
#ifndef IMU_SAMPLE_GRID_PHASE_US
#define IMU_SAMPLE_GRID_PHASE_US 0u
#endif
#if IMU_SAMPLE_GRID_PHASE_US >= IMU_LOOP_PERIOD_US
#error IMU_SAMPLE_GRID_PHASE_US must be < IMU_LOOP_PERIOD_US
#endif
#endif
/**
 * 1 = task FreeRTOS đọc IMU ~100 Hz; loop chỉ gửi queue ESP-NOW/SYNC (không chặn lấy mẫu).
 * 0 = đọc IMU trong loop() + delay cuối vòng.
 */
#ifndef ENABLE_IMU_DEDICATED_TASK
#define ENABLE_IMU_DEDICATED_TASK 1
#endif
#if ENABLE_IMU_DEDICATED_TASK
#define IMU_SAMPLE_TASK_STACK 8192
#define IMU_SAMPLE_TASK_PRIO 5
#define IMU_BATCH_QUEUE_DEPTH 8
/** Core 0: tách SPI khỏi loop Arduino (thường core 1) + WiFi. */
#define IMU_SAMPLE_TASK_CORE 0
#endif
/** Số lần gọi lại getAGT() ngay khi lỗi (thêm 2 → tối đa 3 lần đọc/mẫu). */
#define IMU_GET_AGT_RETRY_EXTRA 2
#define ENABLE_WIFI_SEND 1
#define SERIAL_IMU_OUTPUT 0
/**
 * 0 = slave tự gửi SYNC (định kỳ + sau lỗi RF) — mặc định gốc.
 * 1 = chỉ đồng bộ khi Master gửi TSRQ.
 */
#ifndef SLAVE_SYNC_MASTER_DRIVEN
#define SLAVE_SYNC_MASTER_DRIVEN 0
#endif

/** 1 = in slot IMU/SYNC khi nhận gói Master (dễ spam Serial). */
#ifndef DEBUG_IMU_SLOT_SERIAL
#define DEBUG_IMU_SLOT_SERIAL 0
#endif

/**
 * RSSI nhận từ master < ngưỡng: đệm lô IMU_RAWB; service gửi tối đa SLAVE_IMU_LOW_RSSI_SENDS lần
 * cách SLAVE_IMU_LOW_RSSI_SEND_GAP_MS, dừng khi OK (không chặn loop — millis + loop).
 */
#ifndef SLAVE_IMU_LOW_RSSI_DB
#define SLAVE_IMU_LOW_RSSI_DB (-68)
#endif
/** RSSI thấp: vẫn SYNC nếu RTT < SYNC_ACCEPT_MAX_RTT_US; 1 = chặn SYNC khi RSSI < ngưỡng. */
#ifndef SLAVE_SYNC_RSSI_GATE
#define SLAVE_SYNC_RSSI_GATE 0
#endif
#ifndef SLAVE_SYNC_MIN_RSSI_DB
#define SLAVE_SYNC_MIN_RSSI_DB (-80)
#endif
#ifndef SLAVE_IMU_LOW_RSSI_SENDS
#define SLAVE_IMU_LOW_RSSI_SENDS 2U
#endif
#ifndef SLAVE_IMU_LOW_RSSI_SEND_GAP_MS
#define SLAVE_IMU_LOW_RSSI_SEND_GAP_MS 10U
#endif
#ifndef SLAVE_IMU_LOW_RSSI_QUEUE_CAP
#define SLAVE_IMU_LOW_RSSI_QUEUE_CAP 8U
#endif

/**
 * Công suất phát WiFi (esp_wifi_set_max_tx_power, đơn vị 0,25 dBm).
 * Mặc định khởi động ở max (84 ≈ 21 dBm); thu: WIFI_PS_NONE (không sleep).
 */
#ifndef SLAVE_WIFI_TX_POWER_MAX_QDB
#define SLAVE_WIFI_TX_POWER_MAX_QDB 84
#endif
#ifndef SLAVE_WIFI_TX_POWER_ADAPTIVE
#define SLAVE_WIFI_TX_POWER_ADAPTIVE 0
#endif
#ifndef SLAVE_WIFI_TX_POWER_NORM_QDB
#define SLAVE_WIFI_TX_POWER_NORM_QDB SLAVE_WIFI_TX_POWER_MAX_QDB
#endif
#ifndef SLAVE_WIFI_TX_POWER_BOOST_QDB
#define SLAVE_WIFI_TX_POWER_BOOST_QDB SLAVE_WIFI_TX_POWER_MAX_QDB
#endif
#ifndef SLAVE_WIFI_TX_RSSI_BOOST_DB
#define SLAVE_WIFI_TX_RSSI_BOOST_DB SLAVE_IMU_LOW_RSSI_DB
#endif
#ifndef SLAVE_WIFI_TX_RSSI_RESTORE_DB
#define SLAVE_WIFI_TX_RSSI_RESTORE_DB (-65)
#endif
#ifndef SLAVE_WIFI_TX_POWER_ADJ_INTERVAL_MS
#define SLAVE_WIFI_TX_POWER_ADJ_INTERVAL_MS 2000U
#endif

// 42688 
#define SDA_PIN 4
#define SCL_PIN 16

// GY601N
// #define SDA_PIN 4
// #define SCL_PIN 5


#define ICM42688_ADDR 0x69

#define IMU_ACCEL_FS ICM42688::gpm8
#define IMU_GYRO_FS ICM42688::dps2000
#define IMU_ACCEL_ODR ICM42688::odr100
#define IMU_GYRO_ODR ICM42688::odr100

#define ACC_G_TO_MSS 9.80665f

// Gói IMU ESP-NOW: int16 thô (khớp imu_serial_codec.py trên PC)
#define IMU_RAW_ACC_SCALE 512.0f
// Gyro °/s → int16: cần scale ≤ 32767/FS_dps hoặc sẽ bão hòa sớm (vd. 100 → chỉ ~±328 °/s).
// Đặt FS_dps trùng với IMU_GYRO_FS (vd. dps2000 → 2000).
#define IMU_GYRO_FS_DPS 2000.0f
#define IMU_RAW_GYRO_SCALE (32767.0f / IMU_GYRO_FS_DPS)
/** Nhiệt độ die (°C) → int16 trên không trùng: ×100 (0,01 °C/LSB) */
#define IMU_TEMP_CENTI_SCALE 100.0f
/** ICM42688 không có từ kế: luôn gửi 0 cho mx/my/mz (cùng khung với slave có MAG). */
#define IMU_MAG_INT16_NONE 0

// ---- SYNC CONFIG ----
/** Chu kỳ đồng bộ định kỳ khi đã ổn định (ms) = 30 s + MAC slot; trùng chu kỳ fold drift. */
#define SYNC_INTERVAL_MS 30000UL
/** Mặc định dùng ngay SYNC_INTERVAL_MS (không ramp). Muốn ramp: đặt START nhỏ hơn MAX, STEP > 0. */
#ifndef SYNC_INTERVAL_START_MS
#define SYNC_INTERVAL_START_MS SYNC_INTERVAL_MS
#endif
#ifndef SYNC_INTERVAL_STEP_MS
#define SYNC_INTERVAL_STEP_MS 0U
#endif
/** Đã từng có lastSyncTime nhưng cần mở lại vòng SYNC: chờ tối thiểu (ms) giữa các lần thử. */
#define SYNC_RETRY_AFTER_FAIL_MS 3000U
/** RTT (µs) dưới ngưỡng này → chấp nhận offset ngay; >= ngưỡng → bỏ qua mẫu. */
#define SYNC_ACCEPT_MAX_RTT_US 4000LL
#define SYNC_SAMPLE_GAP_MS 20
#ifndef SYNC_FAIL_BACKOFF_GAP_MS
#define SYNC_FAIL_BACKOFF_GAP_MS 100U
#endif
#ifndef IMU_TX_FAIL_RESYNC_AFTER
#define IMU_TX_FAIL_RESYNC_AFTER 3U
#endif
#ifndef IMU_TX_FAIL_RETRY_GAP_MS
#define IMU_TX_FAIL_RETRY_GAP_MS 50U
#endif
#define SYNC_PERIODIC_MAC_SLOT_DIVISOR 25U
#define SYNC_PERIODIC_SLOT_SPACING_MS 200UL
#define SYNC_PERIODIC_SLOT_WINDOW_MS 50UL
#define SYNC_PERIODIC_MAX_OFFSET_MS \
  ((255U / SYNC_PERIODIC_MAC_SLOT_DIVISOR) * SYNC_PERIODIC_SLOT_SPACING_MS)
/** Drift từ thay đổi median offset giữa các vòng SYNC_DONE. */
#define SYNC_DRIFT_MIN_SPAN_US 10000000ULL
#define SYNC_DRIFT_MAX_ABS_PPM 60.0
#define SYNC_DRIFT_FILTER_ALPHA 0.10
#ifndef SYNC_DRIFT_COMP_ENABLE
#define SYNC_DRIFT_COMP_ENABLE 1
#endif
/** Gộp drift tích lũy vào globalTimeOffset (cùng chu kỳ SYNC_INTERVAL_MS). */
#define SYNC_DRIFT_COMP_INTERVAL_MS SYNC_INTERVAL_MS
#define SYNC_DRIFT_COMP_MIN_ACCEPTED 3U
#define SYNC_DRIFT_NVS_HISTORY_N 5U
#define SYNC_DRIFT_NVS_STD_MAX_PPM 5.0
#define SYNC_DRIFT_NVS_SAVE_DELTA_PPM 1.0
#define SYNC_DRIFT_NVS_SAVE_INTERVAL_MS 600000UL

#define VBAT_ADC_PIN 34
/** Trung bình trượt N mẫu; tần số lấy mẫu ADC = 1000/VBAT_ADC_SAMPLE_PERIOD_MS (Hz). */
#define VBAT_ADC_MA_N 10
#define VBAT_ADC_SAMPLE_PERIOD_MS 500UL
/** Khớp Master TFT: % pin từ cell (3.2V…4.1V), raw 12-bit chân chia áp. */
#define VBAT_CELL_FULL_V 4.1f
#define VBAT_CELL_EMPTY_V 3.2f
#define VBAT_ADC_PIN_VCELL_SCALE 0.496f
#define VBAT_ADC_PIN_AT_CELL_FULL_V (VBAT_CELL_FULL_V * VBAT_ADC_PIN_VCELL_SCALE)
#define VBAT_ADC_REF_V 3.3f
#define VBAT_VCELL_PER_VADC (VBAT_CELL_FULL_V / VBAT_ADC_PIN_AT_CELL_FULL_V)
/** Pin < 10%: tắt WiFi, dừng IMU, LED D2 nhấp 2s tắt / 0.2s sáng. */
#define VBAT_LOW_PCT_ENTER 10
#define VBAT_LOW_LED_OFF_MS 2000UL
#define VBAT_LOW_LED_CYCLE_MS (VBAT_LOW_LED_OFF_MS + 200UL)

ICM42688 IMU(Wire, ICM42688_ADDR);
Preferences prefs;

// ======================== PACKETS ========================
typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint32_t timestamp;
  uint64_t micros_timestamp;
  int timezone_offset;
  uint64_t request_time;
  uint64_t response_time;
  uint16_t slave_adc_raw;
  uint32_t last_sync_rtt_us;
  uint8_t imu_tx_slot;
  uint8_t sync_slot;
  uint8_t imu_tx_slot_count;
  uint8_t slot_flags;
} packet_t;

#define MASTER_IMU_TX_SLOT_COUNT 10U
#define MASTER_SYNC_SLOT_COUNT 11U
#define MASTER_SLOT_UNASSIGNED 0xFFU
#define PACKET_T_SLOT_FLAG_VALID 0x01U
#define PACKET_T_LEGACY_SIZE ((size_t)offsetof(packet_t, imu_tx_slot))

// Gói IMU tối giản (packed): micros + sample_seq + accel/gyro int16 — MAG = IMU_MAG_INT16_NONE.
#pragma pack(push, 1)
typedef struct {
  char type[8];
  uint8_t mac[6];
  uint32_t node_id;
  uint64_t micros_timestamp;
  uint32_t sample_seq;
  int16_t ax, ay, az, gx, gy, gz;
  int16_t mx, my, mz;
  int16_t temp_centi_c;
} imu_packet_raw_t;

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
  int16_t temp_centi_c;
  imu_batch_delta_t rest[IMU_BATCH_MAX_SAMPLES - 1];
} imu_batch_packet_t;
#pragma pack(pop)

#define IMU_BATCH_HEADER_SIZE 54
// Số mẫu IMU gộp mỗi gói ESP-NOW (100 Hz / 5 = 20 gói/s).
#define IMU_BATCH_SAMPLES 5
/** Mất kết nối tạm thời: ~600 mẫu (~6 s @ ~100 Hz). */
#define IMU_OFFLINE_BUFFER_SAMPLES 600
#define IMU_OFFLINE_BATCH_CAP (IMU_OFFLINE_BUFFER_SAMPLES / IMU_BATCH_SAMPLES)

/** TDMA mềm (nhiều slave): slot_index = (MAC[4] | MAC[5]<<8) % N; thời điểm theo getGlobalTimeUs() sau SYNC (không trôi). */
#ifndef ENABLE_IMU_TX_SLOT
#define ENABLE_IMU_TX_SLOT 1
#endif
#if ENABLE_IMU_TX_SLOT
#define IMU_TX_SLOT_COUNT 10
/** 10 × 5000 µs = 50 ms một khung TDMA. */
#define IMU_TX_SLOT_US 5000u
/** Chờ tối đa trong slot 5 ms; hết ngưỡng vẫn gửi (có thể lệch TDMA nhẹ). */
#define IMU_TX_SLOT_MAX_SPIN_US 2000u
#endif

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

#ifndef ACCEL_CALIB_PACKET_LEGACY_SIZE
#define ACCEL_CALIB_PACKET_LEGACY_SIZE 64
#endif

/** Master → Slave: yêu cầu gửi lại mẫu theo dải sample_seq (khớp master esp_now_time_sync). */
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

/** ~100 Hz retx (~7,7 s); 768 thay 896 để offline 600 mẫu vừa DRAM ESP32. */
#define IMU_RETX_RING_CAP 768U
typedef struct {
  uint32_t seq;
  uint64_t micros;
  int16_t ax, ay, az, gx, gy, gz;
  int16_t mx, my, mz;
  int16_t temp_centi_c;
} imu_retx_slot_t;
static imu_retx_slot_t imuRetxRing[IMU_RETX_RING_CAP];

// ======================== MASTER MAC (đồng bộ BNO055 slave + ICM) ========================
// Cũ (TFT master): 3C:E9:0E:B3:FA:C0
// uint8_t masterMAC[6] = {0x3C, 0xE9, 0x0E, 0xB3, 0xFA, 0xC0};
uint8_t masterMAC[6] = {0xE4, 0x65, 0xB8, 0xE6, 0x79, 0xE4};

// ======================== STATE ========================
volatile bool connected = false;
bool hasEverConnected = false;
volatile bool timeSynced = false;

unsigned long lastHelloTime = 0;
volatile unsigned long lastAckTime = 0;
unsigned long ledOffAt = 0;
unsigned long disconnectedSince = 0;
unsigned long lastSyncTime = 0;
static uint32_t syncPeriodicIntervalMs = SYNC_INTERVAL_START_MS;
unsigned int imuPacketCount = 0;
uint32_t imuSampleSeq = 0;
bool imuReadErrorPrinted = false;

#if ENABLE_WIFI_SEND
static bool imuOfflineCaptureMode = false;
#endif

// ---- time sync state ----
int64_t globalTimeOffset = 0;
int64_t globalTimeOffsetTarget = 0;
int64_t networkDelayUs = 0;
int timezoneOffset = 0;

bool syncRunning = false;
unsigned long lastSyncSampleSentMs = 0;

/** Debug drift ổn định hơn: dùng thay đổi median offset giữa các vòng SYNC_DONE. */
static uint64_t syncPrevDriftLocalUs = 0;
static int64_t syncPrevDriftOffsetUs = 0;
static bool syncPrevDriftSampleValid = false;
static double syncDriftFilteredPpm = 0.0;
static bool syncDriftFilteredValid = false;
static uint8_t syncDriftAcceptedCount = 0;
static uint64_t syncDriftAnchorLocalUs = 0;
static unsigned long syncDriftLastCompMs = 0;
static double syncDriftRecentPpm[SYNC_DRIFT_NVS_HISTORY_N] = {0.0};
static uint8_t syncDriftRecentCount = 0;
static uint8_t syncDriftRecentNext = 0;
static float syncDriftLastSavedPpm = 0.0f;
static bool syncDriftLastSavedValid = false;
static uint32_t syncDriftLastSaveMs = 0;

/** Gửi trong gói SYNC tới: RTT trung vị vòng đồng bộ trước — khớp BNO055 / Master. */
uint32_t lastCompletedSyncBestRttUs = 0;

/**
 * Luôn bật đường hiệu chỉnh accel: acc_c = (raw - bias) * scale * global_scale.
 * Mặc định / sau CLRCAL: bias=0, scale=1, global_scale=1 → acc_c ≈ raw (trung tính).
 */
bool accelCalibEnabled = true;
float accelBias[3] = {0.0f, 0.0f, 0.0f};
float accelScale[3] = {1.0f, 1.0f, 1.0f};
float accelGlobalScale = 1.0f;
/** Bù offset gyro (°/s), trừ sau khi đọc chip — trước khi scale int16. */
float gyroBias[3] = {0.0f, 0.0f, 0.0f};

#if ENABLE_WIFI_SEND
/** 0 = chờ callback, 1 = ESP_NOW_SEND_SUCCESS, 2 = fail/timeout */
volatile uint8_t espnowTxResult = 0;
static uint8_t imuTxFailStreak = 0;
static bool syncLastSendOk = true;
static uint32_t imuTxNextAttemptMs = 0;
static uint8_t helloConnectFailCount = 0;
#endif
/** Cập nhật trong onDataRecv (rx_ctrl); dùng cho chính sách gửi IMU khi RSSI thấp. */
volatile int8_t g_slaveLastRxRssiDb = 0;
volatile uint8_t g_slaveLastRxRssiValid = 0;

#if ENABLE_IMU_DEDICATED_TASK
static QueueHandle_t imuBatchTxQueue = NULL;
static TaskHandle_t imuSampleTaskHandle = NULL;
#endif

static bool lowBatteryCrit = false;

static void syncDriftRecentAdd(double ppm);
static void maybeSaveStableSyncDriftToNvs(void);
static bool syncDriftCompActive(void);
static int64_t syncDriftCorrectionUs(uint64_t tNow);
static void syncDriftFoldPendingIntoOffset(uint64_t tNow, bool log);
static void syncDriftPeriodicService(void);

// ======================== UTIL ========================
uint64_t nowUs() {
  return (uint64_t)esp_timer_get_time();
}

uint32_t nowMs() {
  return (uint32_t)(nowUs() / 1000ULL);
}

static uint16_t vbatMaBuf[VBAT_ADC_MA_N];
static uint8_t vbatMaNext = 0;
static uint8_t vbatMaCount = 0;
static unsigned long vbatMaLastPollMs = 0;

/** Gọi mỗi vòng loop: thêm 1 mẫu ADC vào buffer (chu kỳ VBAT_ADC_SAMPLE_PERIOD_MS). */
static void vbatAdcPoll() {
  unsigned long ms = millis();
  if (vbatMaLastPollMs != 0 &&
      (unsigned long)(ms - vbatMaLastPollMs) < VBAT_ADC_SAMPLE_PERIOD_MS) {
    return;
  }
  vbatMaLastPollMs = ms;
  vbatMaBuf[vbatMaNext] = (uint16_t)analogRead(VBAT_ADC_PIN);
  vbatMaNext = (uint8_t)((vbatMaNext + 1) % VBAT_ADC_MA_N);
  if (vbatMaCount < VBAT_ADC_MA_N) {
    vbatMaCount++;
  }
}

/** Trung bình N mẫu cuối (moving average). */
static uint16_t readVbatAdcRaw() {
  if (vbatMaCount == 0) {
    return (uint16_t)analogRead(VBAT_ADC_PIN);
  }
  uint32_t sum = 0;
  if (vbatMaCount < VBAT_ADC_MA_N) {
    for (uint8_t i = 0; i < vbatMaCount; i++) {
      sum += vbatMaBuf[i];
    }
    return (uint16_t)((sum + vbatMaCount / 2) / vbatMaCount);
  }
  for (uint8_t i = 0; i < VBAT_ADC_MA_N; i++) {
    sum += vbatMaBuf[(vbatMaNext + i) % VBAT_ADC_MA_N];
  }
  return (uint16_t)((sum + VBAT_ADC_MA_N / 2) / VBAT_ADC_MA_N);
}

static float adcRawToVadcPin(uint16_t raw12) {
  return (float)raw12 * (VBAT_ADC_REF_V / 4095.0f);
}

static float adcRawToVcellEst(uint16_t raw12) {
  return adcRawToVadcPin(raw12) * VBAT_VCELL_PER_VADC;
}

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

static void enterLowBatteryCritMode(void) {
  if (lowBatteryCrit) {
    return;
  }
  lowBatteryCrit = true;
  ledOffAt = 0;
  Serial.println("LOW BAT <10%: stop IMU, WiFi off, LED blink 2s/0.2s");
#if ENABLE_IMU_DEDICATED_TASK
  if (imuSampleTaskHandle != NULL) {
    vTaskSuspend(imuSampleTaskHandle);
  }
#endif
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  esp_now_deinit();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  connected = false;
  hasEverConnected = false;
}

static bool syncDriftCompActive(void) {
#if SYNC_DRIFT_COMP_ENABLE
  return syncDriftFilteredValid &&
         syncDriftAcceptedCount >= (uint8_t)SYNC_DRIFT_COMP_MIN_ACCEPTED;
#else
  return false;
#endif
}

static int64_t syncDriftCorrectionUs(uint64_t tNow) {
#if SYNC_DRIFT_COMP_ENABLE
  if (!syncDriftCompActive() || syncDriftAnchorLocalUs == 0ULL ||
      tNow < syncDriftAnchorLocalUs) {
    return 0;
  }
  const uint64_t elapsed = tNow - syncDriftAnchorLocalUs;
  return (int64_t)((double)elapsed * syncDriftFilteredPpm / 1000000.0);
#else
  (void)tNow;
  return 0;
#endif
}

static void syncDriftFoldPendingIntoOffset(uint64_t tNow, bool log) {
#if SYNC_DRIFT_COMP_ENABLE
  if (!syncDriftCompActive() || syncDriftAnchorLocalUs == 0ULL ||
      tNow < syncDriftAnchorLocalUs) {
    return;
  }
  const int64_t delta = syncDriftCorrectionUs(tNow);
  if (delta == 0) {
    return;
  }
  globalTimeOffset += delta;
  globalTimeOffsetTarget = globalTimeOffset;
  syncDriftAnchorLocalUs = tNow;
  if (log) {
    Serial.printf(
        "DRIFT_COMP fold=%lld us ppm=%.3f offset=%lld us anchor=%llu us\n",
        (long long)delta, syncDriftFilteredPpm, (long long)globalTimeOffset,
        (unsigned long long)tNow);
  }
#else
  (void)tNow;
  (void)log;
#endif
}

static void syncDriftPeriodicService(void) {
#if SYNC_DRIFT_COMP_ENABLE
  if (!timeSynced || !syncDriftCompActive()) {
    return;
  }
  const unsigned long nowMs = millis();
  if (syncDriftLastCompMs != 0U &&
      (unsigned long)(nowMs - syncDriftLastCompMs) <
          (unsigned long)SYNC_DRIFT_COMP_INTERVAL_MS) {
    return;
  }
  syncDriftFoldPendingIntoOffset(nowUs(), true);
  syncDriftLastCompMs = nowMs;
#endif
}

int64_t getGlobalTimeUs() {
  int64_t t = (int64_t)nowUs();
  if (timeSynced) {
    return t + globalTimeOffset + syncDriftCorrectionUs((uint64_t)t);
  }
  return t;
}

#if ENABLE_WIFI_SEND
static bool imuOfflineCaptureActive(void);
static void imuOfflineCaptureEndOnReconnect(void);
#else
static bool imuOfflineCaptureActive(void);
#endif

static void touchMasterLink(void) {
  lastAckTime = millis();
  connected = true;
#if ENABLE_WIFI_SEND
  helloConnectFailCount = 0;
  imuOfflineCaptureEndOnReconnect();
#endif
  hasEverConnected = true;
  if (disconnectedSince != 0) {
    disconnectedSince = 0;
  }
}

/** Đọc sensor ~100 Hz khi còn link hoặc đang ghi đệm offline sau mất link. */
static bool imuSampleAllowed(void) {
  if (imuOfflineCaptureActive()) {
    return true;
  }
  if (!connected) {
    return false;
  }
  const unsigned long lastAck = lastAckTime;
  if (lastAck == 0UL ||
      (unsigned long)(millis() - lastAck) > CONNECTION_TIMEOUT_MS) {
    return false;
  }
  return true;
}

/** Đóng gói / gửi IMU_RAWB chỉ khi đã SYNC, link OK và không đang vòng SYNC. */
static bool imuStreamReady(void) {
  return imuSampleAllowed() && timeSynced && !syncRunning;
}

/** Micros ghi cho mẫu IMU: mốc trigger ngay trước khi bắt đầu đọc sensor. */
static uint64_t imuTimestampForSampleLoggedUs(void) {
  int64_t t = getGlobalTimeUs();
  return (uint64_t)((t < 0) ? 0 : t);
}

void printMAC(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

void printTime() {
  uint64_t localUs = nowUs();

  if (timeSynced) {
    int64_t adjustedUs = getGlobalTimeUs();
    uint32_t adjustedMs = (uint32_t)(adjustedUs / 1000LL);

    int hours = (adjustedMs / 3600000UL) % 24;
    int minutes = (adjustedMs / 60000UL) % 60;
    int seconds = (adjustedMs / 1000UL) % 60;
    int milliseconds = adjustedMs % 1000UL;

    Serial.printf("TIME %02d:%02d:%02d.%03d (synced)\n",
                  hours, minutes, seconds, milliseconds);
  } else {
    uint32_t localMs = (uint32_t)(localUs / 1000ULL);

    int hours = (localMs / 3600000UL) % 24;
    int minutes = (localMs / 60000UL) % 60;
    int seconds = (localMs / 1000UL) % 60;
    int milliseconds = localMs % 1000UL;

    Serial.printf("TIME %02d:%02d:%02d.%03d (local - not synced)\n",
                  hours, minutes, seconds, milliseconds);
  }
}

void applyAccelCalibration(float &ax, float &ay, float &az) {
  ax = (ax - accelBias[0]) * accelScale[0] * accelGlobalScale;
  ay = (ay - accelBias[1]) * accelScale[1] * accelGlobalScale;
  az = (az - accelBias[2]) * accelScale[2] * accelGlobalScale;
}

static void applyGyroCalibration(float &gx, float &gy, float &gz) {
  gx -= gyroBias[0];
  gy -= gyroBias[1];
  gz -= gyroBias[2];
}

/** Đặt lại hệ số identity; chế độ hiệu chỉnh accel vẫn BẬT; gyro bias về 0. */
void resetAccelCalibration() {
  accelCalibEnabled = true;
  accelBias[0] = accelBias[1] = accelBias[2] = 0.0f;
  accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;
  accelGlobalScale = 1.0f;
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
}

bool saveAccelCalibrationToNvs() {
  bool ok = prefs.begin("imu_calib", false);
  if (!ok) {
    return false;
  }
  prefs.putBool("enabled", true);
  prefs.putBytes("bias", accelBias, sizeof(accelBias));
  prefs.putBytes("scale", accelScale, sizeof(accelScale));
  prefs.putFloat("gscale", accelGlobalScale);
  prefs.putBytes("gyro_bias", gyroBias, sizeof(gyroBias));
  prefs.end();
  return true;
}

void loadAccelCalibrationFromNvs() {
  bool ok = prefs.begin("imu_calib", true);
  if (!ok) {
    Serial.println("CALIB_LOAD,FAIL,open");
    return;
  }

  size_t biasLen = prefs.getBytesLength("bias");
  size_t scaleLen = prefs.getBytesLength("scale");
  if (biasLen == sizeof(accelBias)) {
    prefs.getBytes("bias", accelBias, sizeof(accelBias));
  } else {
    accelBias[0] = accelBias[1] = accelBias[2] = 0.0f;
  }

  if (scaleLen == sizeof(accelScale)) {
    prefs.getBytes("scale", accelScale, sizeof(accelScale));
  } else {
    accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;
  }

  accelGlobalScale = prefs.getFloat("gscale", 1.0f);

  size_t gbLen = prefs.getBytesLength("gyro_bias");
  if (gbLen == sizeof(gyroBias)) {
    prefs.getBytes("gyro_bias", gyroBias, sizeof(gyroBias));
  } else {
    gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
  }

  prefs.end();

  accelCalibEnabled = true;

  Serial.printf(
      "CALIB_LOAD,%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
      accelCalibEnabled ? "ENABLED" : "DISABLED",
      accelBias[0], accelBias[1], accelBias[2],
      accelScale[0], accelScale[1], accelScale[2],
      accelGlobalScale,
      gyroBias[0], gyroBias[1], gyroBias[2]);
}

static void syncDriftRecentReset(void) {
  syncDriftRecentCount = 0;
  syncDriftRecentNext = 0;
}

static void syncDriftRecentAdd(double ppm) {
  syncDriftRecentPpm[syncDriftRecentNext] = ppm;
  syncDriftRecentNext =
      (uint8_t)((syncDriftRecentNext + 1U) % (uint8_t)SYNC_DRIFT_NVS_HISTORY_N);
  if (syncDriftRecentCount < (uint8_t)SYNC_DRIFT_NVS_HISTORY_N) {
    syncDriftRecentCount++;
  }
}

static bool syncDriftRecentStats(double &meanPpm, double &stdPpm) {
  if (syncDriftRecentCount < (uint8_t)SYNC_DRIFT_NVS_HISTORY_N) {
    return false;
  }
  double sum = 0.0;
  for (uint8_t i = 0; i < syncDriftRecentCount; i++) {
    sum += syncDriftRecentPpm[i];
  }
  meanPpm = sum / (double)syncDriftRecentCount;
  double var = 0.0;
  for (uint8_t i = 0; i < syncDriftRecentCount; i++) {
    const double d = syncDriftRecentPpm[i] - meanPpm;
    var += d * d;
  }
  stdPpm = sqrt(var / (double)syncDriftRecentCount);
  return true;
}

static void loadSyncDriftFromNvs(void) {
  bool ok = prefs.begin("sync_drift", true);
  if (!ok) {
    Serial.println("DRIFT_LOAD,FAIL,open");
    return;
  }
  const bool valid = prefs.getBool("valid", false);
  const float ppm = prefs.getFloat("ppm", 0.0f);
  prefs.end();
  if (valid && isfinite(ppm) && fabs((double)ppm) <= SYNC_DRIFT_MAX_ABS_PPM) {
    syncDriftFilteredPpm = (double)ppm;
    syncDriftFilteredValid = true;
    syncDriftAcceptedCount = (uint8_t)SYNC_DRIFT_COMP_MIN_ACCEPTED;
    syncDriftLastSavedPpm = ppm;
    syncDriftLastSavedValid = true;
    Serial.printf("DRIFT_LOAD,OK,ppm=%.3f (used as initial prior)\n", (double)ppm);
  } else {
    Serial.printf("DRIFT_LOAD,NONE,valid=%d,ppm=%.3f\n", valid ? 1 : 0, (double)ppm);
  }
}

static bool saveSyncDriftToNvs(float ppm, double meanPpm, double stdPpm) {
  bool ok = prefs.begin("sync_drift", false);
  if (!ok) {
    Serial.println("DRIFT_SAVE,FAIL,open");
    return false;
  }
  prefs.putBool("valid", true);
  prefs.putFloat("ppm", ppm);
  prefs.end();
  syncDriftLastSavedPpm = ppm;
  syncDriftLastSavedValid = true;
  syncDriftLastSaveMs = millis();
  Serial.printf("DRIFT_SAVE,OK,ppm=%.3f,mean5=%.3f,std5=%.3f\n",
                (double)ppm, meanPpm, stdPpm);
  return true;
}

static void maybeSaveStableSyncDriftToNvs(void) {
  double meanPpm = 0.0;
  double stdPpm = 0.0;
  if (!syncDriftRecentStats(meanPpm, stdPpm)) {
    return;
  }
  if (stdPpm > SYNC_DRIFT_NVS_STD_MAX_PPM) {
    return;
  }
  if (!syncDriftFilteredValid) {
    return;
  }
  if (syncDriftLastSaveMs != 0U &&
      (uint32_t)(millis() - syncDriftLastSaveMs) <
          (uint32_t)SYNC_DRIFT_NVS_SAVE_INTERVAL_MS) {
    return;
  }
  const float ppmToSave = (float)meanPpm;
  if (syncDriftLastSavedValid &&
      fabs((double)ppmToSave - (double)syncDriftLastSavedPpm) <
          SYNC_DRIFT_NVS_SAVE_DELTA_PPM) {
    return;
  }
  (void)saveSyncDriftToNvs(ppmToSave, meanPpm, stdPpm);
}

void sendCalibAck(bool success) {
  packet_t pkt = {};
  strcpy(pkt.type, success ? "CACK" : "CERR");
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, pkt.mac);
#else
  esp_read_mac(pkt.mac, ESP_MAC_WIFI_STA);
#endif
  pkt.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  pkt.timestamp = nowMs();
  pkt.micros_timestamp = nowUs();
  pkt.timezone_offset = timezoneOffset;
  pkt.request_time = 0;
  pkt.response_time = 0;
  pkt.slave_adc_raw = 0;
  pkt.last_sync_rtt_us = 0;
#if ENABLE_WIFI_SEND
  /* Không dùng retry+callback chờ ở đây: onDataRecv có thể gọi khi đang chờ TX khác. */
  esp_now_send(masterMAC, (uint8_t *)&pkt, sizeof(pkt));
#endif
}

bool readImuSample(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz, float &temp_c) {
  int status = -1;
  const int kAttempts = 1 + IMU_GET_AGT_RETRY_EXTRA;
  for (int attempt = 0; attempt < kAttempts; attempt++) {
    status = IMU.getAGT();
    if (status >= 0) {
      break;
    }
    if (attempt + 1 < kAttempts) {
      delayMicroseconds(500);
    }
  }
  if (status < 0) {
    if (!imuReadErrorPrinted) {
      Serial.printf("IMU_READ_ERROR,%d\n", status);
      imuReadErrorPrinted = true;
    }
    return false;
  }

  imuReadErrorPrinted = false;

  ax = IMU.accX() * ACC_G_TO_MSS;
  ay = IMU.accY() * ACC_G_TO_MSS;
  az = IMU.accZ() * ACC_G_TO_MSS;

  gx = IMU.gyrX();
  gy = IMU.gyrY();
  gz = IMU.gyrZ();
  temp_c = IMU.temp();

  return true;
}

static inline int16_t imu_float_to_i16(float v, float scale) {
  float x = roundf(v * scale);
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)x;
}

static inline int16_t imu_temp_c_to_centi(float temp_c) {
  float x = roundf(temp_c * IMU_TEMP_CENTI_SCALE);
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)x;
}

static void imuRetxRingStore(uint32_t seq, uint64_t micros, int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz,
                            int16_t mx, int16_t my, int16_t mz, int16_t temp_centi_c);
void sendSyncRequest();

#if ENABLE_WIFI_SEND
static void imuOfflineQueueReset(void);
static void imuLowRssiQueueReset(void);
#if SLAVE_SYNC_MASTER_DRIVEN
static void slaveStartSyncFromMasterRequest(void);
#endif
#if !SLAVE_SYNC_MASTER_DRIVEN
static void beginSyncRoundNow(const char *reason);
#endif
static void requestImmediateResyncAfterEspNowFail(const char *reason);
static void imuOfflineBatchPush(const imu_batch_packet_t *b);
static void espNowImuSendNotifyOk(void) {
  imuTxFailStreak = 0;
  imuTxNextAttemptMs = 0;
}

static bool imuTxAttemptReady(void) {
  return (int32_t)(millis() - imuTxNextAttemptMs) >= 0;
}

static void imuTxScheduleBackoff(void) {
  imuTxNextAttemptMs = millis() + (unsigned long)IMU_TX_FAIL_RETRY_GAP_MS;
}

static bool imuTxFailResyncEligible(void) {
  if (!connected || !timeSynced) {
    return true;
  }
  if (lastAckTime != 0UL &&
      (unsigned long)(millis() - lastAckTime) <=
          (unsigned long)(CONNECTION_TIMEOUT_MS / 2)) {
    return false;
  }
  return true;
}

static void espNowImuSendNotifyFail(const char *reason,
                                    const imu_batch_packet_t *saveBatch) {
  if (saveBatch != NULL) {
    imuOfflineBatchPush(saveBatch);
  }
  imuTxScheduleBackoff();
  if (!imuTxFailResyncEligible()) {
    static unsigned long s_imuFailHoldLogMs = 0;
    const unsigned long nowMs = millis();
    if (nowMs - s_imuFailHoldLogMs >= 1000UL) {
      s_imuFailHoldLogMs = nowMs;
      Serial.printf(
          "LINK: %s (saved offline, SYNC/ACK OK — retry in %u ms)\n", reason,
          (unsigned)IMU_TX_FAIL_RETRY_GAP_MS);
    }
    return;
  }
  if (imuTxFailStreak < 255U) {
    imuTxFailStreak++;
  }
  if (imuTxFailStreak >= IMU_TX_FAIL_RESYNC_AFTER) {
    imuTxFailStreak = 0;
    requestImmediateResyncAfterEspNowFail(reason);
    return;
  }
  Serial.printf("LINK: %s (saved offline, resync after %u more fail(s))\n",
                reason,
                (unsigned)(IMU_TX_FAIL_RESYNC_AFTER - imuTxFailStreak));
}
#endif
#if ENABLE_IMU_DEDICATED_TASK
static void imuBatchTxQueueDrain(void);
#endif

// ======================== RADIO ========================
#if SLAVE_WIFI_TX_POWER_ADAPTIVE || SLAVE_WIFI_TX_POWER_NORM_QDB > 0
static int8_t s_slaveWifiTxQdb = 0;
static bool s_slaveWifiTxBoosted = false;
static uint32_t s_slaveWifiTxLastAdjMs = 0;

static int8_t slaveWifiClampTxQdb(int8_t qdb) {
  if (qdb < 8) {
    return 8;
  }
  if (qdb > 84) {
    return 84;
  }
  return qdb;
}

static bool slaveWifiApplyTxPowerQdb(int8_t qdb) {
  qdb = slaveWifiClampTxQdb(qdb);
  if (qdb == s_slaveWifiTxQdb) {
    return false;
  }
  const esp_err_t e = esp_wifi_set_max_tx_power(qdb);
  if (e != ESP_OK) {
    return false;
  }
  s_slaveWifiTxQdb = qdb;
  return true;
}

static void slaveWifiTxPowerInit(void) {
  s_slaveWifiTxBoosted = true;
  s_slaveWifiTxLastAdjMs = 0;
  if (slaveWifiApplyTxPowerQdb((int8_t)SLAVE_WIFI_TX_POWER_MAX_QDB)) {
    Serial.printf("[LINK] WiFi init: TX max quarter-dBm=%d (~%.1f dBm), PS=none\n",
                  (int)s_slaveWifiTxQdb, (double)s_slaveWifiTxQdb * 0.25);
  }
#if SLAVE_WIFI_TX_POWER_ADAPTIVE
  if (SLAVE_WIFI_TX_POWER_NORM_QDB < SLAVE_WIFI_TX_POWER_MAX_QDB) {
    s_slaveWifiTxBoosted = false;
    slaveWifiApplyTxPowerQdb((int8_t)SLAVE_WIFI_TX_POWER_NORM_QDB);
  }
#endif
}

static void slaveWifiTxPowerUpdateFromRssi(int8_t rssiDb) {
#if SLAVE_WIFI_TX_POWER_ADAPTIVE
  const uint32_t now = millis();
  if (s_slaveWifiTxLastAdjMs != 0U &&
      (uint32_t)(now - s_slaveWifiTxLastAdjMs) < SLAVE_WIFI_TX_POWER_ADJ_INTERVAL_MS) {
    return;
  }

  bool wantBoost;
  if (rssiDb < (int8_t)SLAVE_WIFI_TX_RSSI_BOOST_DB) {
    wantBoost = true;
  } else if (rssiDb >= (int8_t)SLAVE_WIFI_TX_RSSI_RESTORE_DB) {
    wantBoost = false;
  } else {
    return;
  }

  if (wantBoost == s_slaveWifiTxBoosted) {
    return;
  }

  const int8_t target =
      wantBoost ? (int8_t)SLAVE_WIFI_TX_POWER_BOOST_QDB
                : (int8_t)SLAVE_WIFI_TX_POWER_NORM_QDB;
  if (!slaveWifiApplyTxPowerQdb(target)) {
    return;
  }
  s_slaveWifiTxBoosted = wantBoost;
  s_slaveWifiTxLastAdjMs = now;
  Serial.printf("[LINK] TX power %s: quarter-dBm=%d (~%.1f dBm), RSSI=%d dBm\n",
                wantBoost ? "BOOST" : "NORM", (int)target, (double)target * 0.25,
                (int)rssiDb);
#endif
}

static bool slaveRssiTooWeakForSync(void) {
#if SLAVE_SYNC_RSSI_GATE
  return (g_slaveLastRxRssiValid != 0U) &&
         (g_slaveLastRxRssiDb < (int8_t)SLAVE_SYNC_MIN_RSSI_DB);
#else
  return false;
#endif
}

static void slaveSyncSkipLogRssiWeak(void) {
  static uint32_t s_lastLogMs = 0;
  const uint32_t now = millis();
  if (s_lastLogMs != 0U && (uint32_t)(now - s_lastLogMs) < 3000U) {
    return;
  }
  s_lastLogMs = now;
  if (g_slaveLastRxRssiValid != 0U) {
    Serial.printf(
        "SYNC: bo qua — RSSI yeu (%d dBm < %d dBm)\n",
        (int)g_slaveLastRxRssiDb, (int)SLAVE_SYNC_MIN_RSSI_DB);
  } else {
    Serial.println("SYNC: bo qua — chua co RSSI hop le tu master");
  }
}

static void slaveWifiTxPowerRestoreNorm(void) {
  s_slaveWifiTxBoosted = (SLAVE_WIFI_TX_POWER_NORM_QDB >= SLAVE_WIFI_TX_POWER_BOOST_QDB);
  s_slaveWifiTxLastAdjMs = 0;
  slaveWifiApplyTxPowerQdb((int8_t)SLAVE_WIFI_TX_POWER_MAX_QDB);
#if SLAVE_WIFI_TX_POWER_ADAPTIVE
  if (SLAVE_WIFI_TX_POWER_NORM_QDB < SLAVE_WIFI_TX_POWER_MAX_QDB) {
    s_slaveWifiTxBoosted = false;
    slaveWifiApplyTxPowerQdb((int8_t)SLAVE_WIFI_TX_POWER_NORM_QDB);
  }
#endif
}
#endif

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

  uint8_t ch;
  wifi_second_chan_t sc;
  esp_wifi_get_channel(&ch, &sc);
  Serial.printf("CH=%u%s\n", ch, USE_LONG_RANGE ? " (LR ON)" : "");
#if SLAVE_WIFI_TX_POWER_ADAPTIVE || SLAVE_WIFI_TX_POWER_NORM_QDB > 0
  slaveWifiTxPowerInit();
#endif
}

bool addMasterPeer() {
  if (esp_now_is_peer_exist(masterMAC)) {
    esp_now_del_peer(masterMAC);
  }

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, masterMAC, 6);
  p.channel = CHANNEL;
  p.ifidx = WIFI_IF_STA;
  p.encrypt = false;

  return esp_now_add_peer(&p) == ESP_OK;
}

void disconnectFromMaster(const char *reason) {
  const bool wasTimeSynced = timeSynced;
  connected = false;
  disconnectedSince = millis();
  syncRunning = false;
#if ENABLE_WIFI_SEND
  helloConnectFailCount = 0;
#endif
  lastAckTime = 0;
  clearMasterSlotAssignment();
  lastHelloTime = millis() - (unsigned long)HELLO_RECONNECT_INTERVAL - 1UL;
  lastSyncTime = millis();
  networkDelayUs = 0;
#if ENABLE_WIFI_SEND
  imuLowRssiQueueReset();
#if ENABLE_IMU_DEDICATED_TASK
  imuBatchTxQueueDrain();
#endif
  espnowTxResult = 0;
  if (wasTimeSynced) {
    imuOfflineCaptureMode = true;
    if (syncDriftCompActive() && syncDriftAnchorLocalUs == 0ULL) {
      syncDriftAnchorLocalUs = nowUs();
    }
    Serial.printf(
        "LINK: mat ket noi — ghi IMU offline (toi da %u lo, %u mau)\n",
        (unsigned)IMU_OFFLINE_BATCH_CAP, (unsigned)IMU_OFFLINE_BUFFER_SAMPLES);
  } else {
    imuOfflineCaptureMode = false;
    timeSynced = false;
    globalTimeOffset = 0;
    globalTimeOffsetTarget = 0;
    syncPrevDriftSampleValid = false;
    syncDriftAnchorLocalUs = 0;
    syncDriftLastCompMs = 0;
    syncDriftRecentReset();
    imuOfflineQueueReset();
  }
#else
  timeSynced = false;
  globalTimeOffset = 0;
  globalTimeOffsetTarget = 0;
  syncPrevDriftSampleValid = false;
  syncDriftAnchorLocalUs = 0;
  syncDriftLastCompMs = 0;
  syncDriftRecentReset();
#endif
  g_slaveLastRxRssiValid = 0;
#if SLAVE_WIFI_TX_POWER_ADAPTIVE || SLAVE_WIFI_TX_POWER_NORM_QDB > 0
  slaveWifiTxPowerRestoreNorm();
#endif
  ledOffAt = 0;
  digitalWrite(LED_PIN, LOW);
  Serial.println(reason);
}

#if ENABLE_WIFI_SEND
static void linkResetChipAfterHelloFailures(void) {
  Serial.printf(
      "LINK: %u lan HELLO/ket noi khong thanh cong -> reset chip\n",
      (unsigned)LINK_HELLO_FAIL_RESET_MAX);
  delay(50);
  esp_restart();
}

static void linkHelloConnectAttemptFailed(void) {
  if (helloConnectFailCount < 255) {
    helloConnectFailCount++;
  }
  if (helloConnectFailCount >= LINK_HELLO_FAIL_RESET_MAX) {
    linkResetChipAfterHelloFailures();
  }
}

// Arduino-ESP32 3.x / ESP-IDF 5+: send_cb dùng wifi_tx_info_t; core cũ dùng const uint8_t* MAC.
#if (defined(ARDUINO_ESP32_MAJOR) && (ARDUINO_ESP32_MAJOR >= 3)) ||                        \
    (defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5))
void onEspNowSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
#else
void onEspNowSendCb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnowTxResult = 1U;
    if (connected) {
      lastAckTime = millis();
    }
  } else {
    espnowTxResult = 2U;
  }
}

static bool espNowWaitTxResult(uint32_t waitMs) {
  const uint32_t t0 = millis();
  while (espnowTxResult == 0 && (millis() - t0 < waitMs)) {
    yield();
  }
  if (espnowTxResult == 1U) {
    return true;
  }
  return false;
}

/**
 * Một lần esp_now_send + chờ callback (HELLO/IMU…). Không retry ngay — giảm xung đột RF;
 * IMU fail sẽ dừng stream và mở lại SYNC ở tầng gọi.
 */
static bool espNowSendOnce(const uint8_t *dst, uint8_t *data, size_t len) {
  espnowTxResult = 0;
  esp_err_t err = esp_now_send(dst, data, len);
  if (err != ESP_OK) {
    return false;
  }
  return espNowWaitTxResult(ESPNOW_SEND_WAIT_MS);
}

static bool espNowSendSyncPacket(const uint8_t *dst, packet_t *syncPkt) {
  espnowTxResult = 0;
  const uint64_t tSend = nowUs();
  syncPkt->timestamp = (uint32_t)(tSend / 1000ULL);
  syncPkt->micros_timestamp = tSend;
  syncPkt->request_time = tSend;
  esp_err_t err = esp_now_send(dst, (uint8_t *)syncPkt, sizeof(*syncPkt));
  if (err != ESP_OK) {
    return false;
  }
  return espNowWaitTxResult(ESPNOW_SYNC_SEND_WAIT_MS);
}

static imu_batch_packet_t imuOfflineBatches[IMU_OFFLINE_BATCH_CAP];
static uint8_t imuOfflineBatchCount = 0;
static uint8_t imuOfflineHead = 0;

static void imuOfflineQueueReset(void) {
  imuOfflineBatchCount = 0;
  imuOfflineHead = 0;
}

#if ENABLE_WIFI_SEND
static bool imuOfflineCaptureActive(void) {
  return imuOfflineCaptureMode && timeSynced && !connected &&
         (imuOfflineBatchCount < IMU_OFFLINE_BATCH_CAP);
}

static void imuOfflineCaptureEndOnReconnect(void) {
  if (!imuOfflineCaptureMode) {
    return;
  }
  imuOfflineCaptureMode = false;
  timeSynced = false;
  globalTimeOffset = 0;
  globalTimeOffsetTarget = 0;
}
#else
static bool imuOfflineCaptureActive(void) {
  return false;
}
#endif

static imu_batch_packet_t imuLowRssiQ[SLAVE_IMU_LOW_RSSI_QUEUE_CAP];
static uint8_t imuLowRssiHead = 0;
static uint8_t imuLowRssiCount = 0;

static bool s_imuLowRssiTxActive = false;
static imu_batch_packet_t s_imuLowRssiTxPkt = {};
static uint8_t s_imuLowRssiTxAttempt = 0;
static uint32_t s_imuLowRssiTxNextMs = 0;

static void imuLowRssiQueueReset(void) {
  imuLowRssiHead = 0;
  imuLowRssiCount = 0;
  s_imuLowRssiTxActive = false;
  s_imuLowRssiTxAttempt = 0;
}

static void imuLowRssiEnqueue(const imu_batch_packet_t *b) {
  if (SLAVE_IMU_LOW_RSSI_QUEUE_CAP == 0U) {
    return;
  }
  if (imuLowRssiCount >= (uint8_t)SLAVE_IMU_LOW_RSSI_QUEUE_CAP) {
    imuLowRssiHead =
        (uint8_t)((imuLowRssiHead + 1U) % (uint8_t)SLAVE_IMU_LOW_RSSI_QUEUE_CAP);
    imuLowRssiCount--;
  }
  const uint8_t tail = (uint8_t)(
      (imuLowRssiHead + imuLowRssiCount) % (uint8_t)SLAVE_IMU_LOW_RSSI_QUEUE_CAP);
  imuLowRssiQ[tail] = *b;
  imuLowRssiCount++;
}

/** Đầy thì bỏ batch cũ nhất (FIFO). */
static void imuOfflineBatchPush(const imu_batch_packet_t *b) {
  if (imuOfflineBatchCount >= IMU_OFFLINE_BATCH_CAP) {
    imuOfflineHead = (uint8_t)((imuOfflineHead + 1U) % IMU_OFFLINE_BATCH_CAP);
    imuOfflineBatchCount--;
  }
  uint8_t tail =
      (uint8_t)((imuOfflineHead + imuOfflineBatchCount) % IMU_OFFLINE_BATCH_CAP);
  imuOfflineBatches[tail] = *b;
  imuOfflineBatchCount++;
}

static bool masterSlotValid = false;
static uint8_t masterSyncSlot = 0;
#if ENABLE_IMU_TX_SLOT
static uint8_t imuTxSlotIndex = 0;
static bool imuTxSlotInit = false;
#endif

static void clearMasterSlotAssignment() {
  masterSlotValid = false;
  masterSyncSlot = 0;
#if ENABLE_IMU_TX_SLOT
  imuTxSlotInit = false;
#endif
}

static void applyMasterSlotFromPacket(const packet_t *pkt) {
  if (!(pkt->slot_flags & PACKET_T_SLOT_FLAG_VALID)) {
    return;
  }
  if (pkt->sync_slot < MASTER_SYNC_SLOT_COUNT) {
    masterSyncSlot = pkt->sync_slot;
  }
  masterSlotValid = true;
#if ENABLE_IMU_TX_SLOT
  if (pkt->imu_tx_slot == MASTER_SLOT_UNASSIGNED ||
      pkt->imu_tx_slot >= IMU_TX_SLOT_COUNT) {
    return;
  }
  imuTxSlotIndex = pkt->imu_tx_slot;
  imuTxSlotInit = true;
#if DEBUG_IMU_SLOT_SERIAL
  Serial.printf(
      "[IMU] TX slot=%u/%u (Master cap), SYNC_slot=%u, frame=%lu us\n",
      (unsigned)imuTxSlotIndex, (unsigned)IMU_TX_SLOT_COUNT,
      (unsigned)masterSyncSlot,
      (unsigned long)((uint64_t)IMU_TX_SLOT_COUNT * (uint64_t)IMU_TX_SLOT_US));
#endif
#else
#if DEBUG_IMU_SLOT_SERIAL
  Serial.printf("[IMU] SYNC_slot=%u (Master cap)\n", (unsigned)masterSyncSlot);
#endif
#endif
}

#if ENABLE_IMU_TX_SLOT
static void ensureImuTxSlotIndex() {
  if (imuTxSlotInit) {
    return;
  }
  uint8_t mac[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
  uint16_t h = (uint16_t)mac[4] | ((uint16_t)mac[5] << 8);
  imuTxSlotIndex = (uint8_t)(h % IMU_TX_SLOT_COUNT);
  imuTxSlotInit = true;
#if DEBUG_IMU_SLOT_SERIAL
  Serial.printf(
      "[IMU] TX slot=%u/%u, frame=%lu us (fallback MAC hash)\n",
      (unsigned)imuTxSlotIndex, IMU_TX_SLOT_COUNT,
      (unsigned long)((uint64_t)IMU_TX_SLOT_COUNT * (uint64_t)IMU_TX_SLOT_US));
#endif
}

/** Chờ tới slot của this slave (đồng hồ master qua getGlobalTimeUs). */
static void waitUntilMyTxSlot() {
  if (!timeSynced) {
    return;
  }
  ensureImuTxSlotIndex();
  const uint64_t frameUs =
      (uint64_t)IMU_TX_SLOT_COUNT * (uint64_t)IMU_TX_SLOT_US;
  uint64_t tStart = (uint64_t)getGlobalTimeUs();
  for (;;) {
    uint64_t g = (uint64_t)getGlobalTimeUs();
    uint64_t tmod = g % frameUs;
    uint32_t slot = (uint32_t)(tmod / (uint64_t)IMU_TX_SLOT_US);
    if (slot == (uint32_t)imuTxSlotIndex) {
      break;
    }
    if ((g - tStart) > (uint64_t)IMU_TX_SLOT_MAX_SPIN_US) {
      break;
    }
    yield();
  }
}
#endif

static uint32_t syncPeriodicMinGapMs(void) {
  return (syncPeriodicIntervalMs > SYNC_PERIODIC_MAX_OFFSET_MS)
             ? (syncPeriodicIntervalMs - SYNC_PERIODIC_MAX_OFFSET_MS)
             : 0U;
}

static void syncPeriodicIntervalBumpAfterDone(void) {
#if SYNC_INTERVAL_STEP_MS == 0U
  return;
#endif
  if (syncPeriodicIntervalMs >= (uint32_t)SYNC_INTERVAL_MS) {
    return;
  }
  uint32_t next = syncPeriodicIntervalMs + SYNC_INTERVAL_STEP_MS;
  if (next > (uint32_t)SYNC_INTERVAL_MS) {
    next = (uint32_t)SYNC_INTERVAL_MS;
  }
  if (next == syncPeriodicIntervalMs) {
    return;
  }
  syncPeriodicIntervalMs = next;
  Serial.printf("[SYNC] periodic interval -> %lu ms (max %d ms)\n",
                (unsigned long)syncPeriodicIntervalMs, SYNC_INTERVAL_MS);
}

static uint8_t syncPeriodicMacSlotIndex() {
  if (masterSlotValid && masterSyncSlot < MASTER_SYNC_SLOT_COUNT) {
    return masterSyncSlot;
  }
  uint8_t mac[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
  return (uint8_t)(mac[5] / SYNC_PERIODIC_MAC_SLOT_DIVISOR);
}

static uint32_t syncPeriodicSlotOffsetMs() {
  return (uint32_t)syncPeriodicMacSlotIndex() *
         (uint32_t)SYNC_PERIODIC_SLOT_SPACING_MS;
}

static bool syncPeriodicSlotReady() {
  if (!timeSynced) {
    return true;
  }
  const uint32_t offsetMs = syncPeriodicSlotOffsetMs();
  const uint32_t intervalMs = syncPeriodicIntervalMs;
  const uint32_t phaseMs =
      (uint32_t)(((uint64_t)getGlobalTimeUs() / 1000ULL) % (uint64_t)intervalMs);
  return phaseMs >= offsetMs &&
         phaseMs < (offsetMs + (uint32_t)SYNC_PERIODIC_SLOT_WINDOW_MS);
}

/**
 * Một bước / lần gọi: gửi lô từ đệm RSSI thấp — tối đa SLAVE_IMU_LOW_RSSI_SENDS lần,
 * cách SLAVE_IMU_LOW_RSSI_SEND_GAP_MS; dừng sớm khi gửi OK (tránh trùng seq trên master).
 */
static void imuLowRssiService(void) {
  if (!connected || !timeSynced) {
    imuLowRssiQueueReset();
    return;
  }
  if (syncRunning) {
    return;
  }
  if (!imuTxAttemptReady()) {
    return;
  }
  const uint32_t now = millis();
  if (!s_imuLowRssiTxActive) {
    if (imuLowRssiCount == 0U) {
      return;
    }
    s_imuLowRssiTxPkt = imuLowRssiQ[imuLowRssiHead];
    imuLowRssiHead =
        (uint8_t)((imuLowRssiHead + 1U) % (uint8_t)SLAVE_IMU_LOW_RSSI_QUEUE_CAP);
    imuLowRssiCount--;
    s_imuLowRssiTxActive = true;
    s_imuLowRssiTxAttempt = 0U;
    s_imuLowRssiTxNextMs = now;
  }
  if ((int32_t)(now - s_imuLowRssiTxNextMs) < 0) {
    return;
  }
  const size_t wire = IMU_BATCH_HEADER_SIZE +
                      (size_t)(IMU_BATCH_SAMPLES - 1) * sizeof(imu_batch_delta_t);
#if ENABLE_IMU_TX_SLOT
  waitUntilMyTxSlot();
#endif
  if (espNowSendOnce(masterMAC, (uint8_t *)&s_imuLowRssiTxPkt, wire)) {
    espNowImuSendNotifyOk();
    s_imuLowRssiTxActive = false;
    return;
  }
  s_imuLowRssiTxAttempt++;
  if (s_imuLowRssiTxAttempt < (uint8_t)SLAVE_IMU_LOW_RSSI_SENDS) {
    s_imuLowRssiTxNextMs = now + (uint32_t)SLAVE_IMU_LOW_RSSI_SEND_GAP_MS;
    return;
  }
  s_imuLowRssiTxActive = false;
  espNowImuSendNotifyFail("ESP-NOW IMU low-RSSI retry failed",
                          &s_imuLowRssiTxPkt);
}

static void drainImuOfflineQueue() {
  if (!imuStreamReady() || syncRunning || imuOfflineBatchCount == 0U ||
      !imuTxAttemptReady()) {
    return;
  }
#if ENABLE_IMU_TX_SLOT
  waitUntilMyTxSlot();
#endif
  imu_batch_packet_t *b = &imuOfflineBatches[imuOfflineHead];
  size_t wire = IMU_BATCH_HEADER_SIZE +
                (size_t)(IMU_BATCH_SAMPLES - 1) * sizeof(imu_batch_delta_t);
  if (espNowSendOnce(masterMAC, (uint8_t *)b, wire)) {
    espNowImuSendNotifyOk();
    imuOfflineHead =
        (uint8_t)((imuOfflineHead + 1U) % IMU_OFFLINE_BATCH_CAP);
    imuOfflineBatchCount--;
  } else {
    espNowImuSendNotifyFail("ESP-NOW offline IMU batch failed", NULL);
  }
}

#if ENABLE_IMU_DEDICATED_TASK
static void imuBatchTxQueueDrain(void) {
  if (imuBatchTxQueue == NULL) {
    return;
  }
  imu_batch_packet_t discard;
  while (xQueueReceive(imuBatchTxQueue, &discard, 0) == pdPASS) {
    /* bỏ lô chờ gửi — tránh gửi timestamp theo offset cũ sau reconnect */
  }
}
#endif

#if SLAVE_SYNC_MASTER_DRIVEN
static void slaveStartSyncFromMasterRequest(void) {
  if (!connected || syncRunning) {
    return;
  }
  if (slaveRssiTooWeakForSync()) {
    slaveSyncSkipLogRssiWeak();
    return;
  }
  Serial.println("SYNC: master TSRQ -> starting round...");
  timeSynced = false;
  globalTimeOffset = 0;
  globalTimeOffsetTarget = 0;
  syncRunning = true;
  lastSyncTime = 0;
  lastSyncSampleSentMs = millis() - (unsigned long)SYNC_SAMPLE_GAP_MS;
  syncPrevDriftSampleValid = false;
  syncDriftAnchorLocalUs = 0;
  syncDriftRecentReset();
  imuTxFailStreak = 0;
  syncLastSendOk = true;
#if ENABLE_IMU_DEDICATED_TASK
  imuBatchTxQueueDrain();
#endif
  sendSyncRequest();
}
#endif

#if !SLAVE_SYNC_MASTER_DRIVEN
static void beginSyncRoundNow(const char *reason) {
  if (!connected || syncRunning) {
    return;
  }
  if (slaveRssiTooWeakForSync()) {
    slaveSyncSkipLogRssiWeak();
    return;
  }
  Serial.printf("LINK: %s\n", reason);
  Serial.println("Starting SYNC (TIME required before IMU)...");
  timeSynced = false;
  globalTimeOffset = 0;
  globalTimeOffsetTarget = 0;
  syncRunning = true;
  lastSyncTime = 0;
  lastSyncSampleSentMs = millis() - (unsigned long)SYNC_SAMPLE_GAP_MS;
  syncPrevDriftSampleValid = false;
  syncDriftAnchorLocalUs = 0;
  syncDriftRecentReset();
  imuTxFailStreak = 0;
  syncLastSendOk = true;
#if ENABLE_IMU_DEDICATED_TASK
  imuBatchTxQueueDrain();
#endif
  sendSyncRequest();
}
#endif

static void requestImmediateResyncAfterEspNowFail(const char *reason) {
#if SLAVE_SYNC_MASTER_DRIVEN
  Serial.printf("LINK: %s -> cho master TSRQ (khong tu gui SYNC)\n", reason);
  timeSynced = false;
  syncRunning = false;
  syncPrevDriftSampleValid = false;
  syncDriftAnchorLocalUs = 0;
  syncDriftRecentReset();
  imuTxFailStreak = 0;
#if ENABLE_IMU_DEDICATED_TASK
  imuBatchTxQueueDrain();
#endif
#else
  beginSyncRoundNow(reason);
#endif
}
#endif

// ======================== SEND ========================
void sendHello() {
  packet_t helloPacket = {};
  strcpy(helloPacket.type, "HELLO");

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, helloPacket.mac);
#else
  esp_read_mac(helloPacket.mac, ESP_MAC_WIFI_STA);
#endif

  helloPacket.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  helloPacket.timestamp = nowMs();
  helloPacket.micros_timestamp = nowUs();
  helloPacket.timezone_offset = timezoneOffset;
  helloPacket.request_time = 0;
  helloPacket.response_time = 0;
  helloPacket.slave_adc_raw = 0;
  helloPacket.last_sync_rtt_us = 0;

#if ENABLE_WIFI_SEND
  if (!connected) {
    Serial.println(
        "LINK: dang gui HELLO toi master (chua ket noi), doi ACK...");
    (void)espNowSendOnce(masterMAC, (uint8_t *)&helloPacket, sizeof(helloPacket));
    linkHelloConnectAttemptFailed();
  } else {
    (void)espNowSendOnce(masterMAC, (uint8_t *)&helloPacket, sizeof(helloPacket));
  }
#endif
  lastHelloTime = millis();
}

void sendSyncRequest() {
  if (slaveRssiTooWeakForSync()) {
    return;
  }
  packet_t syncRequest = {};
  strcpy(syncRequest.type, "SYNC");

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, syncRequest.mac);
#else
  esp_read_mac(syncRequest.mac, ESP_MAC_WIFI_STA);
#endif

  syncRequest.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  syncRequest.timezone_offset = timezoneOffset;
  syncRequest.response_time = 0;
  syncRequest.slave_adc_raw = readVbatAdcRaw();
  syncRequest.last_sync_rtt_us = lastCompletedSyncBestRttUs;

#if ENABLE_WIFI_SEND
  syncLastSendOk = espNowSendSyncPacket(masterMAC, &syncRequest);
#else
  syncLastSendOk = true;
#endif
}

void printImuDataSerial(const uint8_t *mac, uint64_t micros, uint32_t seq,
                        float ax, float ay, float az, float gx, float gy,
                        float gz, float temp_c) {
  Serial.printf(
      "IMU,%02X:%02X:%02X:%02X:%02X:%02X,%llu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
      (unsigned long long)micros, (unsigned long)seq, ax, ay, az, gx, gy, gz,
      temp_c);
}

/** Gửi một lô IMU đã đóng gói (ESP-NOW hoặc offline). */
static void imuTransmitBatchPacket(const imu_batch_packet_t *b) {
  size_t wire = IMU_BATCH_HEADER_SIZE +
                (size_t)(IMU_BATCH_SAMPLES - 1) * sizeof(imu_batch_delta_t);
#if ENABLE_WIFI_SEND
  if (!imuStreamReady()) {
    return;
  }
  if (!imuTxAttemptReady()) {
    imuOfflineBatchPush(b);
    return;
  }
  const bool rssiLow = (g_slaveLastRxRssiValid != 0U) &&
                       (g_slaveLastRxRssiDb < (int8_t)SLAVE_IMU_LOW_RSSI_DB);
  if (rssiLow) {
    imuLowRssiEnqueue(b);
    imuLowRssiService();
    return;
  }
#if ENABLE_IMU_TX_SLOT
  waitUntilMyTxSlot();
#endif
  if (espNowSendOnce(masterMAC, (uint8_t *)b, wire)) {
    espNowImuSendNotifyOk();
  } else {
    espNowImuSendNotifyFail("ESP-NOW IMU batch failed", b);
  }
#endif
}

/**
 * use_tx_queue=true: đủ lô → xQueueSend (loop gửi RF); không chặn SYNC.
 * use_tx_queue=false: gửi ngay trong loop (chế độ cũ).
 * readImuSample thất bại: không tăng imuSampleSeq (khớp BNO055).
 */
static bool imuSampleProcessOne(bool use_tx_queue) {
  static int16_t batch_ax[IMU_BATCH_SAMPLES];
  static int16_t batch_ay[IMU_BATCH_SAMPLES];
  static int16_t batch_az[IMU_BATCH_SAMPLES];
  static int16_t batch_gx[IMU_BATCH_SAMPLES];
  static int16_t batch_gy[IMU_BATCH_SAMPLES];
  static int16_t batch_gz[IMU_BATCH_SAMPLES];
  static uint64_t batch_micros[IMU_BATCH_SAMPLES];
  static uint8_t batch_fill = 0;

  if (!imuSampleAllowed()) {
    batch_fill = 0;
    return false;
  }

  digitalWrite(PULSE_PIN, HIGH);
  float ax, ay, az;
  float gx, gy, gz;
  float temp_c = 0.0f;
  const bool imuReadOk = readImuSample(ax, ay, az, gx, gy, gz, temp_c);
  digitalWrite(PULSE_PIN, LOW);
  if (!imuReadOk) {
    return false;
  }

  applyAccelCalibration(ax, ay, az);
  applyGyroCalibration(gx, gy, gz);

  if (!imuStreamReady() && !imuOfflineCaptureActive()) {
    batch_fill = 0;
    return true;
  }

  const uint64_t tu = imuTimestampForSampleLoggedUs();

  batch_ax[batch_fill] = imu_float_to_i16(ax, IMU_RAW_ACC_SCALE);
  batch_ay[batch_fill] = imu_float_to_i16(ay, IMU_RAW_ACC_SCALE);
  batch_az[batch_fill] = imu_float_to_i16(az, IMU_RAW_ACC_SCALE);
  batch_gx[batch_fill] = imu_float_to_i16(gx, IMU_RAW_GYRO_SCALE);
  batch_gy[batch_fill] = imu_float_to_i16(gy, IMU_RAW_GYRO_SCALE);
  batch_gz[batch_fill] = imu_float_to_i16(gz, IMU_RAW_GYRO_SCALE);
  batch_micros[batch_fill] = tu;
  batch_fill++;

#if SERIAL_IMU_OUTPUT
  uint8_t macDbg[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, macDbg);
#else
  esp_read_mac(macDbg, ESP_MAC_WIFI_STA);
#endif
  printImuDataSerial(macDbg, tu, imuSampleSeq, ax, ay, az, gx, gy, gz,
                     temp_c);
#endif

  imuSampleSeq++;
  imuPacketCount++;

  {
    const uint8_t bi = (uint8_t)(batch_fill - 1U);
    imuRetxRingStore(
        imuSampleSeq - 1U, tu, batch_ax[bi], batch_ay[bi], batch_az[bi],
        batch_gx[bi], batch_gy[bi], batch_gz[bi],
        IMU_MAG_INT16_NONE, IMU_MAG_INT16_NONE, IMU_MAG_INT16_NONE,
        imu_temp_c_to_centi(temp_c));
  }

  if (batch_fill < IMU_BATCH_SAMPLES) {
    if (!use_tx_queue) {
      yield();
    }
    return true;
  }

  imu_batch_packet_t b = {};
  memcpy(b.type, "IMU_RAWB\0", 8);
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, b.mac);
#else
  esp_read_mac(b.mac, ESP_MAC_WIFI_STA);
#endif
  b.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  b.count = IMU_BATCH_SAMPLES;
  b.sample_seq0 = imuSampleSeq - IMU_BATCH_SAMPLES;
  b.micros_t0 = batch_micros[0];
  b.ax0 = batch_ax[0];
  b.ay0 = batch_ay[0];
  b.az0 = batch_az[0];
  b.gx0 = batch_gx[0];
  b.gy0 = batch_gy[0];
  b.gz0 = batch_gz[0];
  b.mx0 = IMU_MAG_INT16_NONE;
  b.my0 = IMU_MAG_INT16_NONE;
  b.mz0 = IMU_MAG_INT16_NONE;
  b.temp_centi_c = imu_temp_c_to_centi(temp_c);

  for (int i = 1; i < IMU_BATCH_SAMPLES; i++) {
    uint64_t prev = batch_micros[i - 1];
    uint64_t cur = batch_micros[i];
    uint64_t d = (cur >= prev) ? (cur - prev) : 0ULL;
    if (d > 65535ULL) {
      d = 65535ULL;
    }
    b.rest[i - 1].delta_us_from_prev = (uint16_t)d;
    b.rest[i - 1].ax = batch_ax[i];
    b.rest[i - 1].ay = batch_ay[i];
    b.rest[i - 1].az = batch_az[i];
    b.rest[i - 1].gx = batch_gx[i];
    b.rest[i - 1].gy = batch_gy[i];
    b.rest[i - 1].gz = batch_gz[i];
    b.rest[i - 1].mx = IMU_MAG_INT16_NONE;
    b.rest[i - 1].my = IMU_MAG_INT16_NONE;
    b.rest[i - 1].mz = IMU_MAG_INT16_NONE;
  }

  if (imuOfflineCaptureActive()) {
    imuOfflineBatchPush(&b);
  } else if (use_tx_queue) {
#if ENABLE_IMU_DEDICATED_TASK
    if (imuBatchTxQueue == NULL ||
        xQueueSend(imuBatchTxQueue, &b, 0) != pdPASS) {
#if ENABLE_WIFI_SEND
      imuOfflineBatchPush(&b);
#else
      (void)b;
#endif
    }
#endif
  } else {
    imuTransmitBatchPacket(&b);
  }

  batch_fill = 0;
  if (!use_tx_queue) {
    yield();
  }
  return true;
}

#if ENABLE_IMU_DEDICATED_TASK
#if IMU_SAMPLE_ALIGN_MASTER_10MS_GRID
/**
 * Chờ tới mốc pha trên timeline master:
 * getGlobalTimeUs() % period == IMU_SAMPLE_GRID_PHASE_US (0 = biên lưới như cũ).
 */
static void imuDelayUntilNextMaster10msGrid(void) {
  if (!timeSynced) {
    return;
  }
  int64_t g64 = getGlobalTimeUs();
  if (g64 < 0) {
    return;
  }
  const uint64_t T = (uint64_t)IMU_LOOP_PERIOD_US;
  const uint64_t P = (uint64_t)IMU_SAMPLE_GRID_PHASE_US;
  const uint64_t g = (uint64_t)g64;
  const uint64_t r = g % T;
  uint64_t w;
  if (P == 0ULL) {
    w = (r == 0ULL) ? 0ULL : (T - r);
  } else {
    if (r < P) {
      w = P - r;
    } else if (r == P) {
      w = 0ULL;
    } else {
      w = T - r + P;
    }
  }
  if (w > T + T / 2U) {
    return;
  }
  if (w > 0ULL) {
    delayMicroseconds((uint32_t)w);
  }
}
#endif

static void imuSampleTask(void* arg) {
  (void)arg;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(IMU_LOOP_PERIOD_US / 1000);
  for (;;) {
#if IMU_SAMPLE_ALIGN_MASTER_10MS_GRID
    if (!timeSynced) {
      vTaskDelayUntil(&last, period);
    } else {
      imuDelayUntilNextMaster10msGrid();
    }
#else
    vTaskDelayUntil(&last, period);
#endif
    (void)imuSampleProcessOne(true);
  }
}

static void imuDrainBatchTxQueue(void) {
  if (imuBatchTxQueue == NULL || syncRunning || !imuTxAttemptReady()) {
    return;
  }
  imu_batch_packet_t b;
  if (xQueueReceive(imuBatchTxQueue, &b, 0) == pdPASS) {
    imuTransmitBatchPacket(&b);
  }
}
#endif

// Trả về true nếu đã đọc được một mẫu IMU (chế độ loop cũ). Chế độ task: queue lỗi → fallback loop.
bool sendImuData() {
#if ENABLE_IMU_DEDICATED_TASK
  if (imuBatchTxQueue == NULL) {
    return imuSampleProcessOne(false);
  }
  return false;
#else
  return imuSampleProcessOne(false);
#endif
}

static void imuRetxRingStore(uint32_t seq, uint64_t micros, int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz,
                            int16_t mx, int16_t my, int16_t mz, int16_t temp_centi_c) {
  (void)mx;
  (void)my;
  (void)mz;
  size_t i = (size_t)(seq % IMU_RETX_RING_CAP);
  imu_retx_slot_t *s = &imuRetxRing[i];
  s->seq = seq;
  s->micros = micros;
  s->ax = ax;
  s->ay = ay;
  s->az = az;
  s->gx = gx;
  s->gy = gy;
  s->gz = gz;
  s->mx = IMU_MAG_INT16_NONE;
  s->my = IMU_MAG_INT16_NONE;
  s->mz = IMU_MAG_INT16_NONE;
  s->temp_centi_c = temp_centi_c;
}

static bool imuRetxRingGet(uint32_t seq, imu_retx_slot_t *out) {
  size_t i = (size_t)(seq % IMU_RETX_RING_CAP);
  if (imuRetxRing[i].seq == seq) {
    *out = imuRetxRing[i];
    return true;
  }
  return false;
}

static void imuRetxSendBatchFromSlots(const imu_retx_slot_t *blk, uint8_t n) {
  if (n < 1U || n > (uint8_t)IMU_BATCH_SAMPLES) {
    return;
  }
  imu_batch_packet_t b = {};
  memcpy(b.type, "IMU_RTXB", 8);
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, b.mac);
#else
  esp_read_mac(b.mac, ESP_MAC_WIFI_STA);
#endif
  b.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
  b.count = n;
  b.sample_seq0 = blk[0].seq;
  b.micros_t0 = blk[0].micros;
  b.ax0 = blk[0].ax;
  b.ay0 = blk[0].ay;
  b.az0 = blk[0].az;
  b.gx0 = blk[0].gx;
  b.gy0 = blk[0].gy;
  b.gz0 = blk[0].gz;
  b.mx0 = IMU_MAG_INT16_NONE;
  b.my0 = IMU_MAG_INT16_NONE;
  b.mz0 = IMU_MAG_INT16_NONE;
  b.temp_centi_c = blk[n - 1U].temp_centi_c;
  for (uint8_t i = 1; i < n; i++) {
    uint64_t prev = blk[i - 1U].micros;
    uint64_t cur = blk[i].micros;
    uint64_t d = (cur >= prev) ? (cur - prev) : 0ULL;
    if (d > 65535ULL) {
      d = 65535ULL;
    }
    b.rest[i - 1U].delta_us_from_prev = (uint16_t)d;
    b.rest[i - 1U].ax = blk[i].ax;
    b.rest[i - 1U].ay = blk[i].ay;
    b.rest[i - 1U].az = blk[i].az;
    b.rest[i - 1U].gx = blk[i].gx;
    b.rest[i - 1U].gy = blk[i].gy;
    b.rest[i - 1U].gz = blk[i].gz;
    b.rest[i - 1U].mx = IMU_MAG_INT16_NONE;
    b.rest[i - 1U].my = IMU_MAG_INT16_NONE;
    b.rest[i - 1U].mz = IMU_MAG_INT16_NONE;
  }
  const size_t wire = IMU_BATCH_HEADER_SIZE + (size_t)(n - 1U) * sizeof(imu_batch_delta_t);
#if ENABLE_WIFI_SEND
  if (!imuStreamReady()) {
    return;
  }
#endif
#if ENABLE_IMU_TX_SLOT && ENABLE_WIFI_SEND
  waitUntilMyTxSlot();
#endif
#if ENABLE_WIFI_SEND
  const bool ok = espNowSendOnce(masterMAC, (uint8_t *)&b, wire);
  Serial.printf(
      "IMU_RETX,TX, seq0=%lu, n=%u, wire=%u, esp_ok=%d\n",
      (unsigned long)blk[0].seq, (unsigned)n, (unsigned)wire, ok ? 1 : 0);
  if (!ok) {
    espNowImuSendNotifyFail("ESP-NOW IMU_RETX failed", &b);
  } else {
    espNowImuSendNotifyOk();
  }
#endif
}

static void imuHandleLostRequest(const imu_lost_request_t *rq) {
  uint8_t me[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, me);
#else
  esp_read_mac(me, ESP_MAC_WIFI_STA);
#endif
  if (memcmp(rq->target_mac, me, 6) != 0) {
    return;
  }
  const uint32_t our_node = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFU);
  if (rq->node_id != 0U && rq->node_id != our_node) {
    Serial.printf(
        "IMU_RETX,SKIP, node: req=%lu me=%lu\n", (unsigned long)rq->node_id,
        (unsigned long)our_node);
    return;
  }
  if (rq->seq_count == 0U) {
    Serial.println("IMU_RETX,SKIP, count=0");
    return;
  }
#if !ENABLE_WIFI_SEND
  Serial.println("IMU_RETX,SKIP, ENABLE_WIFI_SEND=0");
  return;
#else
  if (!connected) {
    Serial.println("IMU_RETX,SKIP, not connected to master");
    return;
  }
  if (!timeSynced) {
    Serial.println("IMU_RETX,SKIP, not time synced yet");
    return;
  }
  const uint32_t s0 = rq->seq_first;
  const uint32_t nc = rq->seq_count;
  Serial.printf(
      "IMU_RETX,START, seq %lu..%lu (n=%lu) node_id=%lu\n", (unsigned long)s0,
      (unsigned long)(s0 + nc - 1U), (unsigned long)nc, (unsigned long)our_node);

  imu_retx_slot_t block[IMU_BATCH_SAMPLES];
  uint8_t nfill = 0;
  uint32_t n_hit = 0;
  uint32_t n_miss = 0;
  for (uint32_t k = 0; k < nc; k++) {
    const uint32_t sseq = s0 + k;
    imu_retx_slot_t one;
    if (!imuRetxRingGet(sseq, &one)) {
      n_miss++;
      if (nfill > 0) {
        imuRetxSendBatchFromSlots(block, nfill);
        nfill = 0;
      }
      continue;
    }
    n_hit++;
    if (nfill > 0 && one.seq != (uint32_t)block[nfill - 1U].seq + 1U) {
      imuRetxSendBatchFromSlots(block, nfill);
      nfill = 0;
    }
    block[nfill++] = one;
    if (nfill == (uint8_t)IMU_BATCH_SAMPLES) {
      imuRetxSendBatchFromSlots(block, nfill);
      nfill = 0;
    }
  }
  if (nfill > 0) {
    imuRetxSendBatchFromSlots(block, nfill);
  }
  Serial.printf(
      "IMU_RETX,DONE, in_ring=%lu, not_in_buffer=%lu (req n=%lu)\n",
      (unsigned long)n_hit, (unsigned long)n_miss, (unsigned long)nc);
#endif
}

// ======================== RECV ========================
static void syncSkipLogRttHigh(int64_t rttUs) {
  static uint32_t s_lastLogMs = 0;
  const uint32_t now = millis();
  if (s_lastLogMs != 0U && (uint32_t)(now - s_lastLogMs) < 3000U) {
    return;
  }
  s_lastLogMs = now;
  Serial.printf("SYNC: bo qua — RTT %lld us >= %lld us\n", (long long)rttUs,
                (long long)SYNC_ACCEPT_MAX_RTT_US);
}

static void syncRoundFinish(int64_t bestOff, int64_t bestRtt, int timezone,
                            uint64_t t3, uint64_t t4) {
  const int64_t prevOffset = globalTimeOffset;

  globalTimeOffset = bestOff;
  globalTimeOffsetTarget = bestOff;
  networkDelayUs = bestRtt / 2;
  timeSynced = true;
  timezoneOffset = timezone;
#if ENABLE_WIFI_SEND
  imuTxFailStreak = 0;
  syncLastSendOk = true;
  imuTxNextAttemptMs = 0;
#endif

  lastCompletedSyncBestRttUs =
      (bestRtt > (int64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)bestRtt;

  Serial.printf(
      "SYNC DONE | RTT=%lld us (< %lld us) | delay~%lld us | offset=%lld us "
      "| prev_offset=%lld us | delta=%lld us\n",
      (long long)bestRtt, (long long)SYNC_ACCEPT_MAX_RTT_US,
      (long long)networkDelayUs, (long long)globalTimeOffset,
      (long long)prevOffset, (long long)(globalTimeOffset - prevOffset));

  {
    const uint64_t slv_now = nowUs();
    Serial.printf(
        "SYNC_CLOCK master_t3_us=%llu slave_t4_rx_us=%llu slave_local_now_us=%llu\n",
        (unsigned long long)t3, (unsigned long long)t4,
        (unsigned long long)slv_now);

    if (syncPrevDriftSampleValid) {
      const uint64_t d_local = slv_now - syncPrevDriftLocalUs;
      const int64_t d_offset = bestOff - syncPrevDriftOffsetUs;
      const double ppm = (d_local >= SYNC_DRIFT_MIN_SPAN_US)
                             ? ((double)d_offset * 1e6 / (double)d_local)
                             : 0.0;
      const bool have_drift_span = (d_local >= SYNC_DRIFT_MIN_SPAN_US);
      const bool ppm_ok =
          have_drift_span && (fabs(ppm) <= SYNC_DRIFT_MAX_ABS_PPM);
      if (ppm_ok) {
        if (!syncDriftFilteredValid) {
          syncDriftFilteredPpm = ppm;
          syncDriftFilteredValid = true;
        } else {
          syncDriftFilteredPpm =
              syncDriftFilteredPpm * (1.0 - SYNC_DRIFT_FILTER_ALPHA) +
              ppm * SYNC_DRIFT_FILTER_ALPHA;
        }
        if (syncDriftAcceptedCount < 255U) {
          syncDriftAcceptedCount++;
        }
        syncDriftRecentAdd(ppm);
        maybeSaveStableSyncDriftToNvs();
      }
      Serial.printf(
          "SYNC_DRIFT_OFFSET: d_local_us=%llu d_offset_us=%lld "
          "raw_ppm=%.3f accepted=%d filtered_ppm=%.3f accepted_count=%u comp=%d "
          "(offset increases when slave clock is slower)\n",
          (unsigned long long)d_local, (long long)d_offset, ppm,
          ppm_ok ? 1 : 0,
          syncDriftFilteredValid ? syncDriftFilteredPpm : 0.0,
          (unsigned)syncDriftAcceptedCount,
#if SYNC_DRIFT_COMP_ENABLE
          (syncDriftFilteredValid &&
           syncDriftAcceptedCount >= (uint8_t)SYNC_DRIFT_COMP_MIN_ACCEPTED)
              ? 1
              : 0);
#else
          0);
#endif
    }
    syncPrevDriftLocalUs = slv_now;
    syncPrevDriftOffsetUs = bestOff;
    syncPrevDriftSampleValid = true;
    syncDriftAnchorLocalUs = slv_now;
    syncDriftLastCompMs = millis();
  }

  syncRunning = false;
  lastSyncTime = millis();
  syncPeriodicIntervalBumpAfterDone();
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (info != nullptr && info->rx_ctrl != nullptr) {
    g_slaveLastRxRssiDb = info->rx_ctrl->rssi;
    g_slaveLastRxRssiValid = 1U;
#if SLAVE_WIFI_TX_POWER_ADAPTIVE
    slaveWifiTxPowerUpdateFromRssi(info->rx_ctrl->rssi);
#endif
  }

  if (len == (int)IMU_LOST_REQUEST_SIZE) {
    imu_lost_request_t lq;
    memcpy(&lq, incomingData, sizeof(lq));
    if (memcmp(lq.type, "IMU_LOST", 8) == 0) {
      imuHandleLostRequest(&lq);
      return;
    }
  }

  const int calibFull = (int)sizeof(accel_calib_packet_t);
  if (len == calibFull || len == ACCEL_CALIB_PACKET_LEGACY_SIZE) {
    const bool fullPkt = (len == calibFull);
    accel_calib_packet_t calibPkt = {};
    memcpy(&calibPkt, incomingData, (size_t)(fullPkt ? calibFull : ACCEL_CALIB_PACKET_LEGACY_SIZE));

    /* Master gửi CALGET → trả về thông số đang dùng (RAM/NVS), không đổi NVS. */
    if (strcmp(calibPkt.type, "CALGET") == 0) {
      if (!fullPkt) {
        return;
      }
      accel_calib_packet_t out = {};
      strcpy(out.type, "CALREP");
#if ESP_IDF_VERSION_MAJOR >= 5
      esp_wifi_get_mac(WIFI_IF_STA, out.mac);
#else
      esp_read_mac(out.mac, ESP_MAC_WIFI_STA);
#endif
      out.node_id = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
      out.timestamp = nowMs();
      out.micros_timestamp = nowUs();
      out.bias[0] = accelBias[0];
      out.bias[1] = accelBias[1];
      out.bias[2] = accelBias[2];
      out.scale[0] = accelScale[0];
      out.scale[1] = accelScale[1];
      out.scale[2] = accelScale[2];
      out.global_scale = accelGlobalScale;
      out.save_to_nvs = 0;
      out.enabled = accelCalibEnabled ? 1 : 0;
      out.gyro_bias[0] = gyroBias[0];
      out.gyro_bias[1] = gyroBias[1];
      out.gyro_bias[2] = gyroBias[2];
#if ENABLE_WIFI_SEND
      esp_now_send(masterMAC, (uint8_t *)&out, sizeof(out));
#endif
      Serial.printf(
          "CALGET_TX,%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
          accelCalibEnabled ? "ENABLED" : "DISABLED",
          accelBias[0], accelBias[1], accelBias[2],
          accelScale[0], accelScale[1], accelScale[2],
          accelGlobalScale,
          gyroBias[0], gyroBias[1], gyroBias[2]);
      return;
    }

    if (strcmp(calibPkt.type, "CALIB") == 0) {
      if (calibPkt.enabled != 0) {
        accelBias[0] = calibPkt.bias[0];
        accelBias[1] = calibPkt.bias[1];
        accelBias[2] = calibPkt.bias[2];
        accelScale[0] = calibPkt.scale[0];
        accelScale[1] = calibPkt.scale[1];
        accelScale[2] = calibPkt.scale[2];
        accelGlobalScale = calibPkt.global_scale;
        accelCalibEnabled = true;
        if (fullPkt) {
          gyroBias[0] = calibPkt.gyro_bias[0];
          gyroBias[1] = calibPkt.gyro_bias[1];
          gyroBias[2] = calibPkt.gyro_bias[2];
        }
      } else {
        resetAccelCalibration();
      }

      bool saveOk = true;
      if (calibPkt.save_to_nvs != 0) {
        saveOk = saveAccelCalibrationToNvs();
      }

      Serial.printf(
          "CALIB_RX,%s,%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
          saveOk ? "OK" : "FAIL",
          accelCalibEnabled ? "ENABLED" : "DISABLED",
          accelBias[0], accelBias[1], accelBias[2],
          accelScale[0], accelScale[1], accelScale[2],
          accelGlobalScale,
          gyroBias[0], gyroBias[1], gyroBias[2]);
      sendCalibAck(saveOk);
      return;
    }
  }

  if (len < (int)PACKET_T_LEGACY_SIZE) {
    return;
  }

  packet_t pkt = {};
  memcpy(&pkt, incomingData, (size_t)len);
  if (len < (int)sizeof(packet_t)) {
    pkt.imu_tx_slot = MASTER_SLOT_UNASSIGNED;
    pkt.sync_slot = MASTER_SLOT_UNASSIGNED;
    pkt.slot_flags = 0;
  }

  if (strcmp(pkt.type, "ACK") == 0) {
#if ENABLE_IMU_TX_SLOT
    applyMasterSlotFromPacket(&pkt);
#endif
#if ENABLE_WIFI_SEND
    espnowTxResult = 0;
#endif
    touchMasterLink();
    return;
  }

#if SLAVE_SYNC_MASTER_DRIVEN
  if (strcmp(pkt.type, "TSRQ") == 0) {
    slaveStartSyncFromMasterRequest();
    return;
  }
#endif

  if (strcmp(pkt.type, "TIME") == 0) {
    touchMasterLink();
#if ENABLE_IMU_TX_SLOT
    applyMasterSlotFromPacket(&pkt);
#endif
    uint64_t t4 = nowUs();

    uint64_t t1 = pkt.request_time;
    uint64_t t2 = pkt.micros_timestamp;
    uint64_t t3 = pkt.response_time;

    if (!(t4 >= t1 && t3 >= t2)) {
      return;
    }

    int64_t rtt = (int64_t)((t4 - t1) - (t3 - t2));
    if (rtt <= 0) return;

    /* Master chỉ gửi TIME sau SYNC của slave — chỉ xử lý trong vòng đồng bộ. */
    if (!syncRunning) {
      return;
    }

    if (rtt >= SYNC_ACCEPT_MAX_RTT_US) {
      syncSkipLogRttHigh(rtt);
      return;
    }

    const int64_t offset =
        ((int64_t)t2 - (int64_t)t1 + (int64_t)t3 - (int64_t)t4) / 2;

    syncRoundFinish(offset, rtt, pkt.timezone_offset, t3, t4);
  }
}

static const char* imuAccelFsLabel(uint8_t v) {
  switch (v) {
    case 0x00: return "±16 g";
    case 0x01: return "±8 g";
    case 0x02: return "±4 g";
    case 0x03: return "±2 g";
    default: return "?";
  }
}

static const char* imuGyroFsLabel(uint8_t v) {
  switch (v) {
    case 0x00: return "±2000 °/s";
    case 0x01: return "±1000 °/s";
    case 0x02: return "±500 °/s";
    case 0x03: return "±250 °/s";
    case 0x04: return "±125 °/s";
    case 0x05: return "±62.5 °/s";
    case 0x06: return "±31.25 °/s";
    case 0x07: return "±15.625 °/s";
    default: return "?";
  }
}

static const char* imuOdrLabel(uint8_t v) {
  switch (v) {
    case 0x01: return "32 kHz";
    case 0x02: return "16 kHz";
    case 0x03: return "8 kHz";
    case 0x04: return "4 kHz";
    case 0x05: return "2 kHz";
    case 0x06: return "1 kHz";
    case 0x07: return "200 Hz";
    case 0x08: return "100 Hz";
    case 0x09: return "50 Hz";
    case 0x0A: return "25 Hz";
    case 0x0B: return "12.5 Hz";
    case 0x0C: return "6.25 Hz (LP)";
    case 0x0D: return "3.125 Hz (LP)";
    case 0x0E: return "1.5625 Hz (LP)";
    case 0x0F: return "500 Hz";
    default: return "?";
  }
}

// Đọc 1 byte thanh ghi (USER BANK 0) — cùng bus I2C với ICM42688
static bool imuReadRegBank0(uint8_t reg, uint8_t* out) {
  using namespace ICM42688reg;
  Wire.beginTransmission(ICM42688_ADDR);
  Wire.write(REG_BANK_SEL);
  Wire.write(0);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(ICM42688_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ICM42688_ADDR, 1) != 1) return false;
  *out = Wire.read();
  return true;
}

static void printImuConfigToSerial() {
  uint8_t af = (uint8_t)IMU_ACCEL_FS;
  uint8_t gf = (uint8_t)IMU_GYRO_FS;
  uint8_t ao = (uint8_t)IMU_ACCEL_ODR;
  uint8_t go = (uint8_t)IMU_GYRO_ODR;
  Serial.println("IMU config (macros → setAccel/Gyro*):");
  Serial.printf("  Accel FS: %s (enum 0x%02X)\n", imuAccelFsLabel(af), af);
  Serial.printf("  Gyro FS:  %s (enum 0x%02X)\n", imuGyroFsLabel(gf), gf);
  Serial.printf("  Accel ODR: %s (enum 0x%02X)\n", imuOdrLabel(ao), ao);
  Serial.printf("  Gyro ODR:  %s (enum 0x%02X)\n", imuOdrLabel(go), go);

  uint8_t accelCfg = 0, gyroCfg = 0;
  using namespace ICM42688reg;
  if (!imuReadRegBank0(UB0_REG_ACCEL_CONFIG0, &accelCfg) ||
      !imuReadRegBank0(UB0_REG_GYRO_CONFIG0, &gyroCfg)) {
    Serial.println("IMU readback (chip): I2C read ACCEL_CONFIG0 / GYRO_CONFIG0 failed");
    return;
  }
  uint8_t afs = (accelCfg >> 5) & 0x07;
  uint8_t aos = accelCfg & 0x0F;
  uint8_t gfs = (gyroCfg >> 5) & 0x07;
  uint8_t gos = gyroCfg & 0x0F;
  Serial.println("IMU readback (chip registers):");
  Serial.printf("  ACCEL_CONFIG0 0x%02X: raw=0x%02X | FS=%s | ODR=%s\n",
                UB0_REG_ACCEL_CONFIG0, accelCfg,
                imuAccelFsLabel(afs), imuOdrLabel(aos));
  Serial.printf("  GYRO_CONFIG0  0x%02X: raw=0x%02X | FS=%s | ODR=%s\n",
                UB0_REG_GYRO_CONFIG0, gyroCfg,
                imuGyroFsLabel(gfs), imuOdrLabel(gos));
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(921600);
  delay(200);

  Serial.println("\n=== ESP-NOW TIME SYNC SLAVE ICM42688 ===");

  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_ADC_PIN, ADC_11db);
  Serial.printf(
      "VBAT ADC: GPIO%u, MA=%u mau, ~%.1f Hz (moi %lums)\n",
      (unsigned)VBAT_ADC_PIN, (unsigned)VBAT_ADC_MA_N,
      1000.0f / (float)VBAT_ADC_SAMPLE_PERIOD_MS,
      (unsigned long)VBAT_ADC_SAMPLE_PERIOD_MS);

  Serial.printf("Pulse pin D%d HIGH during IMU read, LOW before packet handling\n", PULSE_PIN);
  Serial.printf(
      "IMU ESP-NOW: %u samples/batch (~%u pkt/s @ ~%lu Hz loop)\n",
      (unsigned)IMU_BATCH_SAMPLES,
      (unsigned)((1000000UL / IMU_LOOP_PERIOD_US) / IMU_BATCH_SAMPLES),
      (unsigned long)(1000000UL / IMU_LOOP_PERIOD_US));
#if ENABLE_WIFI_SEND
  Serial.println("ESP-NOW send: ON");
#else
  Serial.println("ESP-NOW send: OFF");
#endif
#if SERIAL_IMU_OUTPUT
  Serial.println("Serial IMU output: ON");
#else
  Serial.println("Serial IMU output: OFF");
#endif

  // ---- IMU init ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(500);

  int status = IMU.begin();
  if (status < 0) {
    Serial.printf("ICM42688 init failed! status=%d\n", status);
    while (1) delay(1000);
  }

  IMU.setAccelFS(IMU_ACCEL_FS);
  IMU.setGyroFS(IMU_GYRO_FS);
  IMU.setAccelODR(IMU_ACCEL_ODR);
  IMU.setGyroODR(IMU_GYRO_ODR);
  printImuConfigToSerial();

  Serial.println("ICM42688 ready (raw accel/gyro, mag=0)");
  loadAccelCalibrationFromNvs();
  loadSyncDriftFromNvs();
#if SYNC_DRIFT_COMP_ENABLE
  Serial.printf(
      "Drift comp: ON | fold every %lu ms | min accepted=%u | filter alpha=%.2f\n",
      (unsigned long)SYNC_DRIFT_COMP_INTERVAL_MS,
      (unsigned)SYNC_DRIFT_COMP_MIN_ACCEPTED, SYNC_DRIFT_FILTER_ALPHA);
#else
  Serial.println("Drift comp: OFF");
#endif

  // ---- Radio ----
  setupRadio();

  uint8_t mac[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif

  Serial.print("Slave MAC: ");
  printMAC(mac);
  Serial.println();
  Serial.println("----------------------------------");

  // ---- ESP-NOW ----
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
#if ENABLE_WIFI_SEND
  esp_now_register_send_cb(onEspNowSendCb);
#endif

  if (!addMasterPeer()) {
    Serial.println("Add master peer failed!");
    return;
  }

  Serial.print("Master MAC: ");
  printMAC(masterMAC);
  Serial.println(" OK");

  Serial.println("Slave ready, trying to connect to master...");
  printTime();

#if ENABLE_IMU_DEDICATED_TASK
  imuBatchTxQueue = xQueueCreate(IMU_BATCH_QUEUE_DEPTH, sizeof(imu_batch_packet_t));
  if (imuBatchTxQueue == NULL) {
    Serial.println(
        "IMU: xQueueCreate failed — dùng loop() đọc IMU (không task).");
  } else {
    BaseType_t ok = xTaskCreatePinnedToCore(
        imuSampleTask, "imu100", IMU_SAMPLE_TASK_STACK, NULL, IMU_SAMPLE_TASK_PRIO,
        &imuSampleTaskHandle, IMU_SAMPLE_TASK_CORE);
    if (ok != pdPASS) {
      Serial.println("IMU: xTaskCreatePinnedToCore failed — queue delete, fallback loop.");
      vQueueDelete(imuBatchTxQueue);
      imuBatchTxQueue = NULL;
    } else {
#if IMU_SAMPLE_ALIGN_MASTER_10MS_GRID
      Serial.printf(
          "IMU: task ~100 Hz on core %d (queue %d); sau sync: grid period=%u us, "
          "sample_phase=%u us (0=on boundary)\n",
          IMU_SAMPLE_TASK_CORE, IMU_BATCH_QUEUE_DEPTH,
          (unsigned)IMU_LOOP_PERIOD_US, (unsigned)IMU_SAMPLE_GRID_PHASE_US);
#else
      Serial.printf(
          "IMU: task 100 Hz on core %d (queue depth %d)\n",
          IMU_SAMPLE_TASK_CORE, IMU_BATCH_QUEUE_DEPTH);
#endif
    }
  }
#endif

  delay(1000);
  sendHello();
}

// ======================== LOOP ========================
void loop() {
  vbatAdcPoll();
  const uint8_t batPct = adcRawToBatteryPct(readVbatAdcRaw());
  if (lowBatteryCrit) {
    const unsigned long m = millis() % VBAT_LOW_LED_CYCLE_MS;
    digitalWrite(LED_PIN, (m >= VBAT_LOW_LED_OFF_MS) ? HIGH : LOW);
    yield();
    delay(1);
    return;
  }
  if (batPct < VBAT_LOW_PCT_ENTER) {
    enterLowBatteryCritMode();
    return;
  }

#if !SLAVE_SYNC_MASTER_DRIVEN
  if (connected && !syncRunning && !timeSynced) {
    const unsigned long nowMs = millis();
    const unsigned long needGap =
        (lastSyncTime == 0U) ? 0UL : (unsigned long)SYNC_RETRY_AFTER_FAIL_MS;
    const unsigned long sinceLast =
        (lastSyncTime == 0U) ? needGap : (unsigned long)(nowMs - lastSyncTime);
    if (sinceLast >= needGap) {
      if (slaveRssiTooWeakForSync()) {
        slaveSyncSkipLogRssiWeak();
      } else {
        syncRunning = true;
#if ENABLE_WIFI_SEND
        sendSyncRequest();
#endif
        lastSyncSampleSentMs = nowMs;
        Serial.println("Starting SYNC (require TIME before IMU)...");
      }
    }
  }

  if (connected && !syncRunning && timeSynced &&
      (millis() - lastSyncTime > syncPeriodicMinGapMs()) &&
      syncPeriodicSlotReady()) {
    if (slaveRssiTooWeakForSync()) {
      slaveSyncSkipLogRssiWeak();
    } else {
      syncRunning = true;
      lastSyncSampleSentMs = millis() - (unsigned long)SYNC_SAMPLE_GAP_MS;
      Serial.printf("Starting SYNC (%lu ms + MAC slot %u, offset %lu ms)...\n",
                    (unsigned long)syncPeriodicIntervalMs,
                    (unsigned)syncPeriodicMacSlotIndex(),
                    (unsigned long)syncPeriodicSlotOffsetMs());
    }
  }

  if (connected && syncRunning) {
    if (slaveRssiTooWeakForSync()) {
      syncRunning = false;
      slaveSyncSkipLogRssiWeak();
    } else {
      const unsigned long syncGap =
          syncLastSendOk ? (unsigned long)SYNC_SAMPLE_GAP_MS
                         : (unsigned long)SYNC_FAIL_BACKOFF_GAP_MS;
      if (millis() - lastSyncSampleSentMs >= syncGap) {
        sendSyncRequest();
        lastSyncSampleSentMs = millis();
      }
    }
  }
#endif

  bool imuSent = false;
  uint64_t loopStartUsForDelay = 0;
#if ENABLE_IMU_DEDICATED_TASK
  if (imuBatchTxQueue != NULL) {
    if (!syncRunning) {
      imuDrainBatchTxQueue();
    }
    static uint32_t lastSeenImuPacketCount = 0;
    imuSent = (imuPacketCount != lastSeenImuPacketCount);
    lastSeenImuPacketCount = imuPacketCount;
  } else if (!syncRunning) {
    loopStartUsForDelay = nowUs();
    imuSent = sendImuData();
  }
#else
  if (!syncRunning) {
    loopStartUsForDelay = nowUs();
    imuSent = sendImuData();
  }
#endif

  if (ledOffAt > 0 && millis() >= ledOffAt) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }

  if (connected && (millis() - lastAckTime > CONNECTION_TIMEOUT_MS)) {
    disconnectFromMaster("Lost connection to master (ACK timeout), reconnecting...");
  }

  if (!connected) {
    unsigned long t = millis();
    bool noMaster = !hasEverConnected ||
                    (hasEverConnected && disconnectedSince > 0 &&
                     (t - disconnectedSince > NO_MASTER_THRESHOLD));

    unsigned long cycle = noMaster ? LED_NO_MASTER_CYCLE : LED_RECONNECT_CYCLE;
    unsigned long pos = t % cycle;

    if (noMaster) {
      digitalWrite(LED_PIN, (pos < 50) ? HIGH : LOW);
    } else {
      bool on = (pos < LED_FLASH_MS) || (pos >= 125 && pos < 125 + LED_FLASH_MS);
      digitalWrite(LED_PIN, on ? HIGH : LOW);
    }
  }

  if (!connected && (millis() - lastHelloTime > HELLO_RECONNECT_INTERVAL)) {
    sendHello();
  }

#if HELLO_KEEPALIVE_INTERVAL > 0
  if (connected && (millis() - lastHelloTime > HELLO_KEEPALIVE_INTERVAL)) {
    sendHello();
  }
#endif

  syncDriftPeriodicService();

  if (imuSent && connected && timeSynced && imuPacketCount > 0 &&
      (imuPacketCount % 10 == 0)) {
    digitalWrite(LED_PIN, HIGH);
    ledOffAt = millis() + LED_FLASH_MS;
  }

#if ENABLE_WIFI_SEND
  if (connected && !syncRunning) {
    drainImuOfflineQueue();
    imuLowRssiService();
  }
#endif

  yield();

#if !ENABLE_IMU_DEDICATED_TASK
  /*
   * Chu kỳ vòng lặp ≈ IMU_LOOP_PERIOD_US (10 ms) → ~100 Hz mẫu IMU.
   * Nếu elapsedUs >= IMU_LOOP_PERIOD_US (SYNC nặng, drain/TX lâu) thì không delay.
   */
  {
    uint64_t elapsedUs = nowUs() - loopStartUsForDelay;
    if (elapsedUs < IMU_LOOP_PERIOD_US) {
      delayMicroseconds((uint32_t)(IMU_LOOP_PERIOD_US - elapsedUs));
    }
  }
#else
  if (imuBatchTxQueue == NULL) {
    uint64_t elapsedUs = nowUs() - loopStartUsForDelay;
    if (elapsedUs < IMU_LOOP_PERIOD_US) {
      delayMicroseconds((uint32_t)(IMU_LOOP_PERIOD_US - elapsedUs));
    }
  }
#endif
}