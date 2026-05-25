#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// ============================================================================
// --- USER CUSTOMIZATION  ---
// ============================================================================

// --- BLE Branding ---
#define SERIAL_NUM "0123456789" // The unique 10-digit serial

// (DO NOT CHANGE THESE: Required for RaceBox Application compatibility)
#define DEVICE_NAME "RaceBox Mini " SERIAL_NUM // Auto-synced Name
#define MANUFACTURER "RaceBox"
#define FIRMWARE_VER "3.3"

// --- GPS Performance ---
#define MAX_NAVIGATION_RATE 25 // 25Hz: Max rate for RaceBox Mini protocol
#define GPS_BAUD 115200        // High speed for 25Hz data
#define FACTORY_GPS_BAUD 9600  // Default for cold modules

#define SYSTEM_RATE_REPORT_MS 5000 // Interval for Serial stats reporting

// --- Power & Efficiency ---
#define EXPECT_BATTERY true // Set to false to force USB-only No-Battery mode
#define GPS_HOT_TIMEOUT_MS 900000 // 15 Minutes (Stay powered after disconnect)
#define DEEP_SLEEP_DAYS 1         // Safety net before absolute shutdown
#define ENABLE_DEEP_SLEEP false   // Usually false for standard RaceBox usage
#define FAST_ADV_INTERVAL 160 // 100ms: Fast discovery for apps (160 * 0.625ms)
#define ECO_ADV_INTERVAL                                                       \
  4000 // 2500ms: Extremely low power (4000 * 0.625ms = 2.5s latency to connect)
#define LOOP_SLEEP 2500 // 2500ms delay for the main loop iteration while idle
#define SLEEP_WHILE_CHARGING                                                   \
  true // Allow Light Sleep even when plugged in /Set false to force high power
       // mode when plugged in
#define LOW_POWER_BT_TX_POWER -4 // dBm for low power consumption

// --- GNSS Constellation Toggle ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS
// #define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

// --- Hardware Version ---
// Uncomment the line below if using the custom PCB version where
// the GPS_EN_PIN logic is inverted. (For PCB: LOW = ON, HIGH = OFF)
// #define PCB_VERSION

// ============================================================================
// ---  HARDWARE MAPPINGS ---
// ============================================================================

#define GPS_EN_PIN D1      // GPS Power Enable Rail
#define PIN_VBAT_ENABLE 14 // Battery Read Enable
#define PIN_HICHG 22       // Charge Speed (LOW=100mA)
#define PIN_CHG 23         // Charge Indicator (LOW=Charging)
#define ACCEL_INT_PIN PIN_LSM6DS3TR_C_INT1

// ============================================================================
// ---  GLOBAL SYSTEM STATE ---
// ============================================================================

SFE_UBLOX_GNSS myGNSS;
LSM6DS3 IMU(I2C_MODE, 0x6A);

// System Flags
bool deviceConnected = false;
bool gpsEnabled = false;
bool imuEnabled = false;
bool pendingConfig = false;
bool lastChargingState = false;
bool lastPluggedInState = false;
bool batteryConnected = true;
uint8_t currentBatteryPercentage = 100;
float batteryMultiplier = 3.0; // Voltage divider 1/3

// Global State
bool isCritical = false;
bool isNoBatteryMode = false;
int GPSFixType = 0;

// Timing Trackers
unsigned long lastDisconnectTime = 0;
unsigned long lastActivityTime = 0;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
unsigned int gnssUpdateCount = 0;

// Filter/IMU State (Now using hardware filtering)
float imu_ax = 0, imu_ay = 0, imu_az = 0;
float imu_gx = 0, imu_gy = 0, imu_gz = 0;

// BLE Core Objects
const uint8_t RACEBOX_SERVICE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x01, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_TX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x03, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_RX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x02, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_GNSS_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                     0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                     0x04, 0x00, 0x40, 0x6E};

BLEService rbService(RACEBOX_SERVICE_UUID);
BLECharacteristic rbTx(RACEBOX_TX_UUID);
BLECharacteristic rbRx(RACEBOX_RX_UUID);
BLECharacteristic rbGnss(RACEBOX_GNSS_UUID);

BLEService disService(UUID16_SVC_DEVICE_INFORMATION);
BLECharacteristic disModel(UUID16_CHR_MODEL_NUMBER_STRING);
BLECharacteristic disSerial(UUID16_CHR_SERIAL_NUMBER_STRING);
BLECharacteristic disFirmware(UUID16_CHR_FIRMWARE_REVISION_STRING);
BLECharacteristic disHardware(UUID16_CHR_HARDWARE_REVISION_STRING);
BLECharacteristic disManuf(UUID16_CHR_MANUFACTURER_NAME_STRING);

const int OnboardledPin = LED_BLUE;

// --- Helper Utilities ---
template <typename T>
void writeLittleEndian(uint8_t *buffer, int offset, T value) {
  memcpy(buffer + offset, &value, sizeof(T));
}

void calculateChecksum(uint8_t *payload, uint16_t len, uint8_t cls, uint8_t id,
                       uint8_t *ckA, uint8_t *ckB) {
  *ckA = *ckB = 0;
  *ckA += cls;
  *ckB += *ckA;
  *ckA += id;
  *ckB += *ckA;
  *ckA += len & 0xFF;
  *ckB += *ckA;
  *ckA += len >> 8;
  *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

bool isCharging() { return digitalRead(PIN_CHG) == LOW; }

// Calibration Table
struct VoltagePoint {
  float voltage;
  uint8_t percentage;
};

const VoltagePoint batteryMap[] = {
    {4.13, 100}, {4.12, 98}, {4.10, 95}, {4.05, 92}, {4.00, 88}, {3.98, 85},
    {3.96, 82},  {3.94, 79}, {3.92, 75}, {3.90, 72}, {3.88, 68}, {3.85, 65},
    {3.82, 62},  {3.78, 55}, {3.72, 45}, {3.68, 35}, {3.63, 25}, {3.58, 18},

    {3.50, 10},  {3.35, 5},  {3.20, 0}};

const uint8_t mapSize = sizeof(batteryMap) / sizeof(VoltagePoint);

// Lookup Function
float getRawPercentage() {
  float v = getBatteryVoltage(); // Your 16-sample average function
  if (v >= batteryMap[0].voltage)
    return 100.0;
  if (v <= batteryMap[mapSize - 1].voltage)
    return 0.0;

  for (int i = 0; i < mapSize - 1; i++) {
    if (v <= batteryMap[i].voltage && v > batteryMap[i + 1].voltage) {
      float vHigh = batteryMap[i].voltage;
      float vLow = batteryMap[i + 1].voltage;
      uint8_t pHigh = batteryMap[i].percentage;
      uint8_t pLow = batteryMap[i + 1].percentage;
      return pLow + (v - vLow) * (pHigh - pLow) / (vHigh - vLow);
    }
  }
  return 0.0;
}

// State Update
void updateBatteryState() {
  static float filteredPct = -1.0;
  bool pluggedIn = isCharging();
  float rawPct = getRawPercentage();
  float currentV = getBatteryVoltage();

  // Initial Sync
  if (filteredPct < 0) {
    filteredPct = rawPct;
    currentBatteryPercentage = (uint8_t)rawPct;
  }

  // Heavy Filter (Adjusted for 30s interval to prevent lag)
  filteredPct = (rawPct * 0.15) + (filteredPct * 0.85);
  uint8_t rounded = (uint8_t)(filteredPct + 0.5);

  // Set Critical Flag (e.g., below 3.35V / 10%)
  isCritical = (currentV < 3.35);

  // --- STICKY 100% LATCH ---
  // If we were at 100%, don't drop the display until the filtered value
  // hits 95%. This prevents the 5V boost-regulator sag from killing
  // the "Full" status immediately upon power-on.
  if (currentBatteryPercentage == 100 && !pluggedIn && rounded > 95) {
    rounded = 100;
  }

  if (abs((int)rounded - (int)currentBatteryPercentage) >= 2 ||
      rounded == 100 || rounded == 0) {
    if (pluggedIn) {
      if (rounded > currentBatteryPercentage)
        currentBatteryPercentage = rounded;
    } else {
      if (rounded < currentBatteryPercentage)
        currentBatteryPercentage = rounded;

      // Boot/Recovery Sync: If raw is significantly out of sync, force update.
      // This prevents the percentage from getting stuck if updates were missed.
      if (rawPct > currentBatteryPercentage + 5.0 ||
          currentBatteryPercentage > rawPct + 20.0) {
        currentBatteryPercentage = (uint8_t)rawPct;
        filteredPct = rawPct;
      }
    }
  }
}

bool isPluggedIn() {
  // Check if USB power is detected
  return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

float getBatteryVoltage() {
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Enable divider
  delay(1);

  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_VBAT);
    delayMicroseconds(50);
  }
  float adcCount = (float)sum / 8.0;
  float voltage = (batteryMultiplier * 3.6 * adcCount / 4096);

  // --- LOAD COMPENSATION ---
  // If the GPS is running (30mA draw), the battery voltage sags.
  // We add an offset to compensate so the percentage doesn't drop just
  // because the sensors turned on.
  if (gpsEnabled && !isPluggedIn()) {
    voltage += 0.010; // Approx compensation for 30mA on a 1000mAh pack
  }

  if (!isCharging() && !isPluggedIn()) {
    digitalWrite(PIN_VBAT_ENABLE, HIGH);
  }
  return voltage;
}

// ============================================================================
// ---  SENSOR PROCESSING MODULES ---
// ============================================================================

// Assemble and transmit the proprietary RaceBox Mini protocol packet

void sendRaceboxPacket() {
  if (!deviceConnected || myGNSS.packetUBXNAVPVT == NULL)
    return;

  uint8_t payload[80] = {0};
  uint8_t packet[88] = {0};
  auto *data = &myGNSS.packetUBXNAVPVT->data;

  // Time and Resolution
  writeLittleEndian(payload, 0, data->iTOW);
  writeLittleEndian(payload, 4, data->year);
  writeLittleEndian(payload, 6, data->month);
  writeLittleEndian(payload, 7, data->day);
  writeLittleEndian(payload, 8, data->hour);
  writeLittleEndian(payload, 9, data->min);
  writeLittleEndian(payload, 10, data->sec);

  // Status and Accuracy
  uint8_t val = 0;
  if (data->valid.bits.validDate)
    val |= (1 << 0);
  if (data->valid.bits.validTime)
    val |= (1 << 1);
  if (data->valid.bits.fullyResolved)
    val |= (1 << 2);
  writeLittleEndian(payload, 11, val);
  writeLittleEndian(payload, 12, data->tAcc);
  writeLittleEndian(payload, 16, data->nano);
  writeLittleEndian(payload, 20, data->fixType);

  // Fix and Info Flags
  uint8_t fixFlags = 0;
  if (data->fixType == 3)
    fixFlags |= (1 << 0);
  if (myGNSS.getHeadVehValid())
    fixFlags |= (1 << 5);
  writeLittleEndian(payload, 21, fixFlags);

  uint8_t dtFlags = 0;
  if (data->valid.bits.validTime)
    dtFlags |= (1 << 5);
  if (data->valid.bits.validDate)
    dtFlags |= (1 << 6);
  if (data->valid.bits.validTime && data->valid.bits.fullyResolved)
    dtFlags |= (1 << 7);
  writeLittleEndian(payload, 22, dtFlags);

  // Position, Speed, and Heading
  writeLittleEndian(payload, 23, data->numSV);
  writeLittleEndian(payload, 24, (int32_t)data->lon);
  writeLittleEndian(payload, 28, (int32_t)data->lat);
  writeLittleEndian(payload, 32, (int32_t)data->height);
  writeLittleEndian(payload, 36, (int32_t)data->hMSL);
  writeLittleEndian(payload, 40, (uint32_t)data->hAcc);
  writeLittleEndian(payload, 44, (uint32_t)data->vAcc);
  writeLittleEndian(payload, 48, (int32_t)data->gSpeed);
  writeLittleEndian(payload, 52, (int32_t)data->headMot);
  writeLittleEndian(payload, 56, (uint32_t)data->sAcc);
  writeLittleEndian(payload, 60, (uint32_t)data->headAcc);
  writeLittleEndian(payload, 64, (uint16_t)data->pDOP);

  // Fix Quality and Misc
  if (data->fixType < 2)
    writeLittleEndian(payload, 66, (uint8_t)(1 << 0));

  uint8_t batPct = currentBatteryPercentage & 0x7F;
  if (isCharging())
    batPct |= 0x80;
  writeLittleEndian(payload, 67, batPct);

  // Physical Sensors (Hardware Filtered)
  writeLittleEndian(payload, 68, (int16_t)(imu_ax * 1000.0));
  writeLittleEndian(payload, 70, (int16_t)(imu_ay * 1000.0));
  writeLittleEndian(payload, 72, (int16_t)(imu_az * 1000.0));
  writeLittleEndian(payload, 74, (int16_t)(imu_gx * 100.0));
  writeLittleEndian(payload, 76, (int16_t)(imu_gy * 100.0));
  writeLittleEndian(payload, 78, (int16_t)(imu_gz * 100.0));

  // Protocol Wrapper
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0xFF;
  packet[3] = 0x01;
  packet[4] = 80;
  packet[5] = 0;
  memcpy(packet + 6, payload, 80);

  uint8_t ckA, ckB;
  calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
  packet[86] = ckA;
  packet[87] = ckB;

  if (rbTx.notify(packet, 88))
    gpsUpdateCount++;
}

// Background polling and data routing for the u-blox module
void processGNSS() {
  if (!gpsEnabled)
    return;
  myGNSS.checkUblox();

  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;
    GPSFixType = myGNSS.packetUBXNAVPVT->data.fixType;
    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;
      sendRaceboxPacket();
    }
  }

  // Backup Recovery: Ensure background processing during data stalls
  static unsigned long lastValidData = 0;
  if (myGNSS.getPVT())
    lastValidData = millis();
  if (deviceConnected && (millis() - lastValidData > 2000))
    myGNSS.checkUblox();
}

// IMU Sampling (Hardware filters handle smoothing)
void processIMU() {
  if (!imuEnabled)
    return;

  imu_ax = IMU.readFloatAccelX();
  imu_ay = IMU.readFloatAccelY();
  imu_az = IMU.readFloatAccelZ();
  imu_gx = IMU.readFloatGyroX();
  imu_gy = IMU.readFloatGyroY();
  imu_gz = IMU.readFloatGyroZ();
}

// ============================================================================
// --- 🔋 POWER & SYSTEM MANAGEMENT ---
// ============================================================================
bool resetGpsBaudRate() {
  Serial.println("🔍 Deep Scanning for GNSS activity...");
  long bauds[] = {9600, 38400, 115200, 57600};

  for (int b = 0; b < 4; b++) {
    Serial.print("Checking ");
    Serial.print(bauds[b]);
    Serial.print(" baud: ");

    Serial1.end();
    delay(100);
    Serial1.begin(bauds[b]);

    // Sniff for raw activity first (for 1.5 seconds)
    unsigned long sniffStart = millis();
    bool activity = false;
    while (millis() - sniffStart < 1500) {
      if (Serial1.available()) {
        activity = true;
        break;
      }
    }

    if (activity) {
      Serial.print("RAW DATA DETECTED! ");

      // Clear the buffer of NMEA messages
      delay(200);
      while (Serial1.available())
        Serial1.read();

      // Try the library sync
      if (myGNSS.begin(Serial1)) {
        Serial.println("✅ UBX Protocol Synced!");

        if (bauds[b] != GPS_BAUD) {
          Serial.print("Elevating to ");
          Serial.print(GPS_BAUD);
          Serial.println(" baud...");
          myGNSS.setSerialRate(GPS_BAUD);
          delay(100);
          Serial1.end();
          delay(100);
          Serial1.begin(GPS_BAUD);
        }

        myGNSS.saveConfiguration();
        return true;
      } else {
        Serial.println("❌ Bytes received, but u-blox library could not sync "
                       "(Check protocol/clones).");
      }
    } else {
      Serial.println("Silent.");
    }

    Serial1.end();
  }

  Serial.println("❌ GNSS not detected. Check VCC voltage or TX/RX wiring.");
  return false;
}

bool configureGPS() {
  if (!pendingConfig)
    return false;
  Serial.println("⚙️ Syncing GPS Settings...");
  Serial1.begin(GPS_BAUD);

  bool detected = false;
  for (int i = 0; i < 3; i++) {
    if (myGNSS.begin(Serial1)) {
      detected = true;
      break;
    }
    delay(20);
  }

  if (!detected) {
    return resetGpsBaudRate();
  }

  myGNSS.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX);
  myGNSS.setAutoPVT(true);

  // High-Performance Track Dynamics Configuration
  // Used Airborne2g since passenger track cars (GT3/GT4) easily exceed 1g
  // braking/cornering and we do not want the navigation engine to "smooth" or
  // reject these forces as anomalies.
  myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g);

  // Disable low-pass filters to prevent time-domain lag on braking and
  // acceleration telemetry
  myGNSS.setVal8(0x10220001,
                 0); // CFG-ODO-OUTLPVEL: Disable speed (3D) low-pass filter
  myGNSS.setVal8(
      0x10220002,
      0); // CFG-ODO-OUTLPCOG: Disable course over ground low-pass filter

  // Ensure static hold is explicitly disabled so slow pit-lane creeping isn't
  // frozen at 0mph
  myGNSS.setVal8(0x20250038,
                 0); // CFG-MOT-GNSSSPEED_THRS: Static hold threshold = 0

  // Enable Super-S (Automatic Mode) to compensate for any signal attenuation
  // through the ABS enclosure
  myGNSS.setVal8(0x201100D5, 1); // CFG-NAVSPG-SIGATTCOMP: 1 = Automatic

  // For SAM-M10Q 25Hz, we MUST explicitly disable other constellations FIRST
  // before increasing the navigation frequency, otherwise the module rejects
  // it.
#ifdef ENABLE_GNSS_GPS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
#endif

#ifdef ENABLE_GNSS_GALILEO
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
#endif

#ifdef ENABLE_GNSS_GLONASS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
#endif

#ifdef ENABLE_GNSS_BEIDOU
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
#endif

#ifdef ENABLE_GNSS_SBAS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
#endif

#ifdef ENABLE_GNSS_QZSS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
#endif

  // Now that extraneous constellations are OFF, we can safely request 25Hz
  myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE);

  pendingConfig = false;
  Serial.println("✅ Configuration complete.");
  return true;
}

// Re-configure advertising with specific power and interval
// Standard Adafruit/Seeed Bluefruit requires re-adding data to update the
// packet-reported TX power
void setupAdvertising(int8_t power, uint16_t interval) {
  if (deviceConnected)
    return;

  Bluefruit.Advertising.stop();
  Bluefruit.setTxPower(power);
  Bluefruit.Advertising.setInterval(interval, interval + 200);

  // Clear and Rebuild Advertising Data
  // This ensures the TX Power field in the packet matches the new hardware
  // power
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // NOTE: We keep the Service UUIDs in the primary advertisement for
  // compatibility with the RaceBox application.
  Bluefruit.Advertising.addService(rbService);
  Bluefruit.Advertising.addService(disService);

  // Scan Response only contains the Name to keep it simple
  Bluefruit.ScanResponse.clearData();
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.start(0);
}

void enableGPS() {
  if (gpsEnabled)
    return;
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, HIGH);
#endif
  gpsEnabled = true;
  delay(100);
  if (!deviceConnected) {
    setupAdvertising(0, FAST_ADV_INTERVAL);
  }
}

void disableGPS() {
  GPSFixType = 0;
  pendingConfig = true;
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
#else
  digitalWrite(GPS_EN_PIN, LOW);
#endif
  gpsEnabled = false;
  // Serial1.end(); // Dont uncomment this, it causes a 4mA power leak
  // pinMode(PIN_SERIAL1_TX, INPUT_PULLDOWN);
  // pinMode(PIN_SERIAL1_RX, INPUT_PULLDOWN);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  if (!deviceConnected) {
    setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
  }
}

void enableIMU() {
  if (imuEnabled)
    return;
  IMU.settings.accelSampleRate = 1660; // 1.6kHz for hardware filtering
  IMU.settings.gyroSampleRate = 1660;
  IMU.settings.accelRange = 8;
  IMU.settings.gyroRange = 500;

  if (IMU.begin() != 0)
    return;

  // --- HARDWARE FILTER CONFIGURATION ---
  // Gyroscope: Enable LPF1 (Register 0x13, Bit 1)
  IMU.writeRegister(0x13, 0x02);
  // Gyroscope: Set LPF1 Bandwidth (Register 0x15, Bits 1:0 = 01)
  IMU.writeRegister(0x15, 0x01);
  // Accelerometer: Enable LPF2 (Register 0x17, Bit 0) and Set HPCF (Bits 6:5 =
  // 01 for ODR/50)
  IMU.writeRegister(0x17, 0x21);

  imuEnabled = true;
}

void disableIMU() {
  if (!imuEnabled)
    return;
  // Explicitly power down the sensor registers
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  imuEnabled = false;
}

void powerDownSensors() {
  // Soft disable
  if (gpsEnabled)
    disableGPS();
  if (imuEnabled)
    disableIMU();

  // Enforce hardware shutdown state to prevent parasitic leaks or pin sync
  // issues
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
#else
  digitalWrite(GPS_EN_PIN, LOW);
#endif
}

// Manage Charging, Disconnect Timeouts, and Deep Sleep
void managePower() {
  if (isNoBatteryMode) {
    // In No-Battery mode, everything stays hot always.
    if (!gpsEnabled)
      enableGPS();
    configureGPS();
    if (!imuEnabled)
      enableIMU();
    return;
  }

  bool currentlyPluggedIn = isPluggedIn();

  if (!currentlyPluggedIn && !deviceConnected &&
      currentBatteryPercentage == 0) {
    enterDeepSleep();
  }

  // Determine if sensors should be active
  bool shouldBeActive =
      deviceConnected || (currentlyPluggedIn && !SLEEP_WHILE_CHARGING);

  if (shouldBeActive) {
    lastActivityTime = millis();
    lastDisconnectTime = millis();
    if (!gpsEnabled) {
      enableGPS();
    }
    // Keep trying to configure until pendingConfig is false
    configureGPS();
    enableIMU();
  } else {
    if (imuEnabled)
      disableIMU();
  }

  // Hot-State Timeout (Keep GPS active for a window after usage)
  if (!deviceConnected && gpsEnabled && SLEEP_WHILE_CHARGING) {
    if (millis() - lastDisconnectTime > GPS_HOT_TIMEOUT_MS) {
      Serial.printf(
          "⏰ GPS Hot Timeout Reached. Rebooting to clear hardware state...\n");
      Serial.flush();
      delay(10);

      // Hard reset the MCU instead of trying to manually power down bugged
      // peripherals. Setup() will natively drop the board back into its 40uA
      // state.
      NVIC_SystemReset();
    }
  }

  // Deep Sleep Safety Net
  if (ENABLE_DEEP_SLEEP && !deviceConnected && !currentlyPluggedIn) {
    if (millis() - lastActivityTime > (DEEP_SLEEP_DAYS * 86400000UL))
      enterDeepSleep();
  }

  // Reset charging state trackers
  if (lastPluggedInState && !currentlyPluggedIn) {
    lastActivityTime = millis();
    if (!deviceConnected)
      lastDisconnectTime = millis();
  }
  lastPluggedInState = currentlyPluggedIn;
}

void manageBatterySampling() {
  if (isNoBatteryMode) {
    currentBatteryPercentage = 100;
    isCritical = false;
    batteryConnected = false;
    return; // Skip battery processing completely in No-Battery Mode
  }

  static unsigned long lastBatteryUpdate = 0;
  const unsigned long batteryInterval = 30000; // 30 Seconds

  bool charging = isCharging();

  // Force an update if the power state just changed (Plugged in or Unplugged)
  // This ensures the Serial report and LEDs react instantly to the cable.
  static bool lastChargingStatus = false;
  bool stateChanged = (charging != lastChargingStatus);

  batteryConnected = true;

  if (millis() - lastBatteryUpdate >= batteryInterval || stateChanged ||
      lastBatteryUpdate == 0) {
    lastBatteryUpdate = millis();
    lastChargingStatus = charging;

    updateBatteryState();
  }
}

// Periodic Status Feed to the Computer
void reportSystemStats() {
  if (millis() - lastGpsRateCheckTime < SYSTEM_RATE_REPORT_MS)
    return;

  float bleRate = gpsUpdateCount / (SYSTEM_RATE_REPORT_MS / 1000.0);
  float gnssRate = gnssUpdateCount / (SYSTEM_RATE_REPORT_MS / 1000.0);
  Serial.println("--------------------------------------------------");
  Serial.printf("POWER   | Bat: %d%% (%0.2fV) \n", currentBatteryPercentage,
                getBatteryVoltage());
  Serial.printf("STATE   | Charging: %s | USB: %s | BLE: %s | BAT: %s\n",
                isCharging() ? "YES ⚡" : "NO 🔋",
                isPluggedIn() ? "CONNECTED" : "DISCONNECTED",
                deviceConnected ? "CONNECTED" : "IDLE",
                batteryConnected ? "PRESENT" : "MISSING");
  if (gpsEnabled && myGNSS.packetUBXNAVPVT) {
    Serial.printf(
        "GNSS    | BLE: %.2f Hz | GNSS: %.2f Hz | SVs: %u | Fix: %u\n", bleRate,
        gnssRate, myGNSS.packetUBXNAVPVT->data.numSV,
        myGNSS.packetUBXNAVPVT->data.fixType);
  }
  Serial.println("--------------------------------------------------");

  gpsUpdateCount = 0;
  gnssUpdateCount = 0;
  lastGpsRateCheckTime = millis();
}

// --- LED Status ---
void updateLEDs(uint8_t fixType) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  // 1. HIGHEST PRIORITY: Critical Battery Alert
  if (isCritical && !isCharging()) {
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledState = !ledState;

      // Blink Red, keep Green off during critical alert
      digitalWrite(LED_RED, ledState);
      digitalWrite(LED_GREEN, HIGH);
    }
    return; // Exit early so GPS logic doesn't overrule the blink
  }

  // 2. SECOND PRIORITY: GPS Disabled
  if (!gpsEnabled) {
    digitalWrite(LED_RED, HIGH);   // OFF
    digitalWrite(LED_GREEN, HIGH); // OFF
    return;
  }

  // 3. LOWEST PRIORITY: Standard GPS Status
  switch (fixType) {
  case 3:                         // 3D Fix
  case 4:                         // GNSS + DR
    digitalWrite(LED_RED, HIGH);  // Red OFF
    digitalWrite(LED_GREEN, LOW); // Green ON
    break;

  case 1:                         // DR only
  case 2:                         // 2D Fix
    digitalWrite(LED_RED, LOW);   // Red ON
    digitalWrite(LED_GREEN, LOW); // Green ON -> Orange/Yellow
    break;

  default:                         // No fix
    digitalWrite(LED_RED, LOW);    // Red ON
    digitalWrite(LED_GREEN, HIGH); // Green OFF
    break;
  }
}

// BLE Connection Callbacks
void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  lastActivityTime = millis();
  lastDisconnectTime = millis();

  digitalWrite(OnboardledPin, LOW); // Solid Blue ON when connected
  Serial.println("✅ Client connected!");

  // Bumping TX power back to 0 dBm for a stable, high-range connection
  Bluefruit.setTxPower(0);

  Bluefruit.Connection(conn_handle)->requestPHY(); // try 2M, harmless if rejected
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(24, 0, 400); // 30ms, 4s sup timeout
  delay(400);
  uint16_t mtu = Bluefruit.Connection(conn_handle)->getMtu();
  Serial.printf(">> Negotiated MTU = %u (need >= 91 to fit 88-byte notify)\n", mtu);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  lastDisconnectTime = millis(); // Start GPS hot timeout timer immediately
  lastActivityTime = millis();   // Count disconnection as activity

  // Turn off Blue LED immediately on disconnect
  digitalWrite(OnboardledPin, HIGH);
  Serial.println("❌ BLE Client disconnected.");
  Serial.printf("🛰️ GPS staying hot for %d minutes...\n",
                (GPS_HOT_TIMEOUT_MS / 60000));
}

void write_callback(uint16_t conn_handle, BLECharacteristic *chr, uint8_t *data,
                    uint16_t len) {
  Serial.print("📨 Received BLE command: ");
  for (int i = 0; i < len; i++)
    Serial.printf("0x%02X ", data[i]);
  Serial.println();
}

void setIMUForSleep() {
  IMU.settings.gyroEnabled = 0;
  IMU.settings.accelEnabled = 0;
  IMU.begin();
  // 52Hz, ±2g
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x30);
  // Enable tap detection
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x20);
  // Enable single+double tap
  IMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);
  // Low power accel
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);
  // Route to INT1
  IMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);
  // Enable wake pin
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);
}

void enterDeepSleep() {
  Serial.println("💤 Entering Deep Sleep (Shake to Wake)...");
  Bluefruit.autoConnLed(false);

  // Turn off all LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  // Ensure GPS is off
  disableGPS();

  // Configure Triggers for Wake-up
  setIMUForSleep(); // Trigger 1: Shake (IMU INT pin)
  pinMode(PIN_CHG,
          INPUT_PULLUP_SENSE); // Trigger 2: Plug-in (Charge pin goes LOW)

  delay(100); // Small delay for I2C to finish and IMU to settle
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  Serial.flush(); // Ensure serial message is sent before power cut
  NRF_POWER->SYSTEMOFF = 1;
}

bool detectNoBatteryAtBoot() { return !EXPECT_BATTERY; }

void setupHardware() {
  pinMode(GPS_EN_PIN, OUTPUT);
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, HIGH);
#endif

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Start LOW & Stay LOW (Safe & Stable)
  pinMode(PIN_HICHG, OUTPUT);
  digitalWrite(PIN_HICHG, LOW);
  pinMode(PIN_CHG, INPUT_PULLUP); // Prevent float current leakage

  Wire.setClock(400000);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  NRF_POWER->DCDCEN = 1; // Enable DC-DC converter (Saves ~30% radio current)
  // Enable REG0 DC-DC if using VDDH (High Voltage Mode)
  if (NRF_POWER->MAINREGSTATUS & (POWER_MAINREGSTATUS_MAINREGSTATUS_High
                                  << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos)) {
    NRF_POWER->DCDCEN0 = 1;
  }

  // Safety: Ensure QSPI Flash CS is High (Deselected) to prevent floating
  // inputs
  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH);

  pinMode(OnboardledPin, OUTPUT);
  digitalWrite(OnboardledPin, HIGH);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
}

void setupBLE() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(LOW_POWER_BT_TX_POWER);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(12, 24);

  // Service Setup
  disService.begin();
  disModel.setProperties(CHR_PROPS_READ);
  disModel.begin();
  disModel.write(DEVICE_NAME);
  disSerial.setProperties(CHR_PROPS_READ);
  disSerial.begin();
  disSerial.write(SERIAL_NUM);
  disFirmware.setProperties(CHR_PROPS_READ);
  disFirmware.begin();
  disFirmware.write(FIRMWARE_VER);
  disManuf.setProperties(CHR_PROPS_READ);
  disManuf.begin();
  disManuf.write(MANUFACTURER);

  rbService.begin();
  rbTx.setProperties(CHR_PROPS_NOTIFY);
  rbTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rbTx.setMaxLen(244);
  rbTx.setCccdWriteCallback([](uint16_t conn, BLECharacteristic* c, uint16_t value){
    Serial.printf(">> CCCD write on rbTx: 0x%04X (notify %s)\n",
      value, (value & 0x0001) ? "ENABLED" : "disabled");
  });
  rbTx.begin();

  rbRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rbRx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rbRx.setWriteCallback(write_callback);
  rbRx.begin();

  rbGnss.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  rbGnss.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n🚀 SYSTEM STARTUP");

  setupHardware();

  Serial.println("🔍 Checking power source...");
  isNoBatteryMode = detectNoBatteryAtBoot();
  if (isNoBatteryMode) {
    batteryConnected = false;
    Serial.println("⚠️ NO BATTERY DETECTED! Booting in USB Always-On mode.");
  } else {
    Serial.println("🔋 Battery detected! Booting in standard/Eco mode.");
  }

  updateBatteryState();
  if (currentBatteryPercentage == 0 && !isNoBatteryMode && !isCharging()) {
    // Flash RED LED for 5 seconds
    // (10 cycles of 250ms ON + 250ms OFF = 5 seconds)
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_RED, LOW); // ON
      delay(250);
      digitalWrite(LED_RED, HIGH); // OFF
      delay(250);
    }
    // Ensure everything is off before sleep
    digitalWrite(LED_RED, HIGH);
    // Enter Deep Sleep
    enterDeepSleep();
  }

  if (IMU.begin() != 0) {
    Serial.println("❌ IMU Init Failed");
  } else {
    imuEnabled = true;
    if (!isNoBatteryMode)
      disableIMU();
  }

  setupBLE();

  // Initial Advertising Setup
  if (isNoBatteryMode) {
    setupAdvertising(0, FAST_ADV_INTERVAL);
    enableGPS(); // Keep it hot from the start
    Serial.println("📡 BLE Broadcast Started (FAST - Always On).");
  } else {
    setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
    disableGPS();
    Serial.println("📡 BLE Broadcast Started (ECO).");
  }

  // Initialize Activity Trackers to current time to prevent immediate timeouts
  // after long boot/standby durations.
  lastActivityTime = millis();
  lastDisconnectTime = millis();
  lastGpsRateCheckTime = millis();

  // Flash GREEN LED 5 times to indicate successful startup
  Serial.println("Startup Complete.");
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_GREEN, LOW); // ON
    delay(500);
    digitalWrite(LED_GREEN, HIGH); // OFF
    delay(500);
  }
}

void loop() {
  bool idle = !deviceConnected && !gpsEnabled && !imuEnabled;

  if (idle && !isNoBatteryMode) {
    manageBatterySampling(); // Always track battery to prevent deep sleep death
    if (isPluggedIn()) {
      reportSystemStats();
    }
    managePower();
    powerDownSensors(); // Enforce shutdown state while in light sleep
    delay(LOOP_SLEEP);
    return;
  }

  processGNSS();
  processIMU();
  managePower();
  reportSystemStats();
  manageBatterySampling();
  updateLEDs(GPSFixType);
  delay(1);
}
