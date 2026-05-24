#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>
#include <vector>

// --- GPS Configuration ---
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 115200
#define FACTORY_GPS_BAUD 9600
#define MAX_NAVIGATION_RATE 25

SFE_UBLOX_GNSS myGNSS;
HardwareSerial GPS_Serial(2);
// --- Enable GNSS constellations ---
// The specific constellations available and how many you can turn on depend on your u-blox module 
// check this out for which constellations to enable https://app.qzss.go.jp/GNSSView/gnssview.html

#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS
// #define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

constexpr const char* rawDeviceName = "RaceBox Mini 0123456789";

constexpr unsigned long MAX_ALLOWED = 3999999999;

constexpr unsigned long parseSuffix(const char* name) {
    unsigned long val = 0;
    // The suffix starts at index 13 (after "RaceBox Mini ")
    for (int i = 13; i < 23; ++i) {
        val = val * 10 + (name[i] - '0');
    }
    return val;
}

static_assert(parseSuffix(rawDeviceName) <= MAX_ALLOWED, 
              "ERROR: RaceBox Mini number cannot exceed 3999999999 for compatibility with the official RaceBox App");

const String deviceName = rawDeviceName;

const int OnboardledPin = 2;

const unsigned long AccelSampleInterval = 10; // 10ms = 100Hz
// --- Smoothing Configuration ---
// alpha = 1.0: No filtering (raw data)
// alpha = 0.5: 50% current reading, 50% previous (moderate)
// alpha = 0.8: Very snappy, just kills high-frequency "buzz"
float accelAlpha = 0.8; 
float gyroAlpha = 0.8; 
// Storage for the filtered values
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// --- BLE Configuration ---
const char* const RACEBOX_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicTx = NULL;
BLECharacteristic *pCharacteristicRx = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- Packet Timing ---
unsigned long lastPacketSendTime = 0;
const unsigned long PACKET_SEND_INTERVAL_MS = 40;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
const unsigned long GPS_RATE_REPORT_INTERVAL_MS = 5000;
unsigned int gnssUpdateCount = 0;


// --- BLE Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    // Request a larger MTU to fit an 88-byte packet + headers in one go
    pServer->updatePeerMTU(pServer->getConnId(), 128); 
    digitalWrite(OnboardledPin, HIGH);
    Serial.println("✅ BLE Client connected & MTU update requested");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    digitalWrite(OnboardledPin, LOW);
    Serial.println("❌ BLE Client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue(); 
    
    if (rxValue.length() > 0) {
      Serial.print("📨 Received BLE command: ");
      for (size_t i = 0; i < rxValue.length(); i++) {
        Serial.printf("0x%02X ", (uint8_t)rxValue[i]);
      }
      Serial.println();
    }
  }
};

// --- UBX Packet Construction Helpers ---
void writeLittleEndian(uint8_t* buffer, int offset, uint32_t value) { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, int32_t value)  { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, uint16_t value) { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, int16_t value)  { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, uint8_t value)  { buffer[offset] = value; }
void writeLittleEndian(uint8_t* buffer, int offset, int8_t value)   { buffer[offset] = (uint8_t)value; }

void calculateChecksum(uint8_t* payload, uint16_t len, uint8_t cls, uint8_t id, uint8_t* ckA, uint8_t* ckB) {
  *ckA = *ckB = 0;
  *ckA += cls; *ckB += *ckA;
  *ckA += id; *ckB += *ckA;
  *ckA += len & 0xFF; *ckB += *ckA;
  *ckA += len >> 8; *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

void resetGpsBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  GPS_Serial.begin(FACTORY_GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while(1) delay(100);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud!");
  }
  delay(500);

  // Now switch baud rate
  Serial.print("Setting baud rate to ");
  Serial.print(GPS_BAUD);
  Serial.println("...");
  myGNSS.setSerialRate(GPS_BAUD);
  Serial.print("Baud rate changed to ");
  Serial.println(GPS_BAUD);

  GPS_Serial.end();
  delay(100);
  // Re-initialize the serial port at the new baud rate
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("GNSS not detected at ");
    Serial.print(GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while (1) delay(100);
  }
  Serial.print("GNSS detected at ");
  Serial.print(GPS_BAUD);
  Serial.println(" baud! Saving to Flash");
  myGNSS.saveConfiguration(); // Save to flash
  GPS_Serial.end();
}

// --- Universal MPU Class ---
class UniversalMPU {
public:
    Adafruit_MPU6050 mpu; // The actual library object

    bool begin(uint8_t addr = 0x68) {
        Wire.begin();
        
        // 1. Manual WHO_AM_I check
        Wire.beginTransmission(addr);
        Wire.write(0x75);
        if (Wire.endTransmission() != 0) return false; // Device not found
        
        Wire.requestFrom(addr, (uint8_t)1);
        uint8_t chipID = Wire.read();

        Serial.print("🔍 Detected Chip ID: 0x");
        Serial.println(chipID, HEX);

        // 2. If it's a 6500 (0x70) or 9250 (0x71), we "trick" the library
        // We do this by initializing it, but we handle the failure
        if (chipID == 0x70 || chipID == 0x71) {
            Serial.println("Found MPU6500/9250. Applying bypass...");
            // We still call begin, but it might return false because of the ID check.
            // So we ignore the return value and manually force the power settings.
            mpu.begin(addr); 
            
            // Force wake up (Power Management 1 register 0x6B set to 0)
            Wire.beginTransmission(addr);
            Wire.write(0x6B);
            Wire.write(0x00);
            Wire.endTransmission();
            return true;
        }

        // 3. If it's a standard 6050, just let the library do its thing
        return mpu.begin(addr);
    }

    // Pass-through functions to keep your existing code working
    void setAccelerometerRange(mpu6050_accel_range_t r) { mpu.setAccelerometerRange(r); }
    void setGyroRange(mpu6050_gyro_range_t r) { mpu.setGyroRange(r); }
    void setFilterBandwidth(mpu6050_bandwidth_t b) { mpu.setFilterBandwidth(b); }
    void getEvent(sensors_event_t* a, sensors_event_t* g, sensors_event_t* t) { mpu.getEvent(a, g, t); }
};

UniversalMPU mpu; // Still named 'mpu' so your setup() code doesn't have to change

void setup() {
  Serial.begin(115200);
  pinMode(OnboardledPin, OUTPUT);
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU chip");
    while (1) delay(100);
  }
  
  Serial.println("✅ MPU Sensor Online!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize filters with the first real reading so they don't start at zero
  filtered_ax = a.acceleration.x;
  filtered_ay = a.acceleration.y;
  filtered_az = a.acceleration.z;

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(GPS_Serial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    GPS_Serial.end();
    resetGpsBaudRate();
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    // --- Configure GPS update rate to MAX_NAVIGATION_RATE Hz ---
  if (myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE)) {
  Serial.printf("✅ GPS update rate set to %d Hz.\n",MAX_NAVIGATION_RATE );
  } else {
    Serial.println("❌ Failed to set GPS update rate.");
  }

  // --- GNSS Constellation Setup ---

  // GPS
  #ifdef ENABLE_GNSS_GPS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)) {
      Serial.println("✅ GPS enabled.");
    } else {
      Serial.println("❌ Failed to enable GPS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
    Serial.println("🚫 GPS disabled.");
  #endif

  // Galileo
  #ifdef ENABLE_GNSS_GALILEO
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO)) {
      Serial.println("✅ Galileo enabled.");
    } else {
      Serial.println("❌ Failed to enable Galileo.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
    Serial.println("🚫 Galileo disabled.");
  #endif

  // GLONASS
  #ifdef ENABLE_GNSS_GLONASS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS)) {
      Serial.println("✅ GLONASS enabled.");
    } else {
      Serial.println("❌ Failed to enable GLONASS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
    Serial.println("🚫 GLONASS disabled.");
  #endif

  // BeiDou
  #ifdef ENABLE_GNSS_BEIDOU
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU)) {
      Serial.println("✅ BEIDOU enabled.");
    } else {
      Serial.println("❌ Failed to enable BEIDOU.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
    Serial.println("🚫 BEIDOU disabled.");
  #endif

  // Optional: QZSS
  #ifdef ENABLE_GNSS_QZSS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS)) {
      Serial.println("✅ QZSS enabled.");
    } else {
      Serial.println("❌ Failed to enable QZSS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
    Serial.println("🚫 QZSS disabled.");
  #endif

  // Optional: SBAS (satellite-based augmentation)
  #ifdef ENABLE_GNSS_SBAS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS)) {
      Serial.println("✅ SBAS enabled.");
    } else {
      Serial.println("❌ Failed to enable SBAS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
    Serial.println("🚫 SBAS disabled.");
  #endif

  // --- BLE Setup ---
  BLEDevice::init(deviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTx->addDescriptor(new BLE2902());
  pCharacteristicRx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRx->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  // --- Device Information Service ---
  BLEService *pDeviceInfo = pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");
  // Model
  BLECharacteristic *pModel = pDeviceInfo->createCharacteristic("00002a24-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pModel->setValue("RaceBox Mini");
  // Serial number (last 10 digits of device name)
  BLECharacteristic *pSerial = pDeviceInfo->createCharacteristic("00002a25-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  if (deviceName.length() >= 10) {
      pSerial->setValue(deviceName.substring(deviceName.length() - 10));
  } else {
      pSerial->setValue("0000000000");
  }
  // Firmware revision
  BLECharacteristic *pFirm = pDeviceInfo->createCharacteristic("00002a26-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pFirm->setValue("3.3");
  // Hardware revision
  BLECharacteristic *pHardware = pDeviceInfo->createCharacteristic("00002a27-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pHardware->setValue("1");
  // Manufacturer
  BLECharacteristic *pManufacturer = pDeviceInfo->createCharacteristic("00002a29-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue("RaceBox");
  pDeviceInfo->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  BLEAdvertisementData oAdvertisementData;
  oAdvertisementData.setFlags(0x06); // General Discoverable / BR_EDR_NOT_SUPPORTED
  oAdvertisementData.setCompleteServices(BLEUUID(RACEBOX_SERVICE_UUID));
  pAdvertising->setAdvertisementData(oAdvertisementData);
  BLEAdvertisementData oScanResponseData;
  oScanResponseData.setName(deviceName.c_str());
  oScanResponseData.setCompleteServices(BLEUUID("0000180a-0000-1000-8000-00805f9b34fb"));
  pAdvertising->setScanResponseData(oScanResponseData);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Optimization for iOS
  pAdvertising->setMaxPreferred(0x12);
  
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE advertising started with Scan Response.");

  lastGpsRateCheckTime = millis();
}

void loop() {
  myGNSS.checkUblox(); // Required to keep GNSS data flowing
  static unsigned long lastAccelReadMs = 0;
  // Update Accelrometer readings at fixed interval
  if (millis() - lastAccelReadMs >= AccelSampleInterval) {
    lastAccelReadMs += AccelSampleInterval; // Strict timing grid
      lastAccelReadMs = millis();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Apply Exponential Moving Average (Complementary Filter logic)
      filtered_ax = (accelAlpha * a.acceleration.x) + ((1.0 - accelAlpha) * filtered_ax);
      filtered_ay = (accelAlpha * a.acceleration.y) + ((1.0 - accelAlpha) * filtered_ay);
      filtered_az = (accelAlpha * a.acceleration.z) + ((1.0 - accelAlpha) * filtered_az);

      filtered_gx = (gyroAlpha * g.gyro.x) + ((1.0 - gyroAlpha) * filtered_gx);
      filtered_gy = (gyroAlpha * g.gyro.y) + ((1.0 - gyroAlpha) * filtered_gy);
      filtered_gz = (gyroAlpha * g.gyro.z) + ((1.0 - gyroAlpha) * filtered_gz);
  }
  // LED Blink Logic
  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > 500) {
      lastBlinkMs = millis();
      digitalWrite(OnboardledPin, !digitalRead(OnboardledPin));
    }
  } else {
    digitalWrite(OnboardledPin, HIGH);
  }
  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;

    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if (deviceConnected && myGNSS.packetUBXNAVPVT != NULL) {
        const unsigned long now = millis();
        lastPacketSendTime = now;
        gpsUpdateCount++;

        // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
        int16_t gX = filtered_ax * 1000.0 / 9.80665;
        int16_t gY = filtered_ay * 1000.0 / 9.80665;
        int16_t gZ = filtered_az * 1000.0 / 9.80665;

        // Convert gyro to centi-deg/sec
        int16_t rX = filtered_gx * 180.0 / M_PI * 100.0;
        int16_t rY = filtered_gy * 180.0 / M_PI * 100.0;
        int16_t rZ = filtered_gz * 180.0 / M_PI * 100.0;

        uint8_t payload[80] = {0};
        uint8_t packet[88] = {0};

        // Access data directly from myGNSS.packetUBXNAVPVT->data
        writeLittleEndian(payload, 0, myGNSS.packetUBXNAVPVT->data.iTOW);
        writeLittleEndian(payload, 4, myGNSS.packetUBXNAVPVT->data.year);
        writeLittleEndian(payload, 6, myGNSS.packetUBXNAVPVT->data.month);
        writeLittleEndian(payload, 7, myGNSS.packetUBXNAVPVT->data.day);
        writeLittleEndian(payload, 8, myGNSS.packetUBXNAVPVT->data.hour);
        writeLittleEndian(payload, 9, myGNSS.packetUBXNAVPVT->data.min);
        writeLittleEndian(payload, 10, myGNSS.packetUBXNAVPVT->data.sec);

        // Offset 11: Validity Flags (RaceBox Protocol) 
        uint8_t raceboxValidityFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxValidityFlags |= (1 << 0); // Bit 0: valid date 
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxValidityFlags |= (1 << 1); // Bit 1: valid time 
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved 
        writeLittleEndian(payload, 11, raceboxValidityFlags);

        // Offset 12: Time Accuracy (RaceBox Protocol) 
        writeLittleEndian(payload, 12, myGNSS.packetUBXNAVPVT->data.tAcc);

        // Offset 16: Nanoseconds (RaceBox Protocol) 
        writeLittleEndian(payload, 16, myGNSS.packetUBXNAVPVT->data.nano);

        // Offset 20: Fix Status (RaceBox Protocol) 
        writeLittleEndian(payload, 20, myGNSS.packetUBXNAVPVT->data.fixType);

        // Offset 21: Fix Status Flags (RaceBox Protocol)
        uint8_t fixStatusFlagsRacebox = 0;

        if (myGNSS.packetUBXNAVPVT->data.fixType == 3) {
            fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix
        }

        if (myGNSS.getHeadVehValid()) { // Use the confirmed function to check for valid heading
            fixStatusFlagsRacebox |= (1 << 5); // Bit 5: valid heading (as per RaceBox Protocol)
        }
        writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

        // Offset 22: Date/Time Flags (RaceBox Protocol) 
        uint8_t raceboxDateTimeFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxDateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime && myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
        writeLittleEndian(payload, 22, raceboxDateTimeFlags);

        // Offset 23: Number of SVs (RaceBox Protocol) 
        writeLittleEndian(payload, 23, myGNSS.packetUBXNAVPVT->data.numSV);

        // Remaining fields, mostly direct mappings from u-blox data
        writeLittleEndian(payload, 24, myGNSS.packetUBXNAVPVT->data.lon);
        writeLittleEndian(payload, 28, myGNSS.packetUBXNAVPVT->data.lat);
        writeLittleEndian(payload, 32, myGNSS.packetUBXNAVPVT->data.height);
        writeLittleEndian(payload, 36, myGNSS.packetUBXNAVPVT->data.hMSL);

        writeLittleEndian(payload, 40, myGNSS.packetUBXNAVPVT->data.hAcc);
        writeLittleEndian(payload, 44, myGNSS.packetUBXNAVPVT->data.vAcc);
        writeLittleEndian(payload, 48, myGNSS.packetUBXNAVPVT->data.gSpeed);
        writeLittleEndian(payload, 52, myGNSS.packetUBXNAVPVT->data.headMot);
        writeLittleEndian(payload, 56, myGNSS.packetUBXNAVPVT->data.sAcc);
        writeLittleEndian(payload, 60, myGNSS.packetUBXNAVPVT->data.headAcc);

        writeLittleEndian(payload, 64, myGNSS.packetUBXNAVPVT->data.pDOP);

        // Offset 66: Lat/Lon Flags (RaceBox Protocol) 
        uint8_t latLonFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.fixType < 2) { // If no 2D/3D fix, then coordinates are considered invalid 
            latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS Altitude, and MSL Altitude
        }
        writeLittleEndian(payload, 66, latLonFlags);

        // Offset 67: Battery status (1 byte) - report 100% to avoid low battery warnings
        writeLittleEndian(payload, 67, (uint8_t)100);

        writeLittleEndian(payload, 68, gX);
        writeLittleEndian(payload, 70, gY);
        writeLittleEndian(payload, 72, gZ);
        writeLittleEndian(payload, 74, rX);
        writeLittleEndian(payload, 76, rY);
        writeLittleEndian(payload, 78, rZ);

        // Wrap in UBX (standard RaceBox header and checksum)
        packet[0] = 0xB5;
        packet[1] = 0x62;
        packet[2] = 0xFF; // Message Class: RaceBox Data Message 
        packet[3] = 0x01; // Message ID: RaceBox Data Message 
        packet[4] = 80;   // Payload size 
        packet[5] = 0;
        memcpy(packet + 6, payload, 80);
        uint8_t ckA, ckB; 
        calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
        packet[86] = ckA;
        packet[87] = ckB;

        pCharacteristicTx->setValue(packet, 88);
        pCharacteristicTx->notify();
        delay(5);
      }
    }

    // Report packet send rate
    const unsigned long now = millis();
    if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
      float bleRate = gpsUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
      float gnssRate = gnssUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
      // Additional satellite info for debugging: number of satellites, fix type, horizontal accuracy, and lat/lon
      uint8_t sats = 0;
      uint8_t fix = 0;
      uint32_t hAcc = 0;
      double lat = 0.0, lon = 0.0;
      if (myGNSS.packetUBXNAVPVT != NULL) {
        sats = myGNSS.packetUBXNAVPVT->data.numSV;
        fix = myGNSS.packetUBXNAVPVT->data.fixType;
        hAcc = myGNSS.packetUBXNAVPVT->data.hAcc;
        lat = myGNSS.packetUBXNAVPVT->data.lat * 1e-7;
        lon = myGNSS.packetUBXNAVPVT->data.lon * 1e-7;
      }
      Serial.printf("BLE Packet Rate: %.2f Hz | GNSS Update Rate: %.2f Hz | SVs: %u | Fix: %u | HAcc: %u mm | Lat: %.7f Lon: %.7f\n",
                    bleRate, gnssRate, sats, fix, hAcc, lat, lon);
      gpsUpdateCount = 0;
      gnssUpdateCount = 0;
      lastGpsRateCheckTime = now;
    }

    if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
  }
}
