#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
// #include <Adafruit_MMC56x3.h>
#include <Adafruit_HMC5883_U.h>
#include <ICM42688.h>
#include <Servo.h>

/* ===================== FORWARD DECLARATIONS ===================== */
void readGPS();
void stopMotors();
void updateGpsLed();
void gpsLedBlink();
void setCurrentType();
void manualControl(int throttle, int yaw);
void autonomousHold();
void printCurrentDebug();
void sendTelemetry();
void driveMotors(int throttle, int yaw, float gyroRate = 0);
void updateCurrentEstimate(double lat, double lon, bool motorsStopped);
float readCompass();
float readGyroYawRate();
float headingError(float desired, float current);
void handleCalibration(uint16_t ch7);
void calibrateCompass();
void calibrateGyroscope();

/* ===================== PIN DEFINITIONS (STM32) ===================== */

#define PPM_PIN           PA0     // Receiver connection PPM
#define ESC_PORT_PIN      PA8     // ESC connection for port - babord
#define ESC_STARBOARD_PIN PA9     // ESC connection for starboard - tribord
#define LED_GPS           PC13    // LED GPS status - no LED, no fix, blinking LED GPS working, LED steady GPS valid fix

// I2C1 pins for Compass (HMC5883)
#define I2C_SDA           PB7     // I2C1 Data
#define I2C_SCL           PB6     // I2C1 Clock

// SPI1 pins for Gyroscope (ICM42688P)
#define GYRO_CS_PIN       PA4     // SPI1 Chip Select
// PA5: SCK (automatic), PA6: MISO (automatic), PA7: MOSI (automatic)

#define GPS_SERIAL Serial1          // Built-in USART1: PA9 TX / PA10 RX
#define HC12_SERIAL Serial2         // Built-in USART2: PA2 TX / PA3 RX

/* ===================== GLOBAL OBJECTS ===================== */

/* ===================== CONSTANTS ===================== */

#define GPS_BAUD 115200         // valid for fast GPS, most GPS work at 9600 bauds
#define PPM_CHANNELS 8            // There is a total mas of 8 channels 0 to 7 in the array

#define RC_MIN      1000
#define RC_MAX      2000
#define RC_NEUTRAL  1500

#define SYNC_THRESHOLD   3000
#define MIN_VALID_PULSE   600
#define MAX_VALID_PULSE  2500

#define GPS_HDOP_MAX  1.0         // if HDOP is greater than 1, FIX is lousy

#define CURRENT_SAMPLE_TIME_MS 5000
#define CURRENT_LOCK_THRESHOLD 3.0

#define DEADBAND 75             // Manual control deadband: values within ±75 treated as zero

#define HC12_BAUD 38400           // HC12 baud rate
#define TELEMETRY_INTERVAL_MS 1000 // Send telemetry every second

/* ===================== OBJECTS ===================== */

TinyGPSPlus gps;
// Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
Adafruit_HMC5883_Unified mmc = Adafruit_HMC5883_Unified(12345);
ICM42688 icm(SPI, GYRO_CS_PIN);  // ICM42688P via SPI1

Servo escPort;
Servo escStarboard;

/* ===================== GLOBALS ===================== */

volatile uint16_t ppm[PPM_CHANNELS];
volatile uint8_t ppmIndex = 0;

double targetLat = 0;
double targetLon = 0;

unsigned long ledTimer = 0;
bool ledState = false;
unsigned long telemetryTimer = 0;

enum Mode { MODE_AUTONOMOUS, MODE_MANUAL };
Mode currentMode = MODE_MANUAL;
Mode previousMode = MODE_MANUAL;

// Current estimation
double driftLat = 0, driftLon = 0;
unsigned long driftTimer = 0;
float currentDir = 0;
float currentStrength = 0;
bool currentValid = false;

// Control parameters
float HOLD_RADIUS  = 2.0;
float DIST_GAIN    = 1.5;
float YAW_GAIN     = 1.4;
float MAX_THRUST   = 320;
float MAX_YAW      = 180;
float MIN_THRUST   = 120;
float GYRO_DAMPING = 15.0;  // Rate damping gain to prevent spinning

// Sensor availability flags
bool compassAvailable = false;
bool gyroAvailable = false;

// Calibration state
enum CalMode { CAL_NONE, CAL_COMPASS, CAL_GYRO };
CalMode calibrationMode = CAL_NONE;
float compassOffsetX = 0, compassOffsetY = 0, compassOffsetZ = 0;
bool calibrationInProgress = false;
unsigned long calibrationStartTime = 0;

enum Type { CALM_WATERS, LOW_CURRENT, STRONG_CURRENT };
Type currentType = CALM_WATERS;

/* ===================== PPM ISR ===================== */

void ppmISR() {
  static uint32_t lastPPM = 0;
  uint32_t now = micros();
  uint32_t diff = now - lastPPM;
  lastPPM = now;

  if (diff > 0x80000000UL) return;

  if (diff > SYNC_THRESHOLD) {
    ppmIndex = 0;
  } else if (ppmIndex < PPM_CHANNELS) {
    if (diff >= MIN_VALID_PULSE && diff <= MAX_VALID_PULSE) {
      ppm[ppmIndex++] = diff;
    }
  }
}

/* ===================== SETUP ===================== */

void setup() {
  pinMode(LED_GPS, OUTPUT);
  
  // Blink LED 3 times to show code is running
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GPS, LOW);  // LED on
    delay(100);
    digitalWrite(LED_GPS, HIGH); // LED off
    delay(100);
  }

  Serial.begin(115200);        // USB debug
  delay(500); // Wait for serial to be ready
  
  // Configure USART1 RX pin explicitly for GPS (PA10)
  Serial1.setRx(PA10);
  Serial1.begin(GPS_BAUD);     // GPS UART
  
  Serial2.begin(HC12_BAUD);    // HC12 wireless module
  
  delay(500); // Wait for all serials to be ready
  
  // Send startup message
  Serial2.println("\n=== BOUEE REGATE INIT ===");
  
  // Blink LED slowly to confirm we're in setup
  digitalWrite(LED_GPS, LOW);
  delay(200);
  digitalWrite(LED_GPS, HIGH);

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(PPM_PIN, ppmISR, RISING);

  escPort.attach(ESC_PORT_PIN);
  escStarboard.attach(ESC_STARBOARD_PIN);
  stopMotors();
  Serial2.println("[OK] Motors");
  
  // LED blink pattern: 2 quick blinks = motors OK
  digitalWrite(LED_GPS, LOW); delay(50); digitalWrite(LED_GPS, HIGH); delay(50);
  digitalWrite(LED_GPS, LOW); delay(50); digitalWrite(LED_GPS, HIGH); delay(200);

  Wire.begin();
  delay(100); // Let I2C stabilize
  
  Serial2.println("[CHK] Compass...");
  if (!mmc.begin()) {
    Serial2.println("[FAIL] Compass");
    compassAvailable = false;
  } else {
    Serial2.println("[OK] Compass");
    compassAvailable = true;
  }
  
  // LED pattern: 3 quick blinks if compass OK
  if (compassAvailable) {
    for(int i=0; i<3; i++) {
      digitalWrite(LED_GPS, LOW); delay(50); digitalWrite(LED_GPS, HIGH); delay(50);
    }
  }
  
  // Initialize ICM42688P gyroscope via SPI
  Serial2.println("[CHK] Gyro SPI...");
  int status = icm.begin();
  
  if (status < 0) {
    Serial2.print("[FAIL] Gyro err:");
    Serial2.println(status);
    gyroAvailable = false;
  } else {
    icm.setAccelFS(ICM42688::gpm4);
    icm.setGyroFS(ICM42688::dps250);
    Serial2.println("[OK] Gyro");
    gyroAvailable = true;
  }

  // Wait for GPS fix (with timeout for debugging)
  Serial2.println("[GPS] Waiting...");
  unsigned long gpsWaitStart = millis();
  unsigned long gpsTimeout = 20000; // 20 second timeout for testing
  while ((!gps.location.isValid() || gps.hdop.hdop() > GPS_HDOP_MAX) && (millis() - gpsWaitStart < gpsTimeout)) {
    readGPS();
    gpsLedBlink();
    
    // Echo raw GPS data to Serial2 for diagnostics
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial2.write(c);  // Echo GPS to HC12/Serial2
      gps.encode(c);
    }
    
    // Print status every 5 seconds
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 5000) {
      lastStatus = millis();
      Serial2.print("[GPS] S:");
      Serial2.print(gps.satellites.value());
      Serial2.print(" H:");
      Serial2.println(gps.hdop.hdop());
    }
  }
  
  if (!gps.location.isValid()) {
    Serial2.println("[WARN] GPS timeout");
  }

  digitalWrite(LED_GPS, LOW);
  Serial2.println("[OK] GPS");
  Serial2.println("=== READY ===");

  targetLat = gps.location.lat();
  targetLon = gps.location.lng();
}

/* ===================== LOOP ===================== */

void loop() {
  readGPS();
  updateGpsLed();

  uint16_t ch1 = ppm[0]; // yaw
  uint16_t ch3 = ppm[2]; // throttle
  uint16_t ch5 = ppm[4]; // mode
  uint16_t ch6 = ppm[5]; // current type
  uint16_t ch7 = ppm[6]; // calibration switch

  // Handle calibration switch (ch7)
  handleCalibration(ch7);

  if (ch6 < 1300) currentType = CALM_WATERS;
  else if (ch6 > 1700) currentType = STRONG_CURRENT;
  else currentType = LOW_CURRENT;

  setCurrentType();

  currentMode = (ch5 > 1500) ? MODE_AUTONOMOUS : MODE_MANUAL;

  // Detect mode transition: MANUAL -> AUTO
  if (currentMode == MODE_AUTONOMOUS && previousMode == MODE_MANUAL) {
    // Capture current position as new target
    if (gps.location.isValid()) {
      targetLat = gps.location.lat();
      targetLon = gps.location.lng();
      HC12_SERIAL.print("[AUTO] Target set: ");
      HC12_SERIAL.print(targetLat, 6);
      HC12_SERIAL.print(",");
      HC12_SERIAL.println(targetLon, 6);
    } else {
      HC12_SERIAL.println("[AUTO] No GPS fix!");
    }
  }
  
  previousMode = currentMode;

  if (currentMode == MODE_MANUAL) {
    manualControl(ch3, ch1);
  } else {
    autonomousHold();
  }

  static unsigned long dbg = 0;
  if (millis() - dbg > 2000) {
    dbg = millis();
    printCurrentDebug();
  }
  
  // Send telemetry every second
  if (millis() - telemetryTimer >= TELEMETRY_INTERVAL_MS) {
    telemetryTimer = millis();
    sendTelemetry();
  }
}

/* ===================== MODES ===================== */

void manualControl(int throttleRC, int yawRC) {
  int throttle = map(throttleRC, RC_MIN, RC_MAX, -500, 500);
  int yaw = map(yawRC, RC_MIN, RC_MAX, -500, 500);
  
  // Read gyro for rate damping
  float gyroRate = readGyroYawRate();
  driveMotors(throttle, yaw, gyroRate);
}

void autonomousHold() {
  if (!gps.location.isValid()) {
    stopMotors();
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();

  float distance = TinyGPSPlus::distanceBetween(lat, lon, targetLat, targetLon);
  bool motorsStopped = (distance < HOLD_RADIUS);
  updateCurrentEstimate(lat, lon, motorsStopped);

  if (distance < HOLD_RADIUS) {
    stopMotors();
    return;
  }

  float bearing = TinyGPSPlus::courseTo(lat, lon, targetLat, targetLon);
  float heading = readCompass();
  float gyroRate = readGyroYawRate();  // Read gyro for rate damping
  float desiredBearing = bearing;

  if (currentValid) {
    float upstream = fmod(currentDir + 180.0, 360.0);
    float weight = constrain(currentStrength / 3.0, 0.0, 1.0);
    desiredBearing = (1.0 - weight) * bearing + weight * upstream;
  }

  float error = headingError(desiredBearing, heading);
  if (abs(error) < 5) error = 0;

  int yawCmd = constrain(error * YAW_GAIN, -MAX_YAW, MAX_YAW);
  int thrust = constrain(distance * DIST_GAIN * 80, MIN_THRUST, MAX_THRUST);

  if (currentValid && currentStrength > CURRENT_LOCK_THRESHOLD) {
    thrust = MAX_THRUST;
    error = headingError(currentDir + 180.0, heading);
    yawCmd = constrain(error * YAW_GAIN, -MAX_YAW, MAX_YAW);
  }

  driveMotors(thrust, yawCmd, gyroRate);
}

/* ===================== HELPERS ===================== */

void readGPS() {
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
  }
}

float readCompass() {
  if (!compassAvailable) return 0.0; // Return 0 if compass not available
  
  sensors_event_t event;
  mmc.getEvent(&event);
  
  // Apply calibration offsets
  float x = event.magnetic.x - compassOffsetX;
  float y = event.magnetic.y - compassOffsetY;
  
  float heading = atan2(y, x) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float readGyroYawRate() {
  if (!gyroAvailable) return 0.0; // Return 0 if gyro not available
  
  icm.getAGT();
  // Return yaw rate in degrees per second (Z-axis)
  return icm.gyrZ();
}

float headingError(float target, float current) {
  float e = target - current;
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

void driveMotors(int throttle, int yaw, float gyroRate) {
  // Apply deadband to prevent motor jitter (applies to both manual and auto modes)
  if (abs(throttle) < DEADBAND) throttle = 0;
  if (abs(yaw) < DEADBAND) yaw = 0;
  
  // Apply rate damping: reduce yaw command proportional to rotation rate
  // This opposes rapid spinning and provides stability
  int dampingCorrection = -gyroRate * GYRO_DAMPING;
  int totalYaw = yaw + dampingCorrection;
  totalYaw = constrain(totalYaw, -500, 500);
  
  escPort.writeMicroseconds(constrain(RC_NEUTRAL + throttle + totalYaw, 1000, 2000));
  escStarboard.writeMicroseconds(constrain(RC_NEUTRAL + throttle - totalYaw, 1000, 2000));
}

void stopMotors() {
  escPort.writeMicroseconds(RC_NEUTRAL);
  escStarboard.writeMicroseconds(RC_NEUTRAL);
}

void updateCurrentEstimate(double lat, double lon, bool motorsStopped) {
  if (!motorsStopped) {
    driftTimer = 0;
    currentValid = false;
    return;
  }

  if (driftTimer == 0) {
    driftLat = lat;
    driftLon = lon;
    driftTimer = millis();
  }

  if (millis() - driftTimer >= CURRENT_SAMPLE_TIME_MS) {
    currentStrength = TinyGPSPlus::distanceBetween(driftLat, driftLon, lat, lon);
    currentDir = TinyGPSPlus::courseTo(driftLat, driftLon, lat, lon);
    currentValid = true;
    driftTimer = 0;
  }
}

void setCurrentType() {
  if (currentType == CALM_WATERS) {
    HOLD_RADIUS = 2.0; DIST_GAIN = 1.5; YAW_GAIN = 1.4; MAX_THRUST = 300; MIN_THRUST = 120;
  } else if (currentType == LOW_CURRENT) {
    HOLD_RADIUS = 3.0; DIST_GAIN = 1.8; YAW_GAIN = 1.2; MAX_THRUST = 360; MIN_THRUST = 100;
  } else {
    HOLD_RADIUS = 3.5; DIST_GAIN = 2.0; YAW_GAIN = 1.0; MAX_THRUST = 420; MIN_THRUST = 120;
  }
}

void printCurrentDebug() {
  if (!currentValid) return;
  Serial.print("CURRENT | ");
  Serial.print(currentStrength, 2);
  Serial.print(" m | Dir ");
  Serial.println(currentDir, 1);
}

void sendTelemetry() {
  float heading = readCompass();
  
  // Always send heading and status
  HC12_SERIAL.print("HDG:");
  HC12_SERIAL.print(heading, 1);
  HC12_SERIAL.print(" Mode:");
  HC12_SERIAL.print(currentMode == MODE_MANUAL ? "MAN" : "AUTO");
  HC12_SERIAL.print(" Gyro:");
  HC12_SERIAL.print(gyroAvailable ? "OK" : "--");
  HC12_SERIAL.print(" Water:");
  if (currentType == CALM_WATERS) HC12_SERIAL.print("CALM");
  else if (currentType == STRONG_CURRENT) HC12_SERIAL.print("STRONG");
  else HC12_SERIAL.print("LOW");
  
  // Add GPS data if available
  if (gps.location.isValid()) {
    HC12_SERIAL.print(" LAT:");
    HC12_SERIAL.print(gps.location.lat(), 6);
    HC12_SERIAL.print(" LON:");
    HC12_SERIAL.print(gps.location.lng(), 6);
    HC12_SERIAL.print(" Sats:");
    HC12_SERIAL.print(gps.satellites.value());
  } else {
    HC12_SERIAL.print(" GPS:--");
  }
  
  HC12_SERIAL.println();
}

/* ===================== GPS LED ===================== */

void updateGpsLed() {
  if (gps.location.isValid() && gps.hdop.hdop() <= GPS_HDOP_MAX) {
    digitalWrite(LED_GPS, LOW);
  } else {
    gpsLedBlink();
  }
}

void gpsLedBlink() {
  if (millis() - ledTimer > 500) {
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_GPS, ledState ? LOW : HIGH);
  }
}

/* ===================== CALIBRATION ===================== */

void handleCalibration(uint16_t ch7) {
  CalMode requestedMode = CAL_NONE;
  
  // Decode switch position
  if (ch7 < 1300) {
    requestedMode = CAL_COMPASS;
  } else if (ch7 > 1700) {
    requestedMode = CAL_GYRO;
  } else {
    requestedMode = CAL_NONE;
  }
  
  // Start calibration if mode changed to calibration
  if (requestedMode != CAL_NONE && calibrationMode == CAL_NONE) {
    calibrationMode = requestedMode;
    calibrationInProgress = true;
    calibrationStartTime = millis();
    
    if (calibrationMode == CAL_COMPASS) {
      HC12_SERIAL.println("[CAL] Compass START - rotate 360deg");
      calibrateCompass();
    } else if (calibrationMode == CAL_GYRO) {
      HC12_SERIAL.println("[CAL] Gyro START - keep still");
      calibrateGyroscope();
    }
  }
  
  // Exit calibration mode when switch returns to middle
  if (requestedMode == CAL_NONE && calibrationInProgress) {
    calibrationInProgress = false;
    calibrationMode = CAL_NONE;
    unsigned long duration = (millis() - calibrationStartTime) / 1000;
    HC12_SERIAL.print("[CAL] Complete ");
    HC12_SERIAL.print(duration);
    HC12_SERIAL.println("s");
  }
}

void calibrateCompass() {
  if (!compassAvailable) {
    HC12_SERIAL.println("[CAL] Compass N/A");
    return;
  }
  
  // Get compass reading
  sensors_event_t event;
  mmc.getEvent(&event);
  
  // Simple offset calibration (collects min/max over time)
  static float minX = 1000, maxX = -1000;
  static float minY = 1000, maxY = -1000;
  static float minZ = 1000, maxZ = -1000;
  static bool firstRun = true;
  
  if (firstRun || !calibrationInProgress) {
    // Reset on first calibration or when restarting
    minX = maxX = event.magnetic.x;
    minY = maxY = event.magnetic.y;
    minZ = maxZ = event.magnetic.z;
    firstRun = false;
  }
  
  // Update min/max
  if (event.magnetic.x < minX) minX = event.magnetic.x;
  if (event.magnetic.x > maxX) maxX = event.magnetic.x;
  if (event.magnetic.y < minY) minY = event.magnetic.y;
  if (event.magnetic.y > maxY) maxY = event.magnetic.y;
  if (event.magnetic.z < minZ) minZ = event.magnetic.z;
  if (event.magnetic.z > maxZ) maxZ = event.magnetic.z;
  
  // Calculate offsets (hard iron calibration)
  compassOffsetX = (maxX + minX) / 2.0;
  compassOffsetY = (maxY + minY) / 2.0;
  compassOffsetZ = (maxZ + minZ) / 2.0;
  
  // Feedback every 2 seconds
  static unsigned long lastFeedback = 0;
  if (millis() - lastFeedback > 2000) {
    lastFeedback = millis();
    HC12_SERIAL.print("[CAL] Off:");
    HC12_SERIAL.print(compassOffsetX, 1);
    HC12_SERIAL.print(",");
    HC12_SERIAL.print(compassOffsetY, 1);
    HC12_SERIAL.print(",");
    HC12_SERIAL.println(compassOffsetZ, 1);
  }
}

void calibrateGyroscope() {
  if (!gyroAvailable) {
    HC12_SERIAL.println("[CAL] Gyro N/A");
    return;
  }
  
  // Use the library's built-in calibration
  int status = icm.calibrateGyro();
  
  if (status < 0) {
    HC12_SERIAL.print("[CAL] Gyro FAIL:");
    HC12_SERIAL.println(status);
  } else {
    HC12_SERIAL.print("[CAL] Gyro OK bias:");
    HC12_SERIAL.print(icm.getGyroBiasX(), 2);
    HC12_SERIAL.print(",");
    HC12_SERIAL.print(icm.getGyroBiasY(), 2);
    HC12_SERIAL.print(",");
    HC12_SERIAL.println(icm.getGyroBiasZ(), 2);
  }
}
