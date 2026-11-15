/* ==============================================================================
   DECLIVOLT: Downhill Energy Recovery & Smart Vehicle Control System
   ESP32 Firmware (Arduino IDE Sketch)

   Hardware Integration:
   1. L298N Dual H-Bridge Motor Driver (Drive Motors)
      - ENA: GPIO 25 | IN1: GPIO 26 | IN2: GPIO 27 (Motor 1 - OUT1/OUT2)
      - ENB: GPIO 33 | IN3: GPIO 14 | IN4: GPIO 13 (Motor 2 - OUT3/OUT4)
   2. MPU6050 6-Axis Motion Sensor (Slope & Incline Detection)
      - SDA: GPIO 21 | SCL: GPIO 22 | I2C Address: 0x68
   3. DC Voltage Sensor (Wheel-Connected Generator Motors Recovery)
      - Signal Out: GPIO 34 (ADC1)
   4. LM393 IR Slot Speed Sensor (Wheel Speed / RPM Measurement)
      - VCC: 3.3V | GND: GND | D0: GPIO 32 (Interrupt) | A0: Not connected

   Serial Communication:
   - Baud Rate: 115200 Baud
   - Output: Periodic newline-terminated JSON telemetry packets
   - Input Commands: 'F' (Forward), 'B' (Backward), 'S' (Stop)
   ============================================================================== */

#include <Wire.h>
#include <math.h>

// -----------------------------------------------------------------------------
// 1. PIN CONFIGURATIONS
// -----------------------------------------------------------------------------
// L298N Motor Driver Pins
const int PIN_ENA = 25; // Speed enable for Motor 1
const int PIN_IN1 = 26; // Direction control 1
const int PIN_IN2 = 27; // Direction control 2

const int PIN_ENB = 33; // Speed enable for Motor 2
const int PIN_IN3 = 14; // Direction control 3
const int PIN_IN4 = 13; // Direction control 4

// DC Voltage Sensor (Connected to Generator Motors)
const int PIN_VOLTAGE = 34; // ESP32 ADC1 input (safe 0-3.3V range via divider)

// LM393 IR Slot Speed Sensor Pin (Digital D0 output)
const int PIN_SPEED_SENSOR = 32; // Interrupt-capable digital GPIO

// MPU6050 I2C Configuration
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint8_t MPU6050_ADDR = 0x68;

// -----------------------------------------------------------------------------
// 2. SENSOR & RECOVERY CONSTANTS
// -----------------------------------------------------------------------------
// Voltage calculation factor (Configurable for multimeter calibration)
const float VOLTAGE_FACTOR = 7.5f;
const int MIN_ADC_VALUE = 20; // Ignore electrical noise floor when stationary

// Speed sensor disc configuration
const int PULSES_PER_REV = 20; // Number of slots in encoder disc (standard 20-slot disc)
const unsigned long RPM_CALC_INTERVAL_MS = 200; // Recalculate RPM every 200 ms

// Slope detection thresholds (in degrees)
const float SLOPE_THRESHOLD = 3.0f;  // ±3.0 degrees threshold
const float HYSTERESIS_MARGIN = 1.0f; // 1.0 degree hysteresis band to prevent jitter

// Telemetry transmit interval (in milliseconds)
const unsigned long TELEMETRY_INTERVAL_MS = 150;

// -----------------------------------------------------------------------------
// 3. GLOBAL VARIABLES & STATE MACHINE
// -----------------------------------------------------------------------------
enum SlopeStatus {
  SLOPE_FLAT,
  SLOPE_UPHILL,
  SLOPE_DOWNHILL
};

SlopeStatus currentSlopeState = SLOPE_FLAT;

// Calibration offset computed on a flat surface during setup
float pitchOffset = 0.0f;

// Telemetry timer
unsigned long lastTelemetryTime = 0;

// Speed sensor pulse counter & RPM tracking
volatile unsigned long speedPulseCount = 0;
unsigned long lastRpmCalcTime = 0;
float currentWheelRpm = 0.0f;

// Current motor state tracking
char currentMotorState = 'S'; // 'S' = Stop, 'F' = Forward, 'B' = Backward

// -----------------------------------------------------------------------------
// 4. INTERRUPT SERVICE ROUTINE FOR SPEED SENSOR
// -----------------------------------------------------------------------------
void IRAM_ATTR speedPulseISR() {
  speedPulseCount++;
}

// -----------------------------------------------------------------------------
// 5. MOTOR CONTROL FUNCTIONS
// -----------------------------------------------------------------------------
void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, LOW);

  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENB, LOW);

  currentMotorState = 'S';
}

void forwardMotors() {
  // Motor 1 Forward
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, HIGH);

  // Motor 2 Forward
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENB, HIGH);

  currentMotorState = 'F';
}

void backwardMotors() {
  // Motor 1 Backward
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_ENA, HIGH);

  // Motor 2 Backward
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  digitalWrite(PIN_ENB, HIGH);

  currentMotorState = 'B';
}

// -----------------------------------------------------------------------------
// 6. MPU6050 FUNCTIONS
// -----------------------------------------------------------------------------
bool initMPU6050() {
  Wire.begin(PIN_SDA, PIN_SCL, 100000); // 100 kHz I2C clock
  
  // Test connection and wake up MPU6050 (write 0 to PWR_MGMT_1 register 0x6B)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to zero (wakes up the MPU-6050)
  byte error = Wire.endTransmission();
  
  return (error == 0);
}

bool readMPU6050Accel(int16_t &rawX, int16_t &rawY, int16_t &rawZ) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for Accel readings (ACCEL_XOUT_H)
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Request 6 bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
  if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)6) == 6) {
    rawX = (int16_t)((Wire.read() << 8) | Wire.read());
    rawY = (int16_t)((Wire.read() << 8) | Wire.read());
    rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }
  return false;
}

// Compute raw pitch from accelerometer readings
float calculateRawPitch(float ax, float ay, float az) {
  // Standard accelerometer pitch calculation along the vehicle's travel axis
  // Pitch = atan2(ax, sqrt(ay*ay + az*az)) in degrees
  float denominator = sqrt(ay * ay + az * az);
  if (denominator == 0.0f) {
    return 0.0f;
  }
  return atan2(ax, denominator) * (180.0f / M_PI);
}

// Calibrate flat surface at startup by averaging 60 samples
void calibrateFlatSurface() {
  int validSamples = 0;
  float totalPitch = 0.0f;

  for (int i = 0; i < 60; i++) {
    int16_t rx, ry, rz;
    if (readMPU6050Accel(rx, ry, rz)) {
      float ax = rx / 16384.0f;
      float ay = ry / 16384.0f;
      float az = rz / 16384.0f;
      totalPitch += calculateRawPitch(ax, ay, az);
      validSamples++;
    }
    delay(10);
  }

  if (validSamples > 0) {
    pitchOffset = totalPitch / validSamples;
  } else {
    pitchOffset = 0.0f;
  }
}

// Slope status with hysteresis to prevent oscillation near the threshold
// Slope status mapping:
// Pitch above +3.0 deg = UPHILL
// Pitch below -3.0 deg = DOWNHILL
// Between -3.0 and +3.0 deg = FLAT
void updateSlopeStatus(float correctedPitch) {
  if (currentSlopeState == SLOPE_FLAT) {
    if (correctedPitch > SLOPE_THRESHOLD) {
      currentSlopeState = SLOPE_UPHILL;
    } else if (correctedPitch < -SLOPE_THRESHOLD) {
      currentSlopeState = SLOPE_DOWNHILL;
    }
  } else if (currentSlopeState == SLOPE_UPHILL) {
    // Hysteresis release condition (must return below +2.0 deg)
    if (correctedPitch < (SLOPE_THRESHOLD - HYSTERESIS_MARGIN)) {
      if (correctedPitch < -SLOPE_THRESHOLD) {
        currentSlopeState = SLOPE_DOWNHILL;
      } else {
        currentSlopeState = SLOPE_FLAT;
      }
    }
  } else if (currentSlopeState == SLOPE_DOWNHILL) {
    // Hysteresis release condition (must return above -2.0 deg)
    if (correctedPitch > (-SLOPE_THRESHOLD + HYSTERESIS_MARGIN)) {
      if (correctedPitch > SLOPE_THRESHOLD) {
        currentSlopeState = SLOPE_UPHILL;
      } else {
        currentSlopeState = SLOPE_FLAT;
      }
    }
  }
}

const char* getSlopeStatusString() {
  switch (currentSlopeState) {
    case SLOPE_UPHILL:   return "UPHILL";
    case SLOPE_DOWNHILL: return "DOWNHILL";
    case SLOPE_FLAT:
    default:             return "FLAT";
  }
}

// -----------------------------------------------------------------------------
// 7. VOLTAGE MEASUREMENT & CHARGING LOGIC
// -----------------------------------------------------------------------------
float measureGeneratedVoltage(int &adcRaw) {
  adcRaw = analogRead(PIN_VOLTAGE);
  float voltage = 0.0f;

  // Preserve tested logic:
  // Discard electrical noise when generator motors are stationary
  if (adcRaw > MIN_ADC_VALUE) {
    float adcVoltage = adcRaw * (3.3f / 4095.0f);
    voltage = adcVoltage * VOLTAGE_FACTOR;
  }

  return voltage;
}

const char* getChargingStatus(float voltage) {
  if (voltage > 1.5f) {
    return "FAST CHARGING";
  } else if (voltage > 0.1f) {
    return "CHARGING";
  } else {
    return "IDLE";
  }
}

// -----------------------------------------------------------------------------
// 8. WHEEL RPM CALCULATION
// -----------------------------------------------------------------------------
void updateWheelRpm(unsigned long currentMillis) {
  unsigned long timeDelta = currentMillis - lastRpmCalcTime;
  if (timeDelta >= RPM_CALC_INTERVAL_MS) {
    noInterrupts();
    unsigned long pulses = speedPulseCount;
    speedPulseCount = 0;
    interrupts();

    // RPM = (pulses / PULSES_PER_REV) * (60000.0 / timeDelta)
    currentWheelRpm = ((float)pulses * 60000.0f) / ((float)PULSES_PER_REV * (float)timeDelta);
    lastRpmCalcTime = currentMillis;
  }
}

// -----------------------------------------------------------------------------
// 9. SERIAL COMMAND HANDLING
// -----------------------------------------------------------------------------
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    // Process single-character motor commands
    if (cmd == 'F' || cmd == 'f') {
      forwardMotors();
    } else if (cmd == 'B' || cmd == 'b') {
      backwardMotors();
    } else if (cmd == 'S' || cmd == 's') {
      stopMotors();
    }
  }
}

// -----------------------------------------------------------------------------
// 10. SETUP & INITIALIZATION
// -----------------------------------------------------------------------------
void setup() {
  // Initialize Serial interface
  Serial.begin(115200);

  // Configure L298N Motor Driver Pins
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Safe default: ensure motors are completely stopped on boot
  stopMotors();

  // Configure Voltage Sensor ADC pin
  pinMode(PIN_VOLTAGE, INPUT);
  // ESP32 ADC configuration: 12-bit resolution (0-4095), 11dB attenuation (up to ~3.3V)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Configure LM393 Speed Sensor (Digital Pin D0) with Interrupt
  pinMode(PIN_SPEED_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED_SENSOR), speedPulseISR, FALLING);

  // Initialize MPU6050
  initMPU6050();

  // Perform flat-surface calibration
  calibrateFlatSurface();

  lastRpmCalcTime = millis();
}

// -----------------------------------------------------------------------------
// 11. MAIN LOOP
// -----------------------------------------------------------------------------
void loop() {
  // Non-blocking serial command reception for low-latency motor control
  handleSerialCommands();

  unsigned long currentMillis = millis();

  // Calculate Wheel RPM continuously based on interrupt pulse accumulation
  updateWheelRpm(currentMillis);

  // Periodic sensor telemetry broadcast
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;

    // 1. Measure Generated Voltage
    int adcValue = 0;
    float voltage = measureGeneratedVoltage(adcValue);
    const char* chargingStatus = getChargingStatus(voltage);

    // 2. Read MPU6050 Accelerometer
    int16_t rawX = 0, rawY = 0, rawZ = 0;
    readMPU6050Accel(rawX, rawY, rawZ);

    // Convert raw reading to g (±2g range => 16384 LSB/g)
    float ax_g = (float)rawX / 16384.0f;
    float ay_g = (float)rawY / 16384.0f;
    float az_g = (float)rawZ / 16384.0f;

    // Calculate pitch and apply flat-surface calibration offset
    float rawPitch = calculateRawPitch(ax_g, ay_g, az_g);
    float correctedPitch = rawPitch - pitchOffset;

    // Update slope state with hysteresis
    updateSlopeStatus(correctedPitch);
    const char* slopeStr = getSlopeStatusString();

    // 3. Construct and Transmit JSON Telemetry Packet
    // Format:
    // {
    //   "voltage": 2.45,
    //   "pitch": 4.2,
    //   "slope": "DOWNHILL",
    //   "x": 0.12,
    //   "y": 0.08,
    //   "z": 0.98,
    //   "adc": 400,
    //   "charging": "FAST CHARGING",
    //   "rpm": 145.0
    // }
    Serial.print("{\"voltage\":");
    Serial.print(voltage, 2);
    Serial.print(",\"pitch\":");
    Serial.print(correctedPitch, 1);
    Serial.print(",\"slope\":\"");
    Serial.print(slopeStr);
    Serial.print("\",\"x\":");
    Serial.print(ax_g, 2);
    Serial.print(",\"y\":");
    Serial.print(ay_g, 2);
    Serial.print(",\"z\":");
    Serial.print(az_g, 2);
    Serial.print(",\"adc\":");
    Serial.print(adcValue);
    Serial.print(",\"charging\":\"");
    Serial.print(chargingStatus);
    Serial.print("\",\"rpm\":");
    Serial.print(currentWheelRpm, 1);
    Serial.println("}");
  }
}
