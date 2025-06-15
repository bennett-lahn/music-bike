/*
 * Music Bike Sensor System - Main Arduino Code
 * 
 * This embedded system monitors bike motion and rider actions using:
 * - MPU9250 IMU for orientation (pitch/roll/yaw) and g-force detection
 * - Hall effect sensors for speed measurement and direction detection
 * - OLED display for real-time status information
 * - BLE communication for wireless data transmission
 * - Potentiometers for real-time threshold tuning
 * 
 * The system detects various bike maneuvers:
 * - Jumps (based on g-force drop below threshold)
 * - Drops/Impacts (g-force spikes above threshold) 
 * - 180-degree spins (gyroscope rotation + g-force anomalies)
 * 
 * Architecture: FreeRTOS multi-tasking system with mutex-protected shared data
 * Last Updated: 5/24/25 - Added DLPF (Digital Low Pass Filter) on IMU
 */

// === HARDWARE INTERFACE LIBRARIES ===
#include <Wire.h>                // I2C communication for IMU and OLED
#include <Adafruit_GFX.h>        // Graphics library for OLED display
#include <Adafruit_SSD1306.h>    // OLED display driver (128x64 pixels)

// === BLUETOOTH LOW ENERGY LIBRARIES ===
#include <BLEDevice.h>           // ESP32 BLE device initialization
#include <BLEServer.h>           // BLE server for hosting characteristics
#include <BLEUtils.h>            // BLE utility functions
#include <BLE2902.h>             // BLE descriptor for notifications

// === ESP32 SYSTEM LIBRARIES ===
#include <driver/adc.h>          // ADC configuration for potentiometer readings

// === FREERTOS LIBRARIES ===
#include "freertos/FreeRTOS.h"   // Real-time operating system core
#include "freertos/task.h"       // Task creation and management
#include "freertos/semphr.h"     // Semaphores/mutexes for thread safety



//==============================================================================
// HARDWARE PIN ASSIGNMENTS & CONFIGURATION
//==============================================================================

// === I2C COMMUNICATION PINS ===
#define SDA_PIN 18               // I2C Serial Data line (connect to IMU and OLED)
#define SCL_PIN 15               // I2C Serial Clock line (connect to IMU and OLED)

// === USER INPUT PINS ===
#define ZERO_BUTTON_PIN 8        // Button to zero/calibrate IMU orientation offsets

// === SENSOR INPUT PINS ===
#define HALL_SENSOR_PIN 9        // Primary hall effect sensor for speed detection
#define HALL_SENSOR_PIN_2 46     // Secondary hall sensor for direction detection
                                 // (positioned 90° from primary sensor on wheel)

// === ANALOG INPUT PINS (Potentiometers for real-time threshold tuning) ===
#define JUMP_THRESH_POT_PIN 12   // Potentiometer to adjust jump detection sensitivity
#define LAND_THRESH_POT_PIN 11   // Potentiometer to adjust landing detection sensitivity  
#define DROP_THRESH_POT_PIN 10   // Potentiometer to adjust drop/impact detection sensitivity

// === OUTPUT PINS ===
#define BLE_LED_PIN 3            // LED indicator for BLE connection status

//==============================================================================
// DETECTION THRESHOLDS & TUNING PARAMETERS
//==============================================================================

// === G-FORCE THRESHOLDS (g = 9.8 m/s²) ===
// These are default values - actual values set by potentiometers during runtime
float jumpThreshold = 0.5;      // Jump takeoff: g-force drops below this (weightlessness)
float landingThreshold = 2.0;   // Jump landing: g-force exceeds this (impact)
float dropThreshold = 2.5;      // Drop detection: sudden g-force spike above this

// === POTENTIOMETER MAPPING RANGES ===
// These define the min/max values that potentiometers can set for each threshold
#define JUMP_THRESH_MIN 0.1f     // Minimum jump sensitivity (very sensitive)
#define JUMP_THRESH_MAX 1.5f     // Maximum jump sensitivity (less sensitive)
#define LAND_THRESH_MIN 1.0f     // Minimum landing detection threshold
#define LAND_THRESH_MAX 4.0f     // Maximum landing detection threshold
#define DROP_THRESH_MIN 1.5f     // Minimum drop detection threshold
#define DROP_THRESH_MAX 5.0f     // Maximum drop detection threshold

// === TIMING THRESHOLDS ===
#define JUMP_DURATION_MIN 100                    // Minimum airtime (ms) to qualify as jump
#define DIRECTION_THRESHOLD 0.3                  // IMU direction change sensitivity
#define IMU_DIRECTION_ACCEL_THRESHOLD 0.3        // Acceleration threshold for direction detection

// === 180-DEGREE SPIN DETECTION PARAMETERS ===
#define GFORCE_ANOMALY_UPPER 2.37f              // Upper g-force anomaly threshold
#define GFORCE_ANOMALY_LOWER 0.60f              // Lower g-force anomaly threshold  
#define GFORCE_ANOMALY_WINDOW_MS 1500           // Time window to consider g-force anomalies (ms)
#define YAW_RATE_THRESHOLD_MIN 90.0f            // Minimum rotation rate (deg/s) for spin detection
#define SPIN_SCORE_THRESHOLD 0.8f               // Required spin score to trigger 180° detection
#define SPIN_SCORE_DECAY_PER_SEC (1.0f / 3.0f) // Spin score decay rate (full decay in 3 seconds)
#define TOTAL_ROTATION_THRESHOLD 80.0f          // Minimum rotation (degrees) for 180° detection
#define ROTATION_DECAY_FACTOR 0.99f             // Rotation tracking decay factor per cycle

// === PHYSICAL CONSTANTS (Bike Specifications) ===
#define WHEEL_DIAMETER_INCHES 26.0                              // Bike wheel diameter
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_INCHES * 2.54 * 3.14159)  // Full wheel circumference in cm
#define HALF_CIRCUMFERENCE_CM (WHEEL_CIRCUMFERENCE_CM / 2.0)    // Half circumference (distance between hall sensors)

// === SYSTEM TIMING CONSTANTS ===
const unsigned long SPEED_TIMEOUT = 3000;              // Time (ms) before speed resets to zero
const unsigned long EVENT_DISPLAY_DURATION = 2000;     // Time (ms) to display detected events  
const unsigned long debounceDelay = 50;                // Button debounce delay (ms)
const unsigned long directionDetectionTimeout = 500;   // Timeout for direction detection sequence (ms)

// === MPU9250 IMU REGISTER ADDRESSES ===
#define MPU9250_ADDRESS 0x68    // I2C address of MPU9250 IMU chip
#define ACCEL_XOUT_H 0x3B       // Register address for accelerometer X-axis high byte
#define GYRO_XOUT_H 0x43        // Register address for gyroscope X-axis high byte

// === OLED DISPLAY CONFIGURATION ===
#define SCREEN_WIDTH 128        // OLED display width in pixels
#define SCREEN_HEIGHT 64        // OLED display height in pixels  
#define OLED_RESET -1           // Reset pin (-1 = sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C     // I2C address of OLED display
// Note: Adafruit_SSD1306 display object is declared later globally

//==============================================================================
// BLUETOOTH LOW ENERGY (BLE) CONFIGURATION
// Each characteristic represents a different data stream sent to connected devices
//==============================================================================

// === BLE SERVICE & CHARACTERISTIC UUIDs ===
// Main service UUID that groups all bike sensor characteristics
#define SERVICE_UUID                      "0fb899fa-2b3a-4e11-911d-4fa05d130dc1"

// Data stream characteristics - each sends specific sensor readings
#define SPEED_CHARACTERISTIC_UUID         "a635fed5-9a19-4e31-8091-84d020481329"  // Speed (km/h) from hall sensors
#define PITCH_CHARACTERISTIC_UUID         "726c4b96-bc56-47d2-95a1-a6c49cce3a1f"  // Bike pitch angle (degrees)
#define ROLL_CHARACTERISTIC_UUID          "a1e929e3-5a2e-4418-806a-c50ab877d126"  // Bike roll angle (degrees)  
#define YAW_CHARACTERISTIC_UUID           "cd6fc0f8-089a-490e-8e36-74af84977c7b"  // Bike yaw/heading (degrees)
#define GFORCE_CHARACTERISTIC_UUID        "a6210f30-654f-32ea-9e37-432a639fb38e"  // G-force magnitude

// Event characteristics - send notifications when specific events occur  
#define EVENT_CHARACTERISTIC_UUID         "26205d71-58d1-45e6-9ad1-1931cd7343c3"  // Event codes (jump=1, drop=2, 180=3)

// Direction and state characteristics
#define IMU_DIRECTION_CHARACTERISTIC_UUID "ceb04cf6-0555-4243-a27b-c85986ab4bd7"   // IMU-based direction (1=forward, 0=reverse)
#define HALL_DIRECTION_CHARACTERISTIC_UUID "f231de63-475c-463d-9b3f-f338d7458bb9"  // Hall sensor direction (1=forward, 0=reverse)
#define IMU_SPEED_STATE_CHARACTERISTIC_UUID "738f5e54-5479-4941-ae13-caf4a9b07b2e" // Speed state (0=stop, 1=medium, 2=fast)

//==============================================================================
// GLOBAL VARIABLES & THREAD SYNCHRONIZATION
// Data is organized by protection domain (which mutex guards each data group)
//==============================================================================

// === FREERTOS MUTEX HANDLES ===
// These mutexes protect shared data from concurrent access by multiple tasks
SemaphoreHandle_t imuDataMutex = NULL;        // Protects: pitch, roll, yaw, gForce, accel*, gyro*
SemaphoreHandle_t hallDataMutex = NULL;       // Protects: currentSpeed, hallDirectionForward, hallSensorValue*  
SemaphoreHandle_t eventDataMutex = NULL;      // Protects: jumpDetected, dropDetected, oneEightyDetected, imuDirection*, imuSpeedState
SemaphoreHandle_t offsetMutex = NULL;         // Protects: pitchOffset, rollOffset, yawOffset (calibration values)
SemaphoreHandle_t bleConnectionMutex = NULL;  // Protects: deviceConnected, oldDeviceConnected
SemaphoreHandle_t configMutex = NULL;         // Protects: jumpThreshold, landingThreshold, dropThreshold

// === IMU SENSOR DATA (Protected by imuDataMutex) ===
// Raw orientation angles from MPU9250 sensor
float pitch = 0.0, roll = 0.0, yaw = 0.0;       // Euler angles in degrees
float gForce = 1.0;                             // Vertical g-force magnitude (1.0 = 1g at rest)
float accelX = 0.0, accelY = 0.0, accelZ = 0.0; // Raw scaled accelerometer readings (g's)
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;    // Raw scaled gyroscope readings (deg/s)

// === CALIBRATION OFFSETS (Protected by offsetMutex) ===
// Zero position offsets set by pressing the zero button
float pitchOffset = 0.0, rollOffset = 0.0, yawOffset = 0.0;

// === HALL SENSOR DATA (Protected by hallDataMutex) ===
// Speed and direction data from magnetic hall effect sensors on wheel
float currentSpeed = 0.0;          // Current bike speed in km/h
bool hallDirectionForward = true;  // Direction from hall sensor timing (true=forward, false=reverse)
int hallSensorValue = HIGH;        // Current digital state of primary hall sensor
int hallSensorValue2 = HIGH;       // Current digital state of secondary hall sensor

// === EVENT DETECTION DATA (Protected by eventDataMutex) ===
// Boolean flags for detected bike maneuvers and motion states
bool jumpDetected = false;          // True when jump event is currently active/displaying
bool dropDetected = false;          // True when drop/impact event is currently active/displaying  
bool oneEightyDetected = false;     // True when 180° spin event is currently active/displaying
bool imuDirectionForward = true;    // Direction calculated from IMU acceleration patterns
int imuSpeedState = 0;              // Speed state from IMU: 0=Stop/Slow, 1=Medium, 2=Fast

// === BLE CONNECTION STATE (Protected by bleConnectionMutex) ===
volatile bool deviceConnected = false;  // Current BLE connection status (volatile for ISR safety)
bool oldDeviceConnected = false;        // Previous connection state for change detection

// === BLE OBJECTS (Global, initialized in setup) ===
// These pointers reference BLE server and characteristic objects created during initialization
BLEServer* pServer = NULL;                           // Main BLE server instance
BLECharacteristic* pSpeedCharacteristic = NULL;     // Speed data transmission characteristic  
BLECharacteristic* pPitchCharacteristic = NULL;     // Pitch angle transmission characteristic
BLECharacteristic* pRollCharacteristic = NULL;      // Roll angle transmission characteristic
BLECharacteristic* pYawCharacteristic = NULL;       // Yaw angle transmission characteristic
BLECharacteristic* pGForceCharacteristic = NULL;    // G-force transmission characteristic
BLECharacteristic* pEventCharacteristic = NULL;     // Event notification characteristic
BLECharacteristic* pImuDirectionCharacteristic = NULL;    // IMU-based direction characteristic
BLECharacteristic* pHallDirectionCharacteristic = NULL;   // Hall sensor direction characteristic
BLECharacteristic* pImuSpeedStateCharacteristic = NULL;   // IMU speed state characteristic

// === HARDWARE OBJECTS ===
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // OLED display driver instance

// === FREERTOS TASK HANDLES ===
// These handles allow task management and inter-task communication
TaskHandle_t imuTaskHandle = NULL;          // Handle for IMU reading and orientation calculation task
TaskHandle_t hallSensorTaskHandle = NULL;   // Handle for hall sensor speed/direction detection task  
TaskHandle_t processingTaskHandle = NULL;   // Handle for event detection and button handling task
TaskHandle_t bleTaskHandle = NULL;          // Handle for BLE data transmission task
TaskHandle_t displayTaskHandle = NULL;      // Handle for OLED display update task
TaskHandle_t potTuningTaskHandle = NULL;    // Handle for potentiometer threshold tuning task

//==============================================================================
// BLE CONNECTION CALLBACK HANDLER
// Manages BLE client connection and disconnection events
//==============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    // Called when a BLE client (phone/computer) connects to this device
    void onConnect(BLEServer* pServerInstance) {
        // Thread-safe update of connection status using mutex
        if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = true;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Connected");
        digitalWrite(BLE_LED_PIN, HIGH);  // Turn on LED to indicate connection
    }

    // Called when a BLE client disconnects from this device
    void onDisconnect(BLEServer* pServerInstance) {
        // Thread-safe update of connection status using mutex
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = false;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Disconnected");
        digitalWrite(BLE_LED_PIN, LOW);   // Turn off LED to indicate disconnection
        vTaskDelay(pdMS_TO_TICKS(500));   // Small delay before restart
        pServer->startAdvertising();      // Resume advertising to allow reconnection
        Serial.println("BLE Advertising restarted");
    }
};

//==============================================================================
// FREERTOS TASK FUNCTIONS
// Each task runs independently with its own priority and execution frequency
//==============================================================================

//------------------------------------------------------------------------------
// IMU TASK: Read MPU9250 Sensor & Calculate 3D Orientation
// 
// Purpose: Continuously reads accelerometer and gyroscope data from MPU9250,
//          applies Mahony filter algorithm to calculate stable pitch/roll/yaw,
//          and computes vertical g-force for jump/drop detection.
//
// Frequency: ~50Hz (20ms cycle time)
// Priority: 5 (Highest - time-critical sensor fusion)
// Core: 1
//------------------------------------------------------------------------------
void imuTask(void *pvParameters) {
    Serial.println("imuTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Run ~50Hz

    // === MAHONY FILTER STATE VARIABLES ===
    // Mahony filter fuses accelerometer and gyroscope data for stable orientation
    static float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;      // Quaternion components (w,x,y,z)
    static float twoKp = 2.0f * 0.5f;                    // Proportional gain (2 * Kp)
    static float twoKi = 2.0f * 0.0f;                    // Integral gain (2 * Ki) - set to 0 for now
    static unsigned long lastQuatTime = 0;               // Timestamp for delta-time calculation
    static float integralFBx = 0, integralFBy = 0, integralFBz = 0;  // Integral error terms

    // === MPU9250 INITIALIZATION ===
    // Configure the IMU for optimal performance
    Serial.println("Initializing MPU9250...");
    
    // Wake up the MPU9250
    Wire.beginTransmission(MPU9250_ADDRESS); 
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Clear sleep bit
    Wire.endTransmission(true);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for MPU to stabilize

    // Configure accelerometer range: ±2g (most sensitive)
    Wire.beginTransmission(MPU9250_ADDRESS); 
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x00);  // AFS_SEL=0 -> ±2g range (16384 LSB/g)
    Wire.endTransmission(true);

    // Configure gyroscope range: ±250°/s (good balance of range vs. precision)
    Wire.beginTransmission(MPU9250_ADDRESS); 
    Wire.write(0x1B);  // GYRO_CONFIG register  
    Wire.write(0x00);  // FS_SEL=0 -> ±250°/s range (131 LSB/°/s)
    Wire.endTransmission(true);

    // === DIGITAL LOW PASS FILTER (DLPF) CONFIGURATION ===
    // Reduces noise and vibration in sensor readings
    
    // Gyroscope DLPF: Setting 1 = 184Hz bandwidth, 2.9ms delay
    uint8_t gyroDLPFSetting = 1;
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x1A); // CONFIG register
    Wire.write(gyroDLPFSetting);
    Wire.endTransmission(true);
    Serial.print("Set Gyro DLPF to: "); Serial.println(gyroDLPFSetting);

    // Accelerometer DLPF: Setting 1 = 184Hz bandwidth, 5.8ms delay  
    uint8_t accelDLPFSetting = 1;
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x1D); // ACCEL_CONFIG_2 register
    Wire.write(accelDLPFSetting);
    Wire.endTransmission(true);
    Serial.print("Set Accel DLPF to: "); Serial.println(accelDLPFSetting);

    // Initialize timing for filter algorithm
    lastQuatTime = micros();
    xLastWakeTime = xTaskGetTickCount();

    // === MAIN TASK LOOP ===
    while (1) {
        // --- READ RAW SENSOR DATA ---
        int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        
        // Read accelerometer data (6 bytes starting from ACCEL_XOUT_H)
        Wire.beginTransmission(MPU9250_ADDRESS); 
        Wire.write(ACCEL_XOUT_H); 
        Wire.endTransmission(false);
        Wire.requestFrom(MPU9250_ADDRESS, 6, true);
        raw_ax = Wire.read() << 8 | Wire.read(); 
        raw_ay = Wire.read() << 8 | Wire.read(); 
        raw_az = Wire.read() << 8 | Wire.read();
        
        // Read gyroscope data (6 bytes starting from GYRO_XOUT_H)
        Wire.beginTransmission(MPU9250_ADDRESS); 
        Wire.write(GYRO_XOUT_H); 
        Wire.endTransmission(false);
        Wire.requestFrom(MPU9250_ADDRESS, 6, true);
        raw_gx = Wire.read() << 8 | Wire.read(); 
        raw_gy = Wire.read() << 8 | Wire.read(); 
        raw_gz = Wire.read() << 8 | Wire.read();

        // --- CONVERT RAW DATA TO PHYSICAL UNITS ---
        // Convert to g's (±2g range, 16384 LSB/g)
        float local_ax = raw_ax / 16384.0f;
        float local_ay = raw_ay / 16384.0f;
        float local_az = raw_az / 16384.0f;
        
        // Convert to degrees/second (±250°/s range, 131 LSB/°/s)
        float local_gx = raw_gx / 131.0f;
        float local_gy = raw_gy / 131.0f;
        float local_gz = raw_gz / 131.0f;

        // --- CALCULATE ORIENTATION USING MAHONY FILTER ---
        // The Mahony filter combines gyroscope and accelerometer data to estimate orientation
        unsigned long now = micros();
        float dt = (now - lastQuatTime) * 1e-6f; // Delta time in seconds since last update
        lastQuatTime = now;
        if (dt <= 0) dt = 1e-3; // Prevent division by zero or negative dt

        // Mahony filter algorithm implementation
        // Convert gyroscope from deg/s to rad/s for calculations
        float ax_calc = local_ax, ay_calc = local_ay, az_calc = local_az;
        float gx_calc = local_gx * PI/180.0f, gy_calc = local_gy * PI/180.0f, gz_calc = local_gz * PI/180.0f;

        // Normalize accelerometer measurement (to use as gravity reference)
        float norm = sqrt(ax_calc*ax_calc + ay_calc*ay_calc + az_calc*az_calc);
        if (norm > 0.0f) {
           ax_calc /= norm; ay_calc /= norm; az_calc /= norm;
        } else {
             ax_calc = 0; ay_calc = 0; az_calc = 0;  // Prevent division by zero
        }

        // Estimated direction of gravity from current quaternion
        float vx = 2.0f*(q1*q3 - q0*q2); 
        float vy = 2.0f*(q0*q1 + q2*q3); 
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        
        // Error between measured and estimated gravity direction (cross product)
        float ex = (ay_calc*vz - az_calc*vy); 
        float ey = (az_calc*vx - ax_calc*vz); 
        float ez = (ax_calc*vy - ay_calc*vx);

        // Apply integral feedback if enabled (Ki > 0)
        if (twoKi > 0.0f) {
           integralFBx += twoKi * ex * dt; 
           integralFBy += twoKi * ey * dt; 
           integralFBz += twoKi * ez * dt;
           gx_calc += integralFBx; 
           gy_calc += integralFBy; 
           gz_calc += integralFBz;
        }
        
        // Apply proportional feedback (corrects gyroscope bias)
        gx_calc += twoKp * ex; 
        gy_calc += twoKp * ey; 
        gz_calc += twoKp * ez;

        // Integrate quaternion rate of change (quaternion derivative)
        float qDot0 = 0.5f * (-q1*gx_calc - q2*gy_calc - q3*gz_calc);
        float qDot1 = 0.5f * ( q0*gx_calc + q2*gz_calc - q3*gy_calc);
        float qDot2 = 0.5f * ( q0*gy_calc - q1*gz_calc + q3*gx_calc);
        float qDot3 = 0.5f * ( q0*gz_calc + q1*gy_calc - q2*gx_calc);

        // Update quaternion components
        q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
        
        // Normalize quaternion to maintain unit length
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

        // --- CONVERT QUATERNION TO EULER ANGLES ---
        // Calculate pitch, roll, and yaw from normalized quaternion
        float local_pitch = atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f/PI;
        float local_roll  = asin (2.0f*(q0*q2 - q3*q1)) * 180.0f/PI;
        float local_yaw   = atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f/PI;
        if (local_yaw < 0) local_yaw += 360.0f;  // Ensure yaw is 0-360°

        // --- CALCULATE VERTICAL G-FORCE ---
        // Project acceleration onto gravity vector to get vertical component
        float gravityVectorX = 2.0f * (q1 * q3 - q0 * q2);
        float gravityVectorY = 2.0f * (q0 * q1 + q2 * q3);
        float gravityVectorZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        float verticalAcceleration = local_ax * gravityVectorX + local_ay * gravityVectorY + local_az * gravityVectorZ;
        float local_gForce = verticalAcceleration;  // This is the g-force for jump/drop detection

        // --- UPDATE SHARED VARIABLES (Thread-Safe) ---
        // Copy calculated values to global variables protected by mutex
        if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
            pitch = local_pitch;
            roll = local_roll;
            yaw = local_yaw;
            gForce = local_gForce;
            accelX = local_ax;
            accelY = local_ay;
            accelZ = local_az;
            gyroX = local_gx;
            gyroY = local_gy;
            gyroZ = local_gz;
            xSemaphoreGive(imuDataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//------------------------------------------------------------------------------
// HALL SENSOR TASK: Speed Measurement & Direction Detection
//
// Purpose: Monitors two hall effect sensors mounted on the bike wheel to:
//          - Calculate speed based on time between sensor triggers
//          - Determine direction based on which sensor triggers first
//          - Handle timeouts to reset speed to zero when stopped
//
// Frequency: ~200Hz (5ms cycle time) 
// Priority: 4 (High - time-sensitive for accurate speed measurement)
// Core: 1
//
// Hardware Setup: Two hall sensors positioned 90° apart on wheel, 
//                triggered by magnets on spokes
//------------------------------------------------------------------------------
void hallSensorTask(void *pvParameters) {
    Serial.println("hallSensorTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // Run frequently ~200Hz

    // === INTERNAL STATE VARIABLES ===
    // These track sensor state changes and timing for speed/direction calculation
    static int lastHallSensorValue = HIGH;      // Previous state of primary hall sensor
    static int lastHallSensorValue2 = HIGH;     // Previous state of secondary hall sensor
    static unsigned long hall1TriggerTime = 0;  // Timestamp when hall sensor 1 triggered
    static unsigned long hall2TriggerTime = 0;  // Timestamp when hall sensor 2 triggered
    static bool hall1Triggered = false;         // Flag: hall sensor 1 has triggered in current sequence
    static bool hall2Triggered = false;         // Flag: hall sensor 2 has triggered in current sequence
    static unsigned long lastTriggerTime = 0;   // Last trigger time for speed calculation

    lastTriggerTime = millis();
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        unsigned long currentMillis = millis();

        // --- Read Sensors ---
        int currentHallValue = digitalRead(HALL_SENSOR_PIN); // Uses define
        int currentHallValue2 = digitalRead(HALL_SENSOR_PIN_2); // Uses define

        // --- Local calculation variables ---
        float local_speed = 0.0;
        bool local_direction = true;
        bool speed_updated = false;
        bool direction_updated = false;

        // --- Speed Calculation (Hall 1 Trigger) ---
        if (currentHallValue == LOW && lastHallSensorValue == HIGH) {
            unsigned long currentTrigTime = currentMillis;
            hall1TriggerTime = currentTrigTime;
            hall1Triggered = true;

            if (lastTriggerTime > 0) {
                unsigned long timeDiff = currentTrigTime - lastTriggerTime;
                // Check timeDiff is reasonable, prevent division by zero or stale data
                if (timeDiff > 0 && timeDiff < SPEED_TIMEOUT) { // Uses define
                   // Use HALF_CIRCUMFERENCE_CM define
                   local_speed = (HALF_CIRCUMFERENCE_CM / (float)timeDiff) * 36.0f;
                   speed_updated = true;
                } else if (timeDiff >= SPEED_TIMEOUT) {
                    // If time difference is huge, treat as stopped before this trigger
                    local_speed = 0.0f;
                    speed_updated = true;
                }
            } else {
                 // First trigger since boot or last timeout, calculate speed based on assumption?
                 // Or better to wait for the *next* trigger? Let's wait.
            }
             lastTriggerTime = currentTrigTime;
        }

        // --- Direction Detection (Hall 2 Trigger) ---
         if (currentHallValue2 == LOW && lastHallSensorValue2 == HIGH) {
            hall2TriggerTime = currentMillis;
            hall2Triggered = true;
        }

        // --- Determine Direction ---
        if (hall1Triggered && hall2Triggered) {
             local_direction = (hall1TriggerTime <= hall2TriggerTime);
             direction_updated = true;
            hall1Triggered = false;
            hall2Triggered = false;
        }

        // --- Timeout Resets ---
        // Reset direction triggers if timeout occurs
        if ((hall1Triggered || hall2Triggered) &&
            (currentMillis - max(hall1TriggerTime, hall2TriggerTime) > directionDetectionTimeout)) { // Uses define
            hall1Triggered = false;
            hall2Triggered = false;
        }

        // Check speed timeout
        if (currentMillis - lastTriggerTime > SPEED_TIMEOUT) { // Uses define
             // Only update if speed needs changing to 0
             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
                if (currentSpeed != 0.0f) {
                   local_speed = 0.0f; // Prepare to update speed to 0
                   speed_updated = true;
                }
                 xSemaphoreGive(hallDataMutex);
             }
             // Reset lastTriggerTime to prevent continuous zeroing?
             // Or set it to current time so next trigger calculates speed? Let's set to current time.
             lastTriggerTime = currentMillis;
        }

        // --- Update Shared Variables (Protected by Mutex) ---
        if (speed_updated || direction_updated) { // Only take mutex if there's something to update
             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
                 if(speed_updated) {
                    currentSpeed = local_speed;
                 }
                 if(direction_updated) {
                    hallDirectionForward = local_direction;
                 }
                 hallSensorValue = currentHallValue; // Update raw values too
                 hallSensorValue2 = currentHallValue2;
                 xSemaphoreGive(hallDataMutex);
             }
        } else {
             // Still update raw values even if speed/dir didn't change? Maybe.
             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
                 hallSensorValue = currentHallValue;
                 hallSensorValue2 = currentHallValue2;
                 xSemaphoreGive(hallDataMutex);
             }
        }

        // Update last values for next iteration
        lastHallSensorValue = currentHallValue;
        lastHallSensorValue2 = currentHallValue2;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//------------------------------------------------------------------------------
// POTENTIOMETER TUNING TASK: Real-time Threshold Adjustment
//
// Purpose: Continuously reads three potentiometers to allow real-time tuning of:
//          - Jump detection threshold (g-force level for takeoff detection)
//          - Landing detection threshold (g-force level for landing detection)  
//          - Drop detection threshold (g-force level for impact detection)
//
// Frequency: ~10Hz (100ms cycle time)
// Priority: 2 (Low - non-critical background tuning)
// Core: 1
//------------------------------------------------------------------------------
void potTuningTask(void *pvParameters) {
    Serial.println("potTuningTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Run ~10Hz

    // ADC configuration is now done in setup()

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // --- Read Analog Values using Arduino function ---
        // analogRead() uses the width and attenuation set in setup()
        int jumpPotRaw = analogRead(JUMP_THRESH_POT_PIN);
        int landPotRaw = analogRead(LAND_THRESH_POT_PIN);
        int dropPotRaw = analogRead(DROP_THRESH_POT_PIN);

        // --- Map Raw Values (0-4095) to Threshold Ranges ---
        // Ensure JUMP_THRESH_MIN, _MAX, etc. are defined globally
        float local_jumpThreshold = JUMP_THRESH_MIN + ((float)jumpPotRaw / 4095.0f) * (JUMP_THRESH_MAX - JUMP_THRESH_MIN);
        float local_landingThreshold = LAND_THRESH_MIN + ((float)landPotRaw / 4095.0f) * (LAND_THRESH_MAX - LAND_THRESH_MIN);
        float local_dropThreshold = DROP_THRESH_MIN + ((float)dropPotRaw / 4095.0f) * (DROP_THRESH_MAX - DROP_THRESH_MIN);

        // Add bounds checking just in case pots give outlier values or mapping is imperfect
        local_jumpThreshold = constrain(local_jumpThreshold, JUMP_THRESH_MIN, JUMP_THRESH_MAX);
        local_landingThreshold = constrain(local_landingThreshold, LAND_THRESH_MIN, LAND_THRESH_MAX);
        local_dropThreshold = constrain(local_dropThreshold, DROP_THRESH_MIN, DROP_THRESH_MAX);


        // --- Update Shared Config Variables (Protected by Mutex) ---
        // Assumes configMutex is created in setup() and jumpThreshold etc. are global
        if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Use timeout
            jumpThreshold = local_jumpThreshold;
            landingThreshold = local_landingThreshold;
            dropThreshold = local_dropThreshold;
            xSemaphoreGive(configMutex);
        } else {
            Serial.println("Warning: potTuningTask failed to get configMutex!");
        }

        // Delay until the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//------------------------------------------------------------------------------
// PROCESSING TASK: Event Detection & User Input Handling  
//
// Purpose: Core logic task that:
//          - Detects jump/drop/180° spin events using IMU data and thresholds
//          - Handles zero button presses for IMU calibration
//          - Calculates IMU-based direction and speed state
//          - Manages event timing and display duration
//
// Frequency: ~33Hz (30ms cycle time)
// Priority: 3 (Medium-High - main application logic)
// Core: 1
//------------------------------------------------------------------------------
void processingTask(void *pvParameters) {
    Serial.println("processingTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(30); // Run ~33Hz

    //Static internal vars for state
    static bool inJumpState = false;
    static unsigned long jumpStartTime = 0;
    static unsigned long lastJumpTime = 0;
    static unsigned long lastDropTime = 0;
    static bool lastButtonState = HIGH;
    static unsigned long lastDebounceTime = 0;

    // --- 180 Detection State ---
    static unsigned long lastGForceAnomalyTime = 0;
    static float spinScore = 0.0f;
    static float total_rotation = 0.0f;
    static unsigned long lastOneEightyTime = 0;
    static unsigned long lastProcessTime = 0;


    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        unsigned long currentMillis = millis();
        // Calculate a dynamic delta-time in seconds for more accurate integration
        float dt_sec = (lastProcessTime > 0) ? (currentMillis - lastProcessTime) / 1000.0f : (30.0f / 1000.0f);
        lastProcessTime = currentMillis;

        // --- Local copies of data needed ---
        float local_accelX=0.0, local_accelY=0.0, local_accelZ=0.0, local_gForce=1.0, local_gyroZ = 0.0;
        float local_pitch=0.0;
        // ** NEW: Local copies of thresholds for this cycle **
        float local_jumpThreshold = 0.5; // Default if mutex fails
        float local_landingThreshold = 2.0;
        float local_dropThreshold = 2.5;


        // --- Get IMU data ---
        if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
            local_accelX = accelX; local_accelY = accelY; local_accelZ = accelZ;
            local_gForce = gForce;
            local_pitch = pitch;
            local_gyroZ = gyroZ;
            xSemaphoreGive(imuDataMutex);
        } else {
            Serial.println("Warning: processingTask failed to get imuDataMutex!");
        }

        // --- Get Current Thresholds --- 
        if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
             local_jumpThreshold = jumpThreshold;
             local_landingThreshold = landingThreshold;
             local_dropThreshold = dropThreshold;
             xSemaphoreGive(configMutex);
         } else {
             Serial.println("Warning: processingTask failed to get configMutex!");
             // Keep default local values if mutex fails
         }


        // --- Local copies of event/state variables ---
        bool current_jump_state = false;
        bool current_drop_state = false;
        bool current_oneEighty_state = false;
        bool current_imuDir = true;
        int current_imuState = 0;

        if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
            current_jump_state = jumpDetected;
            current_drop_state = dropDetected;
            current_oneEighty_state = oneEightyDetected;
            current_imuDir = imuDirectionForward;
            current_imuState = imuSpeedState;
            xSemaphoreGive(eventDataMutex);
        } else {
             Serial.println("Warning: processingTask failed to get eventDataMutex for reading state!");
        }

        // --- Local flags/variables for this cycle's updates ---
        bool local_jumpDetected_this_cycle = current_jump_state;
        bool local_dropDetected_this_cycle = current_drop_state;
        bool local_oneEightyDetected_this_cycle = current_oneEighty_state;
        bool local_imuDirectionForward = current_imuDir;
        int local_imuSpeedState = current_imuState;


        // --- Jump & Drop Detection
        if (!inJumpState && local_gForce < local_jumpThreshold) { 
             inJumpState = true; jumpStartTime = currentMillis;
        }
        if (inJumpState && local_gForce > local_landingThreshold) {
             unsigned long jumpDuration = currentMillis - jumpStartTime;
             if (jumpDuration > JUMP_DURATION_MIN) {
                 local_jumpDetected_this_cycle = true; lastJumpTime = currentMillis;
                 if(!current_jump_state) { Serial.println("JUMP DETECTED! Duration: " + String(jumpDuration) + "ms G: " + String(local_gForce, 2)); }
             }
             inJumpState = false;
        }
        if(inJumpState && (currentMillis - jumpStartTime > 5000)) { // Timeout
             inJumpState = false; Serial.println("Jump state timeout");
        }
        // Drop Detection
        if (!inJumpState && local_gForce > local_dropThreshold && !current_drop_state) {
            local_dropDetected_this_cycle = true; lastDropTime = currentMillis;
            Serial.println("DROP DETECTED! Impact G-force: " + String(local_gForce, 2));
        }
        // Event clearing logic (remains the same)
        if (current_jump_state && (currentMillis - lastJumpTime > EVENT_DISPLAY_DURATION)) {
            local_jumpDetected_this_cycle = false;
        }
        if (current_drop_state && (currentMillis - lastDropTime > EVENT_DISPLAY_DURATION)) {
            local_dropDetected_this_cycle = false;
        }


        // --- 180 Detection Logic ---
        // 1. Check for recent G-Force anomaly
        if (local_gForce > GFORCE_ANOMALY_UPPER || local_gForce < GFORCE_ANOMALY_LOWER) {
            lastGForceAnomalyTime = currentMillis;
        }
        bool recentGForceAnomaly = (currentMillis - lastGForceAnomalyTime) < GFORCE_ANOMALY_WINDOW_MS;

        // 2. Calculate spin score and total rotation
        if (fabs(local_gyroZ) > YAW_RATE_THRESHOLD_MIN) {
            // We are in a spin, accumulate score and rotation
            spinScore += (fabs(local_gyroZ) / YAW_RATE_THRESHOLD_MIN) * dt_sec;
            total_rotation += local_gyroZ * dt_sec;
        } else {
            // Not spinning fast, decay score and rotation
            spinScore -= SPIN_SCORE_DECAY_PER_SEC * dt_sec;
            total_rotation *= ROTATION_DECAY_FACTOR;
        }
        
        if (spinScore < 0) {
            spinScore = 0;
        }
        // If spin score has fully decayed, reset rotation as well. This signifies the end of a potential maneuver.
        if (spinScore == 0) {
            total_rotation = 0;
        }

        // 3. Check for 180 event trigger
        if (!current_oneEighty_state && spinScore >= SPIN_SCORE_THRESHOLD && recentGForceAnomaly && fabs(total_rotation) >= TOTAL_ROTATION_THRESHOLD) {
            local_oneEightyDetected_this_cycle = true;
            lastOneEightyTime = currentMillis;
            spinScore = 0; // Reset score after detection
            total_rotation = 0; // Reset rotation after detection
            Serial.println(">>> 180 DETECTED! G-Anom, Spin, and Rotation Met");
        }

        // Event clearing logic
        if (current_oneEighty_state && (currentMillis - lastOneEightyTime > EVENT_DISPLAY_DURATION)) {
            local_oneEightyDetected_this_cycle = false;
        }


        // --- IMU-Based Direction & Speed State ---
        float gravityCompAccelY = local_accelY - sin(local_pitch * PI / 180.0f);
         if (current_imuDir == true) {
             if (gravityCompAccelY > IMU_DIRECTION_ACCEL_THRESHOLD) {
                 local_imuDirectionForward = false;
             }
         } else {
             if (gravityCompAccelY < -IMU_DIRECTION_ACCEL_THRESHOLD) {
                 local_imuDirectionForward = true;
             }
         }
         float accelMagnitudeXY = sqrt(local_accelX * local_accelX + gravityCompAccelY * gravityCompAccelY);
         if (accelMagnitudeXY > 0.8f) local_imuSpeedState = 2;
         else if (accelMagnitudeXY > 0.2f) local_imuSpeedState = 1;
         else local_imuSpeedState = 0;


        // --- Zero Button Handling ---
        int reading = digitalRead(ZERO_BUTTON_PIN);
        bool button_pressed = false;
        if (reading == LOW && lastButtonState == HIGH) {
             if ((currentMillis - lastDebounceTime) > debounceDelay) {
                 button_pressed = true;
                 lastDebounceTime = currentMillis;
                 Serial.println(">>> Zero Button: Debounced Press Detected!");
             }
         }
         lastButtonState = reading;

        // --- Update Shared Variables ---
        // Update Offsets if button pressed
        if (button_pressed) {
            float current_raw_pitch=0.0, current_raw_roll=0.0, current_raw_yaw=0.0;
            if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
                 current_raw_pitch = pitch; 
                 current_raw_roll = roll; 
                 current_raw_yaw = yaw;
                 xSemaphoreGive(imuDataMutex);
                 if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) {
                     pitchOffset = current_raw_pitch;
                     rollOffset = current_raw_roll; 
                     yawOffset = current_raw_yaw;
                     xSemaphoreGive(offsetMutex);
                     Serial.println(">>> Zero Button: SUCCESS - Zero position offsets updated!");
                 } else { Serial.println(">>> Zero Button: FAILED to get offsetMutex!"); }
             } else { Serial.println(">>> Zero Button: FAILED to get imuDataMutex!"); }

        } // End of button_pressed handling

        // Update Event Data (State data)
        if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
            jumpDetected = local_jumpDetected_this_cycle;
            dropDetected = local_dropDetected_this_cycle;
            oneEightyDetected = local_oneEightyDetected_this_cycle;
            imuDirectionForward = local_imuDirectionForward;
            imuSpeedState = local_imuSpeedState;
            xSemaphoreGive(eventDataMutex);
        } else {
            Serial.println("Warning: processingTask failed to get eventDataMutex for updating state!");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//------------------------------------------------------------------------------
// BLE COMMUNICATION TASK: Wireless Data Transmission
//
// Purpose: Manages Bluetooth Low Energy communication by:
//          - Sending sensor data (speed, orientation, g-force) to connected devices
//          - Transmitting event notifications (jump/drop/180° detections)
//          - Applying calibration offsets to orientation data
//          - Managing connection state and notifications
//
// Frequency: ~10Hz (100ms cycle time)
// Priority: 2 (Low-Medium - communication can tolerate some latency)
// Core: 1
//------------------------------------------------------------------------------
void bleTask(void *pvParameters) {
    Serial.println("bleTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Notify ~10Hz

    // Static internal vars for state
    static bool prevJumpDetected = false;
    static bool prevDropDetected = false;
    static bool prevOneEightyDetected = false;

    xLastWakeTime = xTaskGetTickCount();


    while (1) {
        bool isConnected = false;
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
             isConnected = deviceConnected;
             xSemaphoreGive(bleConnectionMutex);
          }

        if (isConnected) {
            // --- Read required data ---
            float local_speed=0.0, local_pitch=0.0, local_roll=0.0, local_yaw=0.0, local_gForce=1.0;
            float local_pitchOffset=0.0, local_rollOffset=0.0, local_yawOffset=0.0;
            bool local_jump=false, local_drop=false, local_180=false, local_imuDir=true, local_hallDir=true;
            int local_imuState=0;
            // Note: Thresholds are not currently sent over BLE, but could be added if needed

             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeed; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { local_pitch = pitch; local_roll = roll; local_yaw = yaw; local_gForce = gForce; xSemaphoreGive(imuDataMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { local_pitchOffset = pitchOffset; local_rollOffset = rollOffset; local_yawOffset = yawOffset; xSemaphoreGive(offsetMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; local_180 = oneEightyDetected; local_imuDir = imuDirectionForward; local_imuState = imuSpeedState; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }

            // --- Calculate Zeroed Values ---
            float zeroedPitch = local_pitch - local_pitchOffset;
            float zeroedRoll = local_roll - local_rollOffset;
            float zeroedYaw = local_yaw - local_yawOffset;
            while (zeroedYaw < 0) zeroedYaw += 360;
            while (zeroedYaw >= 360) zeroedYaw -= 360;

            // --- Send Notifications ---
            // ... (Notifications for Speed, Pitch, Roll, Yaw, GForce remain the same) ...
            if(pSpeedCharacteristic) { pSpeedCharacteristic->setValue((uint8_t*)&local_speed, sizeof(local_speed)); pSpeedCharacteristic->notify(); }
            if(pPitchCharacteristic) { pPitchCharacteristic->setValue((uint8_t*)&zeroedPitch, sizeof(zeroedPitch)); pPitchCharacteristic->notify(); }
            if(pRollCharacteristic) { pRollCharacteristic->setValue((uint8_t*)&zeroedRoll, sizeof(zeroedRoll)); pRollCharacteristic->notify(); }
            if(pYawCharacteristic) { pYawCharacteristic->setValue((uint8_t*)&zeroedYaw, sizeof(zeroedYaw)); pYawCharacteristic->notify(); }
            if(pGForceCharacteristic) { pGForceCharacteristic->setValue((uint8_t*)&local_gForce, sizeof(local_gForce)); pGForceCharacteristic->notify(); }


            // Event Notification (Only on change)
            uint8_t eventCode = 0;
            bool notifyEvent = false;
            if (local_jump && !prevJumpDetected) { eventCode = 1; notifyEvent = true; Serial.println("BLE Notify: JUMP (1)"); }
            else if (local_drop && !prevDropDetected) { eventCode = 2; notifyEvent = true; Serial.println("BLE Notify: DROP (2)"); }
            else if (local_180 && !prevOneEightyDetected) { eventCode = 3; notifyEvent = true; Serial.println("BLE Notify: 180 (3)"); }

            if (notifyEvent && pEventCharacteristic) {
                pEventCharacteristic->setValue(&eventCode, sizeof(eventCode));
                pEventCharacteristic->notify();
            }
            prevJumpDetected = local_jump; // Use the static variable
            prevDropDetected = local_drop; // Use the static variable
            prevOneEightyDetected = local_180;

            // Direction & State Notifications
            // ... (Remain the same) ...
            if(pImuDirectionCharacteristic) { uint8_t imuDirCode = local_imuDir ? 1 : 0; pImuDirectionCharacteristic->setValue(&imuDirCode, sizeof(imuDirCode)); pImuDirectionCharacteristic->notify(); }
            if(pHallDirectionCharacteristic){ uint8_t hallDirCode = local_hallDir ? 1 : 0; pHallDirectionCharacteristic->setValue(&hallDirCode, sizeof(hallDirCode)); pHallDirectionCharacteristic->notify(); }
            if(pImuSpeedStateCharacteristic){ uint8_t speedStateCode = (uint8_t)local_imuState; pImuSpeedStateCharacteristic->setValue(&speedStateCode, sizeof(speedStateCode)); pImuSpeedStateCharacteristic->notify(); }

        } else {
             prevJumpDetected = false; // Reset state if disconnected
             prevDropDetected = false;
             prevOneEightyDetected = false;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


//------------------------------------------------------------------------------
// DISPLAY TASK: OLED Screen Updates
//
// Purpose: Updates the 128x64 OLED display with real-time information:
//          - Current sensor readings (g-force, angle, speed, direction)
//          - Event status indicators (jump, drop, 180° spin)
//          - Threshold values from potentiometers
//          - BLE connection status
//
// Frequency: ~6-7Hz (150ms cycle time)
// Priority: 1 (Lowest - display updates are not time-critical)
// Core: 1
//------------------------------------------------------------------------------
void displayTask(void *pvParameters) {
    Serial.println("displayTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(150); // Update display ~6-7Hz

    // Initial display message
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.println("Music Bike RTOS"); display.println("Initializing...");
    display.display();
    vTaskDelay(pdMS_TO_TICKS(1000));

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
         // --- Read required data ---
         float local_speed=0.0, local_pitch=0.0, local_gForce=1.0; // Removed roll, yaw
         float local_pitchOffset=0.0; // Removed rollOffset, yawOffset
         bool local_jump=false, local_drop=false, local_180=false, local_hallDir=true, isConnected=false;
         // ** NEW: Read thresholds **
         float local_jumpThreshold = 0.0;
         float local_landingThreshold = 0.0;
         float local_dropThreshold = 0.0;

         // Read state variables using mutexes
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) { isConnected = deviceConnected; xSemaphoreGive(bleConnectionMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeed; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { local_pitch = pitch; local_gForce = gForce; /*Removed roll, yaw reads*/ xSemaphoreGive(imuDataMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { local_pitchOffset = pitchOffset; /*Removed roll, yaw offset reads*/ xSemaphoreGive(offsetMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; local_180 = oneEightyDetected; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }
         // ** NEW: Read thresholds **
         if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
              local_jumpThreshold = jumpThreshold;
              local_landingThreshold = landingThreshold;
              local_dropThreshold = dropThreshold;
              xSemaphoreGive(configMutex);
          } else { vTaskDelay(1); continue; }


         // --- Calculate Zeroed Pitch Value ---
         float zeroedPitch = local_pitch - local_pitchOffset;
         // Yaw/Roll calculations removed

         // --- Update Display ---
         display.clearDisplay();
         display.setTextSize(1);
         display.setTextColor(SSD1306_WHITE);
         int yPos = 0; // Current Y position for cursor

         // Line 0: G-Force and BLE Status
         display.setCursor(0, yPos); display.print("G:"); display.print(local_gForce, 2);
         if (isConnected) { display.setCursor(SCREEN_WIDTH - 18, yPos); display.print("BLE"); }
         yPos += 10;

          // Line 1: Jump/Drop Status
         display.setCursor(0, yPos); display.print("J:"); display.print(local_jump ? "Y" : "N");
         display.setCursor(32, yPos); display.print(" D:"); display.print(local_drop ? "Y" : "N");
         display.setCursor(75, yPos); display.print(" 180:"); display.print(local_180 ? "Y" : "N");
         yPos += 10;

         // Line 2: Pitch and Jump Threshold
         display.setCursor(0, yPos); display.print("Angle:"); display.print(zeroedPitch, 1); // pitch, called angle on sensor
         display.setCursor(64, yPos); display.print("Jump:"); display.print(local_jumpThreshold, 2); // Jump Thresh
         yPos += 10;

         // Line 3: Landing and Drop Thresholds
         display.setCursor(0, yPos); display.print("Land:"); display.print(local_landingThreshold, 2); // Landing Thresh
         display.setCursor(64, yPos); display.print("Drop:"); display.print(local_dropThreshold, 2); // Drop Thresh
         yPos += 10;

         // Line 4: Speed and Direction
         display.setCursor(0, yPos); display.print("Spd:"); display.print(local_speed, 1);
         display.setCursor(64, yPos); display.print("Dir:"); display.print(local_hallDir ? "F" : "R");
         // yPos += 10; // No more lines needed currently

         display.display();

         vTaskDelayUntil(&xLastWakeTime, xFrequency);
     }
 }


//==============================================================================
// SETUP FUNCTION: System Initialization
// 
// Purpose: One-time initialization of all system components:
//          - Hardware pins (sensors, buttons, LEDs)
//          - I2C communication and OLED display
//          - BLE server and characteristics
//          - FreeRTOS mutexes and tasks
//          - ADC configuration for potentiometers
//==============================================================================
void setup() {
    Serial.begin(115200);
    //while (!Serial); // Waits for USB Serial connection; comment out for standalone/battery operation.
    Serial.println("Music Bike Sensor System Initializing");

    // --- Initialize Hardware Pins ---
    pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(HALL_SENSOR_PIN_2, INPUT);
    pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BLE_LED_PIN, OUTPUT);
    digitalWrite(BLE_LED_PIN, LOW);
    //Analog inputs (channel 1 "ACD1" )
    pinMode(JUMP_THRESH_POT_PIN, INPUT);
    pinMode(LAND_THRESH_POT_PIN, INPUT);
    pinMode(DROP_THRESH_POT_PIN, INPUT);
    //analogSetWidth(12); // 12-bit resolution (0-4095) // causes compile error, default setting 12 so unneeded  
    // Set attenuation per pin using the defined GPIOs
    analogSetPinAttenuation(JUMP_THRESH_POT_PIN, ADC_11db); // Approx 0-3.6V range
    analogSetPinAttenuation(LAND_THRESH_POT_PIN, ADC_11db);
    analogSetPinAttenuation(DROP_THRESH_POT_PIN, ADC_11db);

    // --- Initialize I2C ---
    Wire.begin(SDA_PIN, SCL_PIN);

    // --- Initialize OLED Display ---
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed")); for(;;);
    }
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.println("Starting RTOS..."); display.display();
    delay(500);

    // --- Create Mutexes ---
    imuDataMutex = xSemaphoreCreateMutex();
    hallDataMutex = xSemaphoreCreateMutex();
    eventDataMutex = xSemaphoreCreateMutex();
    offsetMutex = xSemaphoreCreateMutex();
    bleConnectionMutex = xSemaphoreCreateMutex();
    configMutex = xSemaphoreCreateMutex();

    if (!imuDataMutex || !hallDataMutex || !eventDataMutex || !offsetMutex || !bleConnectionMutex || !configMutex ) {
        Serial.println("Failed to create mutexes!"); for(;;);
    }

    // --- BLE Initialization ---
    Serial.println("Initializing BLE...");
    BLEDevice::init("MusicBike_RTOS");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 32); // Increased GATT table size slightly

    pSpeedCharacteristic = pService->createCharacteristic(SPEED_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pSpeedCharacteristic->addDescriptor(new BLE2902());
    pPitchCharacteristic = pService->createCharacteristic(PITCH_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pPitchCharacteristic->addDescriptor(new BLE2902());
    pRollCharacteristic = pService->createCharacteristic(ROLL_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pRollCharacteristic->addDescriptor(new BLE2902());
    pYawCharacteristic = pService->createCharacteristic(YAW_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pYawCharacteristic->addDescriptor(new BLE2902());
    pEventCharacteristic = pService->createCharacteristic(EVENT_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pEventCharacteristic->addDescriptor(new BLE2902());
    pImuDirectionCharacteristic = pService->createCharacteristic(IMU_DIRECTION_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pImuDirectionCharacteristic->addDescriptor(new BLE2902());
    pHallDirectionCharacteristic = pService->createCharacteristic(HALL_DIRECTION_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pHallDirectionCharacteristic->addDescriptor(new BLE2902());
    pImuSpeedStateCharacteristic = pService->createCharacteristic(IMU_SPEED_STATE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pImuSpeedStateCharacteristic->addDescriptor(new BLE2902());
    pGForceCharacteristic = pService->createCharacteristic(GFORCE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pGForceCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("BLE Initialized and Advertising!");

    display.clearDisplay(); display.setCursor(0,0);
    display.println("BLE Advertising!"); display.println("Creating tasks...");
    display.display();
    delay(500);

    // --- Create Tasks ---
    // Priorities: IMU=5, Hall=4, Processing=3, PotTuning=2, BLE=2, Display=1
    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 5, &imuTaskHandle, 1);
    ret = xTaskCreatePinnedToCore(hallSensorTask, "HallTask", 4096, NULL, 4, &hallSensorTaskHandle, 1);
    ret = xTaskCreatePinnedToCore(processingTask, "ProcessingTask", 4096, NULL, 3, &processingTaskHandle, 1);
    ret = xTaskCreatePinnedToCore(potTuningTask, "PotTuneTask", 2048, NULL, 2, &potTuningTaskHandle, 1);
    ret = xTaskCreatePinnedToCore(bleTask, "BLETask", 4096, NULL, 2, &bleTaskHandle, 1);
    ret = xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 1);

    Serial.println("Tasks created. Initialization complete!");
    display.clearDisplay(); display.setCursor(0,0); display.println("Tasks Running!"); display.display();
}

//==============================================================================
// LOOP FUNCTION: FreeRTOS Compatibility
//
// Purpose: Arduino's main loop function - kept minimal since all functionality
//          is handled by FreeRTOS tasks. Simply delays to yield CPU time to tasks.
//==============================================================================
void loop() {
      vTaskDelay(pdMS_TO_TICKS(1000));  // Yield to FreeRTOS scheduler
}
