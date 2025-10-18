// WARNING: THIS FILE IS DEPRECATED.
// Bike_Sensor_Filtered_IMU.ino is the only up to date code for the embedded platform
// This file is kept for reference only.

// =============================================================================
// HARDWARE ADJUSTMENT NOTES (As of 2025-05-04)
// =============================================================================
//
// 1. Hall Effect Sensor Power (KY-003 Modules on GPIO 15, 16):
//    - Supply voltage for these sensors was changed from 3.3V to 5V.
//    - Reason: Resolved signal instability ('flickering') and phantom speed
//      readings observed when sensors were powered at 3.3V. The sensors
//      provide a stable output when powered at 5V.
//
// 2. Startup Stability Capacitor:
//    - A capacitor was added across the main power input (or near the ESP32).
//    - Reason: To stabilize the supply voltage during high current demands
//      at startup (especially from the radio), improving boot reliability
//      and preventing potential resets when running on battery power.
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/adc.h>

// --- FreeRTOS Includes ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For Mutexes

//==============================================================================
// CONFIGURATION SETTINGS
//==============================================================================

// Pin definitions
#define SDA_PIN 18
#define SCL_PIN 15
#define ZERO_BUTTON_PIN 8
#define HALL_SENSOR_PIN 9
#define HALL_SENSOR_PIN_2 46 
#define BLE_LED_PIN 3
#define JUMP_THRESH_POT_PIN 12 
#define LAND_THRESH_POT_PIN 11
#define DROP_THRESH_POT_PIN 10

// Threshold variables
// Default values, will be overwritten by potentiometers
float jumpThreshold = 0.5;      // gForce check (< threshold for takeoff)
float landingThreshold = 2.0;   // gForce check (> threshold for landing)
float dropThreshold = 2.5;      // gForce check (> threshold spike for drop)

// Rolling average size
#define AVG_SIZE 50
#define THRESHOLD 5.0f
=======

//Define tunable ranges for potentiometers
#define JUMP_THRESH_MIN 0.1f
#define JUMP_THRESH_MAX 1.5f
#define LAND_THRESH_MIN 1.0f
#define LAND_THRESH_MAX 4.0f
#define DROP_THRESH_MIN 1.5f
#define DROP_THRESH_MAX 5.0f

// Other Thresholds Constants
#define JUMP_DURATION_MIN 100
#define DIRECTION_THRESHOLD 0.3 // For IMU-based direction
#define IMU_DIRECTION_ACCEL_THRESHOLD 0.3 // Threshold for IMU-based direction change (+/- this value)

// Physical constants
#define WHEEL_DIAMETER_INCHES 26.0
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_INCHES * 2.54 * 3.14159)
#define HALF_CIRCUMFERENCE_CM (WHEEL_CIRCUMFERENCE_CM / 2.0)

// Timing constants from original code (if specific names were used there)
const unsigned long SPEED_TIMEOUT = 3000;
const unsigned long EVENT_DISPLAY_DURATION = 2000;
const unsigned long debounceDelay = 50;
const unsigned long directionDetectionTimeout = 500; // ms timeout to reset hall direction detection state

// MPU9250 registers
#define MPU9250_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Usually -1 if sharing Arduino reset pin
#define SCREEN_ADDRESS 0x3C
// Note: Adafruit_SSD1306 display object is declared later globally

//==============================================================================
// BLE CONFIGURATION
//==============================================================================

// BLE: Define Service and Characteristic UUIDs
#define SERVICE_UUID                      "0fb899fa-2b3a-4e11-911d-4fa05d130dc1"
#define SPEED_CHARACTERISTIC_UUID         "a635fed5-9a19-4e31-8091-84d020481329"
#define PITCH_CHARACTERISTIC_UUID         "726c4b96-bc56-47d2-95a1-a6c49cce3a1f"
#define ROLL_CHARACTERISTIC_UUID          "a1e929e3-5a2e-4418-806a-c50ab877d126"
#define YAW_CHARACTERISTIC_UUID           "cd6fc0f8-089a-490e-8e36-74af84977c7b"
#define GFORCE_CHARACTERISTIC_UUID        "a6210f30-654f-32ea-9e37-432a639fb38e"
#define EVENT_CHARACTERISTIC_UUID         "26205d71-58d1-45e6-9ad1-1931cd7343c3"
#define IMU_DIRECTION_CHARACTERISTIC_UUID "ceb04cf6-0555-4243-a27b-c85986ab4bd7"
#define HALL_DIRECTION_CHARACTERISTIC_UUID "f231de63-475c-463d-9b3f-f338d7458bb9"
#define IMU_SPEED_STATE_CHARACTERISTIC_UUID "738f5e54-5479-4941-ae13-caf4a9b07b2e"
#define ACCELEROMETER_ZERO_CHARACTERISTIC_UUID "a29ff0d6-5bf9-4878-83f0-9f66a7e35a15"

//==============================================================================
// GLOBAL VARIABLES (Shared Data) & MUTEXES
//==============================================================================

// --- Mutex Handles ---
SemaphoreHandle_t imuDataMutex = NULL;
SemaphoreHandle_t hallDataMutex = NULL;
SemaphoreHandle_t eventDataMutex = NULL;
SemaphoreHandle_t offsetMutex = NULL;
SemaphoreHandle_t bleConnectionMutex = NULL; // Protect deviceConnected flag
SemaphoreHandle_t configMutex = NULL; // Mutex for tunable config variables

// --- IMU Data (Protected by imuDataMutex) ---
// Average over 10 readings
float pitch[AVG_SIZE], roll[AVG_SIZE], yaw[AVG_SIZE];
float pitchAvg = 0.0, rollAvg = 0.0, yawAvg = 0.0;
float gForce[AVG_SIZE]; 
float gForceAvg = 1.0; // Initialize to 1g

float accelX = 0.0, accelY = 0.0, accelZ = 0.0; // Raw scaled accel
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;   // Raw scaled gyro

// --- Orientation Offsets (Protected by offsetMutex) ---
float pitchOffset = 0.0, rollOffset = 0.0, yawOffset = 0.0;

// --- Hall Sensor Data (Protected by hallDataMutex) ---
float currentSpeed[AVG_SIZE];          // km/h
float currentSpeedAvg = 0.0;
bool hallDirectionForward = true;
int hallSensorValue = HIGH;       // Current raw value
int hallSensorValue2 = HIGH;      // Current raw value 2

// --- Processed/Event Data (Protected by eventDataMutex) ---
bool jumpDetected = false;
bool dropDetected = false;
bool imuDirectionForward = true;
int imuSpeedState = 0; // 0=Stop/Slow, 1=Medium, 2=Fast

// --- BLE State (Protected by bleConnectionMutex) ---
volatile bool deviceConnected = false; // Volatile because modified by ISR/Callback context
bool oldDeviceConnected = false;      // Only used within BLE task, maybe make local?

// --- BLE Objects (Global, initialized in setup) ---
BLEServer* pServer = NULL;
BLECharacteristic* pSpeedCharacteristic = NULL;
BLECharacteristic* pPitchCharacteristic = NULL; // Declaration added
BLECharacteristic* pRollCharacteristic = NULL;  // Declaration added
BLECharacteristic* pYawCharacteristic = NULL;   // Declaration added
BLECharacteristic* pGForceCharacteristic = NULL;
BLECharacteristic* pEventCharacteristic = NULL; // Declaration added
BLECharacteristic* pImuDirectionCharacteristic = NULL; // Declaration added
BLECharacteristic* pHallDirectionCharacteristic = NULL;// Declaration added
BLECharacteristic* pImuSpeedStateCharacteristic = NULL; // Declaration added
BLECharacteristic* pAccelerometerZeroCharacteristic = NULL;

// --- Display Object (Global) ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Now uses defines

// --- Task Handles (Optional, for controlling tasks later) ---
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t hallSensorTaskHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t potTuningTaskHandle = NULL;

//==============================================================================
// BLE CALLBACK CLASS
//==============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) {
        if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = true;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Connected");
        digitalWrite(BLE_LED_PIN, HIGH); // Uses define
    }

    void onDisconnect(BLEServer* pServerInstance) {
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = false;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Disconnected");
        digitalWrite(BLE_LED_PIN, LOW); // Uses define
        vTaskDelay(pdMS_TO_TICKS(500));
        pServer->startAdvertising();
        Serial.println("BLE Advertising restarted");
    }
};

// Callback class for accelerometer zero characteristic
class AccelerometerZeroCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            // Any non-zero value will trigger the zero reset
            if (value[0] != 0) {
                Serial.println(">>> BLE Zero Command Received!");
                
                // Get current orientation values
                float current_raw_pitch = 0.0, current_raw_roll = 0.0, current_raw_yaw = 0.0;
                
                if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
                    current_raw_pitch = pitchAvg;
                    current_raw_roll = rollAvg;
                    current_raw_yaw = yawAvg;
                    xSemaphoreGive(imuDataMutex);
                    
                    // Update offsets
                    if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) {
                        pitchOffset = current_raw_pitch;
                        rollOffset = current_raw_roll;
                        yawOffset = current_raw_yaw;
                        xSemaphoreGive(offsetMutex);
                        Serial.println(">>> BLE Zero: SUCCESS - Zero position offsets updated!");
                    }
                }
            }
        }
    }
};

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

// Updates array of rolling average and returns new rolling average value
float updateRollingAverage(float array[], float newVal) {
    float alpha = 2.0f / (AVG_SIZE + 1);
    
    // Calculate current average
    float currentAvg = 0.0f;
    for (int i = 0; i < AVG_SIZE; i++) {
        currentAvg += array[i];
    }
    currentAvg /= AVG_SIZE;

    // Spike rejection: if newVal deviates too much, ignore it
    if (fabs(newVal - currentAvg) > THRESHOLD) {
        newVal = currentAvg;
    }
    float newEMA = alpha * newVal + (1 - alpha) * array[AVG_SIZE - 1];

    // Shift array and update last value
    for (int i = 0; i < AVG_SIZE - 1; i++) {
        array[i] = array[i + 1];
    }
    array[AVG_SIZE - 1] = newEMA;

    return newEMA;
}

//==============================================================================
// TASK FUNCTIONS (Code within tasks now uses defines)
//==============================================================================

//------------------------------------------------------------------------------
// Task: Read MPU9250 and Calculate Orientation
//------------------------------------------------------------------------------
void imuTask(void *pvParameters) {
    Serial.println("imuTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Run ~50Hz

    // Mahony filter internal variables
    static float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
    static float twoKp = 2.0f * 0.5f;
    static float twoKi = 2.0f * 0.0f;
    static unsigned long lastQuatTime = 0;
    static float integralFBx = 0, integralFBy = 0, integralFBz = 0;

    // Initialize MPU9250 communication here
    Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true); // Wake up
    Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true); // Accel +/- 2g
    Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true); // Gyro +/- 250dps

    lastQuatTime = micros();
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // --- Read Raw Data ---
        int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        // Read Accel
        Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(ACCEL_XOUT_H); Wire.endTransmission(false);
        Wire.requestFrom(MPU9250_ADDRESS, 6, true);
        raw_ax = Wire.read() << 8 | Wire.read(); raw_ay = Wire.read() << 8 | Wire.read(); raw_az = Wire.read() << 8 | Wire.read();
        // Read Gyro
        Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(GYRO_XOUT_H); Wire.endTransmission(false);
        Wire.requestFrom(MPU9250_ADDRESS, 6, true);
        raw_gx = Wire.read() << 8 | Wire.read(); raw_gy = Wire.read() << 8 | Wire.read(); raw_gz = Wire.read() << 8 | Wire.read();

        // --- Local calculation variables ---
        float local_ax = raw_ax / 16384.0f;
        float local_ay = raw_ay / 16384.0f;
        float local_az = raw_az / 16384.0f;
        float local_gx = raw_gx / 131.0f;
        float local_gy = raw_gy / 131.0f;
        float local_gz = raw_gz / 131.0f;

        // --- Calculate Angles (Mahony AHRS) ---
        unsigned long now = micros();
        float dt = (now - lastQuatTime) * 1e-6f; // Delta t in seconds
        lastQuatTime = now;
        if (dt <= 0) dt = 1e-3; // Prevent division by zero or negative dt

        // (Mahony filter calculations remain the same)
        float ax_calc = local_ax, ay_calc = local_ay, az_calc = local_az;
        float gx_calc = local_gx * PI/180.0f, gy_calc = local_gy * PI/180.0f, gz_calc = local_gz * PI/180.0f;

        float norm = sqrt(ax_calc*ax_calc + ay_calc*ay_calc + az_calc*az_calc);
        if (norm > 0.0f) {
           ax_calc /= norm; ay_calc /= norm; az_calc /= norm;
        } else {
             ax_calc = 0; ay_calc = 0; az_calc = 0;
        }

        float vx = 2.0f*(q1*q3 - q0*q2); float vy = 2.0f*(q0*q1 + q2*q3); float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        float ex = (ay_calc*vz - az_calc*vy); float ey = (az_calc*vx - ax_calc*vz); float ez = (ax_calc*vy - ay_calc*vx);

        if (twoKi > 0.0f) {
           integralFBx += twoKi * ex * dt; integralFBy += twoKi * ey * dt; integralFBz += twoKi * ez * dt;
           gx_calc += integralFBx; gy_calc += integralFBy; gz_calc += integralFBz;
        }
        gx_calc += twoKp * ex; gy_calc += twoKp * ey; gz_calc += twoKp * ez;

        float qDot0 = 0.5f * (-q1*gx_calc - q2*gy_calc - q3*gz_calc);
        float qDot1 = 0.5f * ( q0*gx_calc + q2*gz_calc - q3*gy_calc);
        float qDot2 = 0.5f * ( q0*gy_calc - q1*gz_calc + q3*gx_calc);
        float qDot3 = 0.5f * ( q0*gz_calc + q1*gy_calc - q2*gx_calc);

        q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

        // --- Calculate Euler Angles and G-Force ---
        float local_pitch = atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f/PI;
        float local_roll  = asin (2.0f*(q0*q2 - q3*q1)) * 180.0f/PI;
        float local_yaw   = atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f/PI;
        if (local_yaw < 0) local_yaw += 360.0f;

        float gravityVectorX = 2.0f * (q1 * q3 - q0 * q2);
        float gravityVectorY = 2.0f * (q0 * q1 + q2 * q3);
        float gravityVectorZ = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        float verticalAcceleration = local_ax * gravityVectorX + local_ay * gravityVectorY + local_az * gravityVectorZ;
        float local_gForce = verticalAcceleration;

        // --- Update Shared Variables (Protected by Mutex) ---
        if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
            pitchAvg = updateRollingAverage(pitch, local_pitch);
            rollAvg = updateRollingAverage(roll, local_roll);
            yawAvg = updateRollingAverage(yaw, local_yaw);
            gForceAvg = updateRollingAverage(gForce, local_gForce);

            accelX = local_ax;
            accelY = local_ay;
            accelZ = local_az;
            gyroX = local_gx * 180.0f / PI;
            gyroY = local_gy * 180.0f / PI;
            gyroZ = local_gz * 180.0f / PI;
            xSemaphoreGive(imuDataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//------------------------------------------------------------------------------
// Task: Read Hall Sensors, Calculate Speed and Direction
//------------------------------------------------------------------------------
void hallSensorTask(void *pvParameters) {
    Serial.println("hallSensorTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // Run frequently ~200Hz

    // Internal hall task variables (static within the task)
    static int lastHallSensorValue = HIGH;
    static int lastHallSensorValue2 = HIGH;
    static unsigned long hall1TriggerTime = 0;
    static unsigned long hall2TriggerTime = 0;
    static bool hall1Triggered = false;
    static bool hall2Triggered = false;
    static unsigned long lastTriggerTime = 0; // For speed calc

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
                if (currentSpeedAvg != 0.0f) {
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
                    currentSpeedAvg = updateRollingAverage(currentSpeed, local_speed);
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
// Task: Read Potentiometers and Update Thresholds (Arduino ADC Version)
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
// Task: Process Data, Detect Events, Handle Button
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


    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        unsigned long currentMillis = millis();
        // --- Local copies of data needed ---
        float local_accelX=0.0, local_accelY=0.0, local_accelZ=0.0, local_gForce=1.0;
        float local_pitch=0.0;
        // ** NEW: Local copies of thresholds for this cycle **
        float local_jumpThreshold = 0.5; // Default if mutex fails
        float local_landingThreshold = 2.0;
        float local_dropThreshold = 2.5;


        // --- Get IMU data ---
        if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
            local_accelX = accelX; local_accelY = accelY; local_accelZ = accelZ;
            local_gForce = gForceAvg;
            local_pitch = pitchAvg;
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
        bool current_imuDir = true;
        int current_imuState = 0;

        if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
            current_jump_state = jumpDetected;
            current_drop_state = dropDetected;
            current_imuDir = imuDirectionForward;
            current_imuState = imuSpeedState;
            xSemaphoreGive(eventDataMutex);
        } else {
             Serial.println("Warning: processingTask failed to get eventDataMutex for reading state!");
        }

        // --- Local flags/variables for this cycle's updates ---
        bool local_jumpDetected_this_cycle = current_jump_state;
        bool local_dropDetected_this_cycle = current_drop_state;
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
                 current_raw_pitch = pitchAvg; 
                 current_raw_roll = rollAvg; 
                 current_raw_yaw = yawAvg;
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
// Task: Handle BLE Notifications
//------------------------------------------------------------------------------
void bleTask(void *pvParameters) {
    Serial.println("bleTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Notify ~10Hz

    // Static internal vars for state
    static bool prevJumpDetected = false;
    static bool prevDropDetected = false;

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
            bool local_jump=false, local_drop=false, local_imuDir=true, local_hallDir=true;
            int local_imuState=0;
            // Note: Thresholds are not currently sent over BLE, but could be added if needed

             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeedAvg; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { local_pitch = pitchAvg; local_roll = rollAvg; local_yaw = yawAvg; local_gForce = gForceAvg; xSemaphoreGive(imuDataMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { local_pitchOffset = pitchOffset; local_rollOffset = rollOffset; local_yawOffset = yawOffset; xSemaphoreGive(offsetMutex); } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; local_imuDir = imuDirectionForward; local_imuState = imuSpeedState; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }

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

            if (notifyEvent && pEventCharacteristic) {
                pEventCharacteristic->setValue(&eventCode, sizeof(eventCode));
                pEventCharacteristic->notify();
            }
            prevJumpDetected = local_jump; // Use the static variable
            prevDropDetected = local_drop; // Use the static variable

            // Direction & State Notifications
            // ... (Remain the same) ...
            if(pImuDirectionCharacteristic) { uint8_t imuDirCode = local_imuDir ? 1 : 0; pImuDirectionCharacteristic->setValue(&imuDirCode, sizeof(imuDirCode)); pImuDirectionCharacteristic->notify(); }
            if(pHallDirectionCharacteristic){ uint8_t hallDirCode = local_hallDir ? 1 : 0; pHallDirectionCharacteristic->setValue(&hallDirCode, sizeof(hallDirCode)); pHallDirectionCharacteristic->notify(); }
            if(pImuSpeedStateCharacteristic){ uint8_t speedStateCode = (uint8_t)local_imuState; pImuSpeedStateCharacteristic->setValue(&speedStateCode, sizeof(speedStateCode)); pImuSpeedStateCharacteristic->notify(); }

        } else {
             prevJumpDetected = false; // Reset state if disconnected
             prevDropDetected = false;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


//------------------------------------------------------------------------------
// Task: Update OLED Display
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
         bool local_jump=false, local_drop=false, local_hallDir=true, isConnected=false;
         // ** NEW: Read thresholds **
         float local_jumpThreshold = 0.0;
         float local_landingThreshold = 0.0;
         float local_dropThreshold = 0.0;

         // Read state variables using mutexes
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) { isConnected = deviceConnected; xSemaphoreGive(bleConnectionMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeedAvg; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { local_pitch = pitchAvg; local_gForce = gForceAvg; /*Removed roll, yaw reads*/ xSemaphoreGive(imuDataMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { local_pitchOffset = pitchOffset; /*Removed roll, yaw offset reads*/ xSemaphoreGive(offsetMutex); } else { vTaskDelay(1); continue; }
         if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }
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
// SETUP FUNCTION
//==============================================================================
void setup() {
    // Zero rolling average arrays
    for (int i = 0; i < AVG_SIZE; i++) {
        pitch[i] = 0.0;
        roll[i] = 0.0;
        yaw[i] = 0.0;
        gForce[i] = 0.0;
        currentSpeed[i] = 0.0;
    }
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

    pAccelerometerZeroCharacteristic = pService->createCharacteristic(
        ACCELEROMETER_ZERO_CHARACTERISTIC_UUID, 
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pAccelerometerZeroCharacteristic->addDescriptor(new BLE2902());
    pAccelerometerZeroCharacteristic->setCallbacks(new AccelerometerZeroCallbacks());

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
// LOOP FUNCTION (Empty)
//==============================================================================
void loop() {
      vTaskDelay(pdMS_TO_TICKS(1000));
}
