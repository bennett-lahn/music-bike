// WARNING: THIS FILE IS DEPRECATED.
// Bike_Sensor_Filtered_IMU.ino is the only up to date code for the embedded platform
// This file is kept for reference only.

// Includes remain mostly the same
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// #include <TensorFlowLite.h>
// #include <tensorflow/lite/micro/all_ops_resolver.h>
// #include <tensorflow/lite/micro/micro_error_reporter.h>
// #include <tensorflow/lite/micro/micro_interpreter.h>
// #include <tensorflow/lite/schema/schema_generated.h>
// #include "model_data.h"

// --- FreeRTOS Includes ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For Mutexes

//==============================================================================
// CONFIGURATION SETTINGS (Copied from original code)
//==============================================================================

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Usually -1 if sharing Arduino reset pin
#define SCREEN_ADDRESS 0x3C
// Note: Adafruit_SSD1306 display object is declared later globally

// Pin definitions
#define HALL_SENSOR_PIN 5
#define SDA_PIN 8
#define SCL_PIN 9
#define HALL_SENSOR_PIN_2 6 // Pin for the second Hall sensor
#define BLE_LED_PIN 7       // Pin for the blue BLE connection indicator LED

// MPU9250 registers
#define MPU9250_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// Physical constants
#define WHEEL_DIAMETER_INCHES 26.0
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_INCHES * 2.54 * 3.14159)
#define HALF_CIRCUMFERENCE_CM (WHEEL_CIRCUMFERENCE_CM / 2.0)

// Threshold values
#define JUMP_THRESHOLD 0.5 // Adjusted for gForce check (e.g., < 0.5g for takeoff)
#define JUMP_DURATION_MIN 100
#define LANDING_THRESHOLD 2.0 // Adjusted for gForce check (e.g., > 2.0g for landing)
#define DROP_THRESHOLD 2.5    // Adjusted for gForce check (e.g., > 2.5g spike)
#define DIRECTION_THRESHOLD 0.3 // For IMU-based direction
#define IMU_DIRECTION_ACCEL_THRESHOLD 0.3 // Threshold for IMU-based direction change (+/- this value)

// Timing constants from original code (if specific names were used there)
const unsigned long SPEED_TIMEOUT = 3000;
const unsigned long EVENT_DISPLAY_DURATION = 2000;
const unsigned long debounceDelay = 50;
const unsigned long directionDetectionTimeout = 500; // ms timeout to reset hall direction detection state

//==============================================================================
// BLE CONFIGURATION (Copied from original code)
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

// --- IMU Data (Protected by imuDataMutex) ---
float pitch = 0.0, roll = 0.0, yaw = 0.0;
float gForce = 1.0; // Initialize to 1g
float accelX = 0.0, accelY = 0.0, accelZ = 0.0; // Raw scaled accel
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;   // Raw scaled gyro
// Mahony filter variables (used only within IMU task, could be static there)
float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
float twoKp = 2.0f * 0.5f;
float twoKi = 2.0f * 0.0f;
unsigned long lastQuatTime = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

// --- Orientation Offsets (Protected by offsetMutex) ---
float pitchOffset = 0.0, rollOffset = 0.0, yawOffset = 0.0;

// --- Hall Sensor Data (Protected by hallDataMutex) ---
float currentSpeed = 0.0;          // km/h
bool hallDirectionForward = true;
int hallSensorValue = HIGH;       // Current raw value
int hallSensorValue2 = HIGH;      // Current raw value 2
// Internal hall task variables (could be static within the task)
int lastHallSensorValue = HIGH;
int lastHallSensorValue2 = HIGH;
unsigned long hall1TriggerTime = 0;
unsigned long hall2TriggerTime = 0;
bool hall1Triggered = false;
bool hall2Triggered = false;
unsigned long lastTriggerTime = 0; // For speed calc

// --- Processed/Event Data (Protected by eventDataMutex) ---
bool jumpDetected = false;
bool dropDetected = false;
bool imuDirectionForward = true;
int imuSpeedState = 0; // 0=Stop/Slow, 1=Medium, 2=Fast
// Internal event task variables (could be static within the task)
bool inJumpState = false;
unsigned long jumpStartTime = 0;
unsigned long lastJumpTime = 0;
unsigned long lastDropTime = 0;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// -- TensorFlow Global Variables ---
// const int numSamples = MAX_SAMPLE_LENGTH;
float sampleRate = 100.0;
unsigned long sampleInterval;   // Will be calculated based on sample rate

// Sliding window parameters
int overlapPercentage = 50;     // Adjustable: percentage of overlap (0-90)
int slidingStep;                // Will be calculated based on overlap percentage
unsigned long lastSampleTime = 0;
int samplesRead = 0;            // Start from 0 to collect a full set of samples
// tflite::MicroErrorReporter tflErrorReporter;
// tflite::AllOpsResolver tflOpsResolver;
// const tflite::Model* tflModel = nullptr;
// tflite::MicroInterpreter* tflInterpreter = nullptr;
// TfLiteTensor* tflInputTensor = nullptr;
// TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model
// constexpr int tensorArenaSize = 100 * 1024;
// byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// --- BLE State (Protected by bleConnectionMutex) ---
volatile bool deviceConnected = false; // Volatile because modified by ISR/Callback context
bool oldDeviceConnected = false;       // Only used within BLE task, maybe make local?

// --- BLE Objects (Global, initialized in setup) ---
BLEServer* pServer = NULL;
BLECharacteristic* pSpeedCharacteristic = NULL;
BLECharacteristic* pPitchCharacteristic = NULL; 
BLECharacteristic* pRollCharacteristic = NULL;  
BLECharacteristic* pYawCharacteristic = NULL;   
BLECharacteristic* pGForceCharacteristic = NULL;
BLECharacteristic* pEventCharacteristic = NULL;
BLECharacteristic* pImuDirectionCharacteristic = NULL;
BLECharacteristic* pHallDirectionCharacteristic = NULL;
BLECharacteristic* pImuSpeedStateCharacteristic = NULL;
BLECharacteristic* pAccelerometerZeroCharacteristic = NULL;

// --- Display Object (Global) ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Task Handles (Optional, for controlling tasks later) ---
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t hallSensorTaskHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

//==============================================================================
// BLE CALLBACK CLASSES
//==============================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) {
        if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = true;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Connected");
        digitalWrite(BLE_LED_PIN, HIGH);
    }

    void onDisconnect(BLEServer* pServerInstance) {
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            deviceConnected = false;
            xSemaphoreGive(bleConnectionMutex);
        }
        Serial.println("BLE Device Disconnected");
        digitalWrite(BLE_LED_PIN, LOW);
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
                    current_raw_pitch = pitch;
                    current_raw_roll = roll;
                    current_raw_yaw = yaw;
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
// FUNCTION PROTOTYPES FOR HELPER FUNCTIONS
//==============================================================================
// (None needed currently)

//==============================================================================
// TASK FUNCTIONS (Code within tasks now uses defines)
//==============================================================================

//------------------------------------------------------------------------------
// Task: Read MPU9250 and Calculate Orientation
//------------------------------------------------------------------------------
void imuTask(void *pvParameters) {
    Serial.println("imuTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Run ~100Hz

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
            pitch = local_pitch;
            roll = local_roll;
            yaw = local_yaw;
            gForce = local_gForce;
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
// Task: Process Data, Detect Events, Handle Button
//------------------------------------------------------------------------------
void processingTask(void *pvParameters) {
    Serial.println("processingTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(15); // Run ~66Hz

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        unsigned long currentMillis = millis();
        // --- Local copies of data needed ---
        float local_accelX=0.0, local_accelY=0.0, local_accelZ=0.0, local_gForce=1.0;
        float local_pitch=0.0;

        // --- Get IMU data ---
        if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
            local_accelX = accelX; local_accelY = accelY; local_accelZ = accelZ;
            local_gForce = gForce;
            local_pitch = pitch; // Read pitch needed for gravity comp
            xSemaphoreGive(imuDataMutex);
        } else {
            Serial.println("Warning: processingTask failed to get imuDataMutex!");
            // Continue execution with potentially stale data if necessary
        }

        // --- Local copies of event/state variables ---
        bool current_jump_state = false;
        bool current_drop_state = false;
        bool current_imuDir = true; // Default if mutex fails
        int current_imuState = 0;   // Default if mutex fails

        if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
            current_jump_state = jumpDetected;
            current_drop_state = dropDetected;
            current_imuDir = imuDirectionForward; // Read current direction
            current_imuState = imuSpeedState;     // Read current speed state
            xSemaphoreGive(eventDataMutex);
        } else {
             Serial.println("Warning: processingTask failed to get eventDataMutex for reading state!");
             // Removed 'continue'. Task will proceed using default/previous values for current_imuDir/State.
        }

        // --- Local flags/variables for this cycle's updates ---
        bool local_jumpDetected_this_cycle = current_jump_state;
        bool local_dropDetected_this_cycle = current_drop_state;
        bool local_imuDirectionForward = current_imuDir; // Start assuming current state holds
        int local_imuSpeedState = current_imuState;

        // --- Jump & Drop Detection (Using gForce and defines) ---
        // (Jump/Drop logic...)
        if (!inJumpState && local_gForce < JUMP_THRESHOLD) {
             inJumpState = true; jumpStartTime = currentMillis;
        }
        if (inJumpState && local_gForce > LANDING_THRESHOLD) {
            unsigned long jumpDuration = currentMillis - jumpStartTime;
            if (jumpDuration > JUMP_DURATION_MIN) {
                local_jumpDetected_this_cycle = true; lastJumpTime = currentMillis;
                if(!current_jump_state) { Serial.println("JUMP DETECTED! Duration: " + String(jumpDuration) + "ms G: " + String(local_gForce, 2)); }
            }
            inJumpState = false;
        }
        if(inJumpState && (currentMillis - jumpStartTime > 5000)) {
             inJumpState = false; Serial.println("Jump state timeout");
        }
        if (!inJumpState && local_gForce > DROP_THRESHOLD && !current_drop_state) {
            local_dropDetected_this_cycle = true; lastDropTime = currentMillis;
            Serial.println("DROP DETECTED! Impact G-force: " + String(local_gForce, 2));
        }
        if (current_jump_state && (currentMillis - lastJumpTime > EVENT_DISPLAY_DURATION)) {
            local_jumpDetected_this_cycle = false;
        }
        if (current_drop_state && (currentMillis - lastDropTime > EVENT_DISPLAY_DURATION)) {
            local_dropDetected_this_cycle = false;
        }

        // --- IMU-Based Direction & Speed State ---
        float gravityCompAccelY = local_accelY - sin(local_pitch * PI / 180.0f);

        // *** IMU Direction Logic (Hysteresis) ***
        if (current_imuDir == true) { // If we were going forward...
            if (gravityCompAccelY > IMU_DIRECTION_ACCEL_THRESHOLD) { // ...check for significant reverse accel
                local_imuDirectionForward = false; // Switch to reverse
            }
        } else { // If we were going reverse...
            if (gravityCompAccelY < -IMU_DIRECTION_ACCEL_THRESHOLD) { // ...check for significant forward accel
                local_imuDirectionForward = true; // Switch to forward
            }
        }
        // ****************************************

        // IMU Speed State Calculation
        float accelMagnitudeXY = sqrt(local_accelX * local_accelX + gravityCompAccelY * gravityCompAccelY);
        if (accelMagnitudeXY > 0.8f) local_imuSpeedState = 2; // Fast
        else if (accelMagnitudeXY > 0.2f) local_imuSpeedState = 1; // Medium
        else local_imuSpeedState = 0; // Slow/Stopped

        // --- Update Event Data (State data like jump/drop/direction/speed_state) ---
        if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
            jumpDetected = local_jumpDetected_this_cycle;
            dropDetected = local_dropDetected_this_cycle;
            imuDirectionForward = local_imuDirectionForward; // Update with holding logic result
            imuSpeedState = local_imuSpeedState;
            xSemaphoreGive(eventDataMutex);
        } else {
             Serial.println("Warning: processingTask failed to get eventDataMutex for updating state!");
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait for next cycle
    } // End while(1)
} // End processingTask

//------------------------------------------------------------------------------
// Task: Handle BLE Notifications
//------------------------------------------------------------------------------
void bleTask(void *pvParameters) {
    Serial.println("bleTask started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Notify ~40Hz

    xLastWakeTime = xTaskGetTickCount();
    bool prevJumpDetected = false;
    bool prevDropDetected = false;

    while (1) {
        bool isConnected = false;
         if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) {
            isConnected = deviceConnected;
            xSemaphoreGive(bleConnectionMutex);
         }

        if (isConnected) {
            // --- Read required data (using mutexes) ---
            float local_speed=0.0, local_pitch=0.0, local_roll=0.0, local_yaw=0.0, local_gForce=1.0;
            float local_pitchOffset=0.0, local_rollOffset=0.0, local_yawOffset=0.0;
            bool local_jump=false, local_drop=false, local_imuDir=true, local_hallDir=true;
            int local_imuState=0;

             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
                 local_speed = currentSpeed; local_hallDir = hallDirectionForward;
                 xSemaphoreGive(hallDataMutex);
             } else { vTaskDelay(1); continue; } // Avoid busy wait if mutex locked briefly

             if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
                 local_pitch = pitch; local_roll = roll; local_yaw = yaw; local_gForce = gForce;
                 xSemaphoreGive(imuDataMutex);
             } else { vTaskDelay(1); continue; }

             if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) {
                 local_pitchOffset = pitchOffset; local_rollOffset = rollOffset; local_yawOffset = yawOffset;
                 xSemaphoreGive(offsetMutex);
             } else { vTaskDelay(1); continue; }

             if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) {
                 local_jump = jumpDetected; local_drop = dropDetected;
                 local_imuDir = imuDirectionForward; local_imuState = imuSpeedState;
                 xSemaphoreGive(eventDataMutex);
             } else { vTaskDelay(1); continue; }

            // --- Calculate Zeroed Values ---
            float zeroedPitch = local_pitch - local_pitchOffset;
            float zeroedRoll = local_roll - local_rollOffset;
            float zeroedYaw = local_yaw - local_yawOffset;
            while (zeroedYaw < 0) zeroedYaw += 360;
            while (zeroedYaw >= 360) zeroedYaw -= 360;

            // --- Send Notifications (Check characteristic pointers are not NULL) ---
            if(pSpeedCharacteristic) {
               pSpeedCharacteristic->setValue((uint8_t*)&local_speed, sizeof(local_speed));
               pSpeedCharacteristic->notify();
            }
             if(pPitchCharacteristic) {
                pPitchCharacteristic->setValue((uint8_t*)&zeroedPitch, sizeof(zeroedPitch));
                pPitchCharacteristic->notify();
             }
            if(pRollCharacteristic) {
               pRollCharacteristic->setValue((uint8_t*)&zeroedRoll, sizeof(zeroedRoll));
               pRollCharacteristic->notify();
            }
             if(pYawCharacteristic) {
                pYawCharacteristic->setValue((uint8_t*)&zeroedYaw, sizeof(zeroedYaw));
                pYawCharacteristic->notify();
             }
             if(pGForceCharacteristic) {
                pGForceCharacteristic->setValue((uint8_t*)&local_gForce, sizeof(local_gForce));
                pGForceCharacteristic->notify();
             }

            // Event Notification (Only on change)
            uint8_t eventCode = 0;
            bool notifyEvent = false;
            if (local_jump && !prevJumpDetected) { eventCode = 1; notifyEvent = true; Serial.println("BLE Notify: JUMP (1)"); }
            else if (local_drop && !prevDropDetected) { eventCode = 2; notifyEvent = true; Serial.println("BLE Notify: DROP (2)"); }
            // else if ((!local_jump && prevJumpDetected) || (!local_drop && prevDropDetected)) { eventCode = 0; notifyEvent = true; Serial.println("BLE Notify: Event Cleared (0)"); } // Optional: Notify clear

            if (notifyEvent && pEventCharacteristic) {
                pEventCharacteristic->setValue(&eventCode, sizeof(eventCode));
                pEventCharacteristic->notify();
            }
            prevJumpDetected = local_jump;
            prevDropDetected = local_drop;

            // Direction & State Notifications
            if(pImuDirectionCharacteristic) {
               uint8_t imuDirCode = local_imuDir ? 1 : 0;
               pImuDirectionCharacteristic->setValue(&imuDirCode, sizeof(imuDirCode));
               pImuDirectionCharacteristic->notify();
            }
            if(pHallDirectionCharacteristic){
               uint8_t hallDirCode = local_hallDir ? 1 : 0;
               pHallDirectionCharacteristic->setValue(&hallDirCode, sizeof(hallDirCode));
               pHallDirectionCharacteristic->notify();
            }
            if(pImuSpeedStateCharacteristic){
               uint8_t speedStateCode = (uint8_t)local_imuState;
               pImuSpeedStateCharacteristic->setValue(&speedStateCode, sizeof(speedStateCode));
               pImuSpeedStateCharacteristic->notify();
            }

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

    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); display.println("Music Bike RTOS"); display.println("Initializing...");
    display.display();
    vTaskDelay(pdMS_TO_TICKS(1000));

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
         // --- Read required data ---
         float local_speed=0.0, local_pitch=0.0, local_roll=0.0, local_yaw=0.0, local_gForce=1.0;
         float local_pitchOffset=0.0, local_rollOffset=0.0, local_yawOffset=0.0;
         bool local_jump=false, local_drop=false, local_hallDir=true, isConnected=false;

         // Read state variables using mutexes
          if (xSemaphoreTake(bleConnectionMutex, portMAX_DELAY) == pdTRUE) { isConnected = deviceConnected; xSemaphoreGive(bleConnectionMutex); } else { vTaskDelay(1); continue; }
          if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeed; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
          if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { local_pitch = pitch; local_roll = roll; local_yaw = yaw; local_gForce = gForce; xSemaphoreGive(imuDataMutex); } else { vTaskDelay(1); continue; }
          if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { local_pitchOffset = pitchOffset; local_rollOffset = rollOffset; local_yawOffset = yawOffset; xSemaphoreGive(offsetMutex); } else { vTaskDelay(1); continue; }
          if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }

         // --- Calculate Zeroed Values ---
         float zeroedPitch = local_pitch - local_pitchOffset;
         float zeroedRoll = local_roll - local_rollOffset;
         float zeroedYaw = local_yaw - local_yawOffset;
         while (zeroedYaw < 0) zeroedYaw += 360;
         while (zeroedYaw >= 360) zeroedYaw -= 360;

        // --- Update Display ---
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        if (isConnected) { display.setCursor(SCREEN_WIDTH - 12, 0); display.print(" B"); } // Uses define
        display.setCursor(0, 0); display.print("G:"); display.print(local_gForce, 2);
        display.setCursor(0, 10); display.print("J:"); display.print(local_jump ? "Y" : "N");
        display.setCursor(64, 10); display.print("D:"); display.print(local_drop ? "Y" : "N");
        display.setCursor(0, 25); display.print("P:"); display.print(zeroedPitch, 1);
        display.setCursor(64, 25); display.print("R:"); display.print(zeroedRoll, 1);
        display.setCursor(0, 35); display.print("Y:"); display.print(zeroedYaw, 1);
        display.setCursor(0, 45); display.print("Spd:"); display.print(local_speed, 1);
        display.setCursor(64, 45); display.print("Dir:"); display.print(local_hallDir ? "F" : "R");

        display.display();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//==============================================================================
// SETUP FUNCTION
//==============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Music Bike Sensor System Initializing (RTOS)...");

    // --- Initialize Hardware Pins ---
    pinMode(HALL_SENSOR_PIN, INPUT); // Uses define
    pinMode(HALL_SENSOR_PIN_2, INPUT); // Uses define
    pinMode(BLE_LED_PIN, OUTPUT); // Uses define
    digitalWrite(BLE_LED_PIN, LOW);

    // --- Initialize I2C ---
    Wire.begin(SDA_PIN, SCL_PIN); // Uses defines

    // --- Initialize OLED Display ---
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Uses define
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

    if (!imuDataMutex || !hallDataMutex || !eventDataMutex || !offsetMutex || !bleConnectionMutex) {
         Serial.println("Failed to create mutexes!"); for(;;);
    }

    // --- BLE Initialization ---
    Serial.println("Initializing BLE...");
    BLEDevice::init("MusicBike_RTOS");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Use SERVICE_UUID define
    BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 32);

    // Create Characteristics (Use defines for UUIDs)
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
    pAdvertising->addServiceUUID(SERVICE_UUID); // Use define
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("BLE Initialized and Advertising!");

    display.clearDisplay(); display.setCursor(0,0);
    display.println("BLE Advertising!"); display.println("Creating tasks...");
    display.display();
    delay(500);

    // --- Create Tasks ---
    xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 5, &imuTaskHandle, 1);
    xTaskCreatePinnedToCore(hallSensorTask, "HallTask", 4096, NULL, 4, &hallSensorTaskHandle, 1);
    xTaskCreatePinnedToCore(processingTask, "ProcessingTask", 4096, NULL, 3, &processingTaskHandle, 1);
    xTaskCreatePinnedToCore(bleTask, "BLETask", 4096, NULL, 2, &bleTaskHandle, 0); // Run BLE on other core
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, &displayTaskHandle, 1);

    Serial.println("Tasks created. Initialization complete!");
    display.clearDisplay(); display.setCursor(0,0); display.println("Tasks Running!"); display.display();
}

//==============================================================================
// LOOP FUNCTION (Empty)
//==============================================================================
void loop() {
     vTaskDelay(pdMS_TO_TICKS(1000));
}

//==============================================================================
// HELPER FUNCTIONS (If any)
//==============================================================================
// (None currently)