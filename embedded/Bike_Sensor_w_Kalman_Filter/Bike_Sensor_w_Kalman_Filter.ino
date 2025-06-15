// WARNING: THIS FILE IS DEPRECATED.
// Bike_Sensor_Filtered_IMU.ino is the only up to date code for the embedded platform
// This file is kept for reference only. It attempts to implement a Kalman Filter to reduce IMU noise.
// It also adds correction for yaw drift.

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

// =============================================================================
// SENSOR FUSION IMPLEMENTATION NOTES
// =============================================================================
//
// This implementation uses the Mahony AHRS filter with magnetometer integration
// for robust 9-axis sensor fusion. The system combines:
//
// - MPU9250 accelerometer and gyroscope (6-axis)
// - AK8963 magnetometer (3-axis) for yaw drift correction
// - Mahony filter for quaternion-based orientation estimation
// - Kalman filter for additional smoothing and state estimation
//
// BENEFITS OF THIS APPROACH:
// - No external dependencies (self-contained implementation)
// - Magnetometer prevents yaw drift common in gyro-only systems
// - Proven Mahony algorithm with good performance characteristics
// - Full control over filter parameters and behavior
// - Compatible with existing Kalman filter integration
//
// MAGNETOMETER INTEGRATION:
// - Factory calibration automatically read from AK8963
// - Continuous 100Hz measurement mode
// - Tilt compensation for accurate heading
// - Dual error correction (gravity + magnetic field)
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

#include <math.h> // For sqrt, atan2, etc.

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
#define THRESHOLD 2.0f

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

// Kalman Filter Confidence Threshold
#define KALMAN_VARIANCE_THRESHOLD 1.0f // If variance is above this, EMA might be preferred (increased from 0.1f)

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
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_INT_PIN_CFG 0x37

// AK8963 Magnetometer registers (accessed through MPU9250)
#define AK8963_ADDRESS 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL 0x0A
#define AK8963_ASTC 0x0C
#define AK8963_I2CDIS 0x0F
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

// MPU9250 I2C Master registers for magnetometer access
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR 0x25
#define MPU9250_I2C_SLV0_REG 0x26
#define MPU9250_I2C_SLV0_CTRL 0x27
#define MPU9250_EXT_SENS_DATA_00 0x49

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
SemaphoreHandle_t i2cMutex = NULL; // Mutex for I2C bus access

// --- IMU Data (Protected by imuDataMutex) ---
// EMA values
float pitchAvg = 0.0, rollAvg = 0.0, yawAvg = 0.0;
float gForceAvg = 1.0; // Initialize to 1g

// Kalman Filtered Data (also protected by imuDataMutex)
float kalmanPitchDeg = 0.0f;
float kalmanRollDeg = 0.0f;
float kalmanYawDeg = 0.0f;
float kalmanPitchVariance = 1.0f; // Initialize with high uncertainty
float kalmanRollVariance = 1.0f;  // Initialize with high uncertainty
float kalmanYawVariance = 1.0f;   // Initialize with high uncertainty

float accelX = 0.0, accelY = 0.0, accelZ = 0.0; // Raw scaled accel
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;   // Raw scaled gyro
float magX = 0.0, magY = 0.0, magZ = 0.0;      // Raw scaled magnetometer

// --- Orientation Offsets (Protected by offsetMutex) ---
float pitchOffset = 0.0, rollOffset = 0.0, yawOffset = 0.0;

// --- Hall Sensor Data (Protected by hallDataMutex) ---
float currentSpeedAvg = 0.0; // EMA value
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

// Magnetometer calibration values (factory calibration from AK8963)
float magCalX = 1.0f, magCalY = 1.0f, magCalZ = 1.0f;

//==============================================================================
// KALMAN FILTER CLASS DEFINITION
//==============================================================================
class KalmanFilter {
public:
    // Constructor
    // q_angle: Process noise variance for the angle estimates
    // q_rate: Process noise variance for the rate estimates
    // q_speed: Process noise variance for the speed estimate
    // r_angle: Measurement noise variance for the angle measurements
    // r_speed: Measurement noise variance for the speed measurement
    KalmanFilter(float q_angle, float q_rate, float q_speed, float r_angle, float r_speed) {
        // Process noise covariances
        Q_angle = q_angle;
        Q_rate = q_rate;
        Q_speed = q_speed;
        
        // Measurement noise covariances
        R_angle = r_angle;
        R_speed = r_speed;

        // Initialize state vector [pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate, speed, speed_rate]
        x[0] = 0.0f;  // pitch
        x[1] = 0.0f;  // roll
        x[2] = 0.0f;  // yaw
        x[3] = 0.0f;  // pitch_rate
        x[4] = 0.0f;  // roll_rate
        x[5] = 0.0f;  // yaw_rate
        x[6] = 0.0f;  // speed
        x[7] = 0.0f;  // speed_rate

        // Initialize covariance matrix P
        // Start with high uncertainty
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                P[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // Predict step: call with gyro rates (rad/s), acceleration (m/s^2), and time step dt (s)
    void predict(float gyro_pitch_rate, float gyro_roll_rate, float gyro_yaw_rate, float acceleration, float dt) {
        // State transition matrix F (8x8)
        float F[8][8] = {
            {1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, dt},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}
        };

        // Process noise matrix Q (8x8)
        float Q[8][8] = {
            {Q_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, Q_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, Q_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, Q_rate, 0.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, Q_rate, 0.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Q_rate, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Q_speed, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Q_speed}
        };

        // Predict state
        float x_new[8];
        for(int i = 0; i < 8; i++) {
            x_new[i] = 0.0f;
            for(int j = 0; j < 8; j++) {
                x_new[i] += F[i][j] * x[j];
            }
        }

        // Add control inputs
        x_new[3] = gyro_pitch_rate;  // Update pitch rate from gyro
        x_new[4] = gyro_roll_rate;   // Update roll rate from gyro
        x_new[5] = gyro_yaw_rate;    // Update yaw rate from gyro
        x_new[7] = acceleration;     // Update speed rate from acceleration

        // Predict covariance
        float P_new[8][8];
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                P_new[i][j] = 0.0f;
                for(int k = 0; k < 8; k++) {
                    P_new[i][j] += F[i][k] * P[k][j];
                }
            }
        }

        // Add process noise
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                P_new[i][j] += Q[i][j];
            }
        }

        // Update state and covariance
        for(int i = 0; i < 8; i++) {
            x[i] = x_new[i];
            for(int j = 0; j < 8; j++) {
                P[i][j] = P_new[i][j];
            }
        }
    }

    // Update step: call with measured angles (rad) and speed (m/s)
    void update(float measured_pitch, float measured_roll, float measured_yaw, float measured_speed) {
        // Measurement matrix H (4x8)
        float H[4][8] = {
            {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // pitch measurement
            {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // roll measurement
            {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // yaw measurement
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}   // speed measurement
        };

        // Measurement noise matrix R (4x4)
        float R[4][4] = {
            {R_angle, 0.0f, 0.0f, 0.0f},
            {0.0f, R_angle, 0.0f, 0.0f},
            {0.0f, 0.0f, R_angle, 0.0f},
            {0.0f, 0.0f, 0.0f, R_speed}
        };

        // Innovation
        float y[4] = {
            measured_pitch - x[0],
            measured_roll - x[1],
            measured_yaw - x[2],
            measured_speed - x[6]
        };

        // Innovation covariance S = H*P*H' + R (4x4)
        float S[4][4];
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                S[i][j] = R[i][j];
                for(int k = 0; k < 8; k++) {
                    for(int l = 0; l < 8; l++) {
                        S[i][j] += H[i][k] * P[k][l] * H[j][l];
                    }
                }
            }
        }

        // Invert S matrix (simplified for 4x4 diagonal-dominant case)
        float S_inv[4][4];
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                S_inv[i][j] = (i == j) ? (1.0f / S[i][i]) : 0.0f;
            }
        }

        // Kalman gain K = P*H'*S_inv (8x4)
        float K[8][4];
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 4; j++) {
                K[i][j] = 0.0f;
                for(int k = 0; k < 8; k++) {
                    for(int l = 0; l < 4; l++) {
                        K[i][j] += P[i][k] * H[l][k] * S_inv[l][j];
                    }
                }
            }
        }

        // Update state
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 4; j++) {
                x[i] += K[i][j] * y[j];
            }
        }

        // Update covariance P = (I - K*H)*P
        float P_new[8][8];
        float I_KH[8][8];
        
        // Calculate I - K*H
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                I_KH[i][j] = (i == j) ? 1.0f : 0.0f; // Identity matrix
                for(int k = 0; k < 4; k++) {
                    I_KH[i][j] -= K[i][k] * H[k][j];
                }
            }
        }
        
        // P_new = (I - K*H) * P
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                P_new[i][j] = 0.0f;
                for(int k = 0; k < 8; k++) {
                    P_new[i][j] += I_KH[i][k] * P[k][j];
                }
            }
        }

        // Update covariance matrix
        for(int i = 0; i < 8; i++) {
            for(int j = 0; j < 8; j++) {
                P[i][j] = P_new[i][j];
            }
        }
    }

    // Getters
    float getPitchRad() { return x[0]; }
    float getRollRad() { return x[1]; }
    float getYawRad() { return x[2]; }
    float getPitchRateRadS() { return x[3]; }
    float getRollRateRadS() { return x[4]; }
    float getYawRateRadS() { return x[5]; }
    float getSpeedMS() { return x[6]; }
    float getSpeedRateMS2() { return x[7]; }

    float getPitchDeg() { return x[0] * 180.0f / M_PI; }
    float getRollDeg() { return x[1] * 180.0f / M_PI; }
    float getYawDeg() { 
        float yaw_deg = x[2] * 180.0f / M_PI; 
        while (yaw_deg < 0.0f) yaw_deg += 360.0f;
        while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
        return yaw_deg;
    }
    float getPitchRateDegS() { return x[3] * 180.0f / M_PI; }
    float getRollRateDegS() { return x[4] * 180.0f / M_PI; }
    float getYawRateDegS() { return x[5] * 180.0f / M_PI; }
    float getSpeedKMH() { return x[6] * 3.6f; }

    // Get variance of estimates
    float getPitchVariance() { return P[0][0]; }
    float getRollVariance() { return P[1][1]; }
    float getYawVariance() { return P[2][2]; }
    float getSpeedVariance() { return P[6][6]; }

private:
    float Q_angle;    // Process noise variance for angles
    float Q_rate;     // Process noise variance for rates
    float Q_speed;    // Process noise variance for speed
    float R_angle;    // Measurement noise variance for angles
    float R_speed;    // Measurement noise variance for speed

    float x[8];       // State vector
    float P[8][8];    // Covariance matrix
};

// --- Kalman Filter Instances ---
// Tunable parameters for the Kalman filters
// Q_angle: Process noise for angles (increased for more responsiveness)
// Q_rate: Process noise for rates (increased for more responsiveness)
// Q_speed: Process noise for speed (0.01f)
// R_angle: Measurement noise for angles (decreased for more trust in measurements)
// R_speed: Measurement noise for speed (0.1f)
KalmanFilter kalmanFilter(0.01f, 0.01f, 0.01f, 0.01f, 0.1f);

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
                
                // Get current orientation values using the same logic as BLE task
                float current_pitch = 0.0, current_roll = 0.0, current_yaw = 0.0;
                
                if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
                    // Use the same selection logic as the BLE task
                    float ema_pitch = pitchAvg;
                    float ema_roll = rollAvg;
                    float ema_yaw = yawAvg;
                    
                    float kf_pitch = kalmanPitchDeg;
                    float kf_roll = kalmanRollDeg;
                    float kf_yaw = kalmanYawDeg;
                    float kf_pitch_variance = kalmanPitchVariance;
                    float kf_roll_variance = kalmanRollVariance;
                    float kf_yaw_variance = kalmanYawVariance;
                    
                    // Use the same variance-based selection as BLE task
                    if (kf_pitch_variance < KALMAN_VARIANCE_THRESHOLD) {
                        current_pitch = kf_pitch; // Use Kalman pitch
                    } else {
                        current_pitch = ema_pitch; // Use EMA pitch
                    }
                    
                    if (kf_roll_variance < KALMAN_VARIANCE_THRESHOLD) {
                        current_roll = kf_roll; // Use Kalman roll
                    } else {
                        current_roll = ema_roll; // Use EMA roll
                    }
                    
                    if (kf_yaw_variance < KALMAN_VARIANCE_THRESHOLD) {
                        current_yaw = kf_yaw; // Use Kalman yaw
                    } else {
                        current_yaw = ema_yaw; // Use EMA yaw
                    }
                    
                    xSemaphoreGive(imuDataMutex);
                    
                    // Update offsets
                    if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) {
                        pitchOffset = current_pitch;
                        rollOffset = current_roll;
                        yawOffset = current_yaw;
                        xSemaphoreGive(offsetMutex);
                        Serial.println(">>> BLE Zero: SUCCESS - Zero position offsets updated!");
                        Serial.printf(">>> BLE Zero: New offsets - P:%.2f R:%.2f Y:%.2f\n", 
                                     pitchOffset, rollOffset, yawOffset);
                    }
                }
            }
        }
    }
};

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

// Apply magnetometer correction to yaw
float correctYawWithMagnetometer(float dmpYaw, float mx, float my, float mz, float pitch, float roll) {
    // Tilt compensation for magnetometer
    float magX_comp = mx * cos(pitch) + mz * sin(pitch);
    float magY_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    
    // Calculate magnetic heading
    float magHeading = atan2(magY_comp, magX_comp);
    if (magHeading < 0) magHeading += 2 * M_PI;
    
    // Simple complementary filter between DMP yaw and magnetometer heading
    // Adjust the weight (0.02) based on your needs - lower values trust DMP more
    float correctedYaw = 0.98f * dmpYaw + 0.02f * magHeading;
    
    return correctedYaw;
}

// Initialize AK8963 magnetometer
void initMagnetometer() {
    Serial.println("Initializing magnetometer...");
    
    // Enable I2C master mode
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_USER_CTRL);
    Wire.write(0x20); // Enable I2C master mode
    Wire.endTransmission();
    
    // Set I2C master clock to 400kHz
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_I2C_MST_CTRL);
    Wire.write(0x0D); // I2C master clock 400kHz
    Wire.endTransmission();
    
    // Enable bypass mode to access magnetometer directly first
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_INT_PIN_CFG);
    Wire.write(0x02); // Enable I2C bypass
    Wire.endTransmission();
    
    delay(100);
    
    // Check magnetometer WHO_AM_I
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(AK8963_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDRESS, 1, true);
    uint8_t whoami = Wire.read();
    Serial.printf("Magnetometer WHO_AM_I: 0x%02X (should be 0x48)\n", whoami);
    
    // Read factory calibration values
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(AK8963_ASAX);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDRESS, 3, true);
    uint8_t asax = Wire.read();
    uint8_t asay = Wire.read();
    uint8_t asaz = Wire.read();
    
    // Calculate calibration multipliers
    magCalX = (float)(asax - 128) / 256.0f + 1.0f;
    magCalY = (float)(asay - 128) / 256.0f + 1.0f;
    magCalZ = (float)(asaz - 128) / 256.0f + 1.0f;
    
    Serial.printf("Mag calibration: X=%.3f, Y=%.3f, Z=%.3f\n", magCalX, magCalY, magCalZ);
    
    // Set magnetometer to power down mode
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(AK8963_CNTL);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
    
    // Set magnetometer to continuous measurement mode 2 (100Hz)
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(AK8963_CNTL);
    Wire.write(0x16); // 16-bit output, continuous mode 2
    Wire.endTransmission();
    delay(10);
    
    // Disable bypass mode and use I2C master
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_INT_PIN_CFG);
    Wire.write(0x00); // Disable I2C bypass
    Wire.endTransmission();
    
    // Set up automatic magnetometer reading via I2C master
    // Read 7 bytes starting from ST1 (includes ST1, X, Y, Z, ST2)
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_I2C_SLV0_ADDR);
    Wire.write(AK8963_ADDRESS | 0x80); // Read from AK8963
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_I2C_SLV0_REG);
    Wire.write(AK8963_ST1); // Start reading from ST1
    Wire.endTransmission();
    
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_I2C_SLV0_CTRL);
    Wire.write(0x87); // Enable and read 7 bytes
    Wire.endTransmission();
    
    Serial.println("Magnetometer initialized successfully!");
}

// Read magnetometer data
void readMagnetometer(float &mx, float &my, float &mz) {
    // Read magnetometer data from external sensor data registers
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(MPU9250_EXT_SENS_DATA_00);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 7, true);
    
    uint8_t st1 = Wire.read(); // Status 1
    int16_t raw_mx = Wire.read() | (Wire.read() << 8); // X LSB, X MSB
    int16_t raw_my = Wire.read() | (Wire.read() << 8); // Y LSB, Y MSB
    int16_t raw_mz = Wire.read() | (Wire.read() << 8); // Z LSB, Z MSB
    uint8_t st2 = Wire.read(); // Status 2
    
    // Check if data is ready and not overrun
    if ((st1 & 0x01) && !(st2 & 0x08)) {
        // Apply factory calibration and convert to µT (microTesla)
        // AK8963 sensitivity: 0.15 µT/LSB for 16-bit mode
        mx = (float)raw_mx * 0.15f * magCalX;
        my = (float)raw_my * 0.15f * magCalY;
        mz = (float)raw_mz * 0.15f * magCalZ;
    }
    // If data not ready or overrun, keep previous values
}

// Simple EMA (Exponential Moving Average) calculation with spike rejection
float updateEMA(float previousEMA, float newVal) {
    float alpha = 2.0f / (AVG_SIZE + 1);
    
    // Spike rejection: if newVal deviates too much from previous EMA, ignore it
    if (fabs(newVal - previousEMA) > THRESHOLD) {
        newVal = previousEMA;
    }
    
    // Calculate new EMA: EMA = α * newValue + (1 - α) * previousEMA
    return alpha * newVal + (1.0f - alpha) * previousEMA;
}

//==============================================================================
// TASK FUNCTIONS (Code within tasks now uses defines)
//==============================================================================

//------------------------------------------------------------------------------
// Task: Read MPU9250 and Calculate Orientation (Mahony Filter + Magnetometer)
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
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true); // Wake up
        Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true); // Accel +/- 2g
        Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true); // Gyro +/- 250dps
        Wire.beginTransmission(MPU9250_ADDRESS);  Wire.write(0x1A); Wire.write(0x04); Wire.endTransmission(true); // DLPF_CFG = 4 (gyroscope 20 Hz low pass filter)
        Wire.beginTransmission(MPU9250_ADDRESS);  Wire.write(0x1D); Wire.write(0x04); Wire.endTransmission(true); // A_DLPF_CFG = 4 (accelerometer 20 Hz low pass filter) 
        initMagnetometer();
        xSemaphoreGive(i2cMutex);
    }
    
    lastQuatTime = micros();
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // --- Read Raw Data ---
        int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        float local_mx, local_my, local_mz;
        
        // Protect I2C operations with mutex
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            // Read Accel
            Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(ACCEL_XOUT_H); Wire.endTransmission(false);
            Wire.requestFrom(MPU9250_ADDRESS, 6, true);
            raw_ax = Wire.read() << 8 | Wire.read(); raw_ay = Wire.read() << 8 | Wire.read(); raw_az = Wire.read() << 8 | Wire.read();
            // Read Gyro
            Wire.beginTransmission(MPU9250_ADDRESS); Wire.write(GYRO_XOUT_H); Wire.endTransmission(false);
            Wire.requestFrom(MPU9250_ADDRESS, 6, true);
            raw_gx = Wire.read() << 8 | Wire.read(); raw_gy = Wire.read() << 8 | Wire.read(); raw_gz = Wire.read() << 8 | Wire.read();

            // Read magnetometer data
            readMagnetometer(local_mx, local_my, local_mz);
            xSemaphoreGive(i2cMutex);
        } else {
            // If we can't get the mutex, skip this cycle
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // --- Local calculation variables ---
        float local_ax = raw_ax / 16384.0f;
        float local_ay = raw_ay / 16384.0f;
        float local_az = raw_az / 16384.0f;
        float local_gx = raw_gx / 131.0f;
        float local_gy = raw_gy / 131.0f;
        float local_gz = raw_gz / 131.0f;

        // --- Calculate Angles (Mahony AHRS with Magnetometer) ---
        unsigned long now = micros();
        float dt = (now - lastQuatTime) * 1e-6f; // Delta t in seconds
        lastQuatTime = now;
        if (dt <= 0) dt = 1e-3; // Prevent division by zero or negative dt

        // Convert to proper units and normalize
        float ax_calc = local_ax, ay_calc = local_ay, az_calc = local_az;
        float gx_calc = local_gx * PI/180.0f, gy_calc = local_gy * PI/180.0f, gz_calc = local_gz * PI/180.0f;
        float mx_calc = local_mx, my_calc = local_my, mz_calc = local_mz;

        // Normalize accelerometer measurement
        float norm = sqrt(ax_calc*ax_calc + ay_calc*ay_calc + az_calc*az_calc);
        if (norm > 0.0f) {
           ax_calc /= norm; ay_calc /= norm; az_calc /= norm;
        } else {
             ax_calc = 0; ay_calc = 0; az_calc = 0;
        }

        // Normalize magnetometer measurement
        norm = sqrt(mx_calc*mx_calc + my_calc*my_calc + mz_calc*mz_calc);
        if (norm > 0.0f) {
           mx_calc /= norm; my_calc /= norm; mz_calc /= norm;
        } else {
             mx_calc = 0; my_calc = 0; mz_calc = 0;
        }

        // Reference direction of Earth's magnetic field (normalized)
        float hx = 2.0f * (mx_calc * (0.5f - q2*q2 - q3*q3) + my_calc * (q1*q2 - q0*q3) + mz_calc * (q1*q3 + q0*q2));
        float hy = 2.0f * (mx_calc * (q1*q2 + q0*q3) + my_calc * (0.5f - q1*q1 - q3*q3) + mz_calc * (q2*q3 - q0*q1));
        float bx = sqrt(hx*hx + hy*hy);
        float bz = 2.0f * (mx_calc * (q1*q3 - q0*q2) + my_calc * (q2*q3 + q0*q1) + mz_calc * (0.5f - q1*q1 - q2*q2));

        // Estimated direction of gravity and magnetic field
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        float wx = 2.0f*(bx*(0.5f - q2*q2 - q3*q3) + bz*(q1*q3 - q0*q2));
        float wy = 2.0f*(bx*(q1*q2 - q0*q3) + bz*(q0*q1 + q2*q3));
        float wz = 2.0f*(bx*(q0*q2 + q1*q3) + bz*(0.5f - q1*q1 - q2*q2));

        // Error is sum of cross product between estimated direction and measured direction of field
        float ex = (ay_calc*vz - az_calc*vy) + (my_calc*wz - mz_calc*wy);
        float ey = (az_calc*vx - ax_calc*vz) + (mz_calc*wx - mx_calc*wz);
        float ez = (ax_calc*vy - ay_calc*vx) + (mx_calc*wy - my_calc*wx);

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

        // --- Kalman Filter Update ---
        // Use Mahony filter output as measurements (convert to radians for Kalman filter)
        float mahony_pitch_rad = local_pitch * (M_PI / 180.0f);
        float mahony_roll_rad = local_roll * (M_PI / 180.0f);
        
        // Handle yaw wrapping: convert to radians and normalize to [-π, π]
        float mahony_yaw_rad = local_yaw * (M_PI / 180.0f);
        while (mahony_yaw_rad > M_PI) mahony_yaw_rad -= 2.0f * M_PI;
        while (mahony_yaw_rad < -M_PI) mahony_yaw_rad += 2.0f * M_PI;

        // Get current speed from hall sensors (convert km/h to m/s)
        float current_speed_ms = 0.0f;
        if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
            current_speed_ms = currentSpeedAvg / 3.6f; // Convert km/h to m/s
            xSemaphoreGive(hallDataMutex);
        }

        // Calculate acceleration magnitude for speed estimation
        float acceleration = sqrt(local_ax * local_ax + local_ay * local_ay + local_az * local_az) - 1.0f; // Subtract 1g

        // Gyro rates in rad/s for Kalman filter
        float gyro_x_rad_s = local_gx * (M_PI / 180.0f); // Pitch rate
        float gyro_y_rad_s = local_gy * (M_PI / 180.0f); // Roll rate
        float gyro_z_rad_s = local_gz * (M_PI / 180.0f); // Yaw rate

        // Predict step (uses gyro rates and acceleration)
        kalmanFilter.predict(gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s, acceleration, dt);

        // Update step (uses Mahony filter output and hall sensor speed)
        kalmanFilter.update(mahony_pitch_rad, mahony_roll_rad, mahony_yaw_rad, current_speed_ms);

        // Get filtered values
        float local_kalman_pitch_deg = kalmanFilter.getPitchDeg();
        float local_kalman_roll_deg = kalmanFilter.getRollDeg();
        float local_kalman_yaw_deg = kalmanFilter.getYawDeg();
        float local_kalman_pitch_variance = kalmanFilter.getPitchVariance();
        float local_kalman_roll_variance = kalmanFilter.getRollVariance();
        float local_kalman_yaw_variance = kalmanFilter.getYawVariance();
        float local_kalman_speed_kmh = kalmanFilter.getSpeedKMH();

        // Debug output every 100 cycles (~2 seconds at 50Hz)
            static int debug_counter = 0;
            debug_counter++;
            if (debug_counter >= 100) {
                debug_counter = 0;
                Serial.printf("Kalman: P=%.2f(%.3f) R=%.2f(%.3f) Y=%.2f(%.3f) | DMP: P=%.2f R=%.2f Y=%.2f\n", 
                             local_kalman_pitch_deg, local_kalman_pitch_variance,
                             local_kalman_roll_deg, local_kalman_roll_variance,
                             local_kalman_yaw_deg, local_kalman_yaw_variance,
                             local_pitch, local_roll, local_yaw);
            }

            // --- Update Shared Variables (Protected by Mutex) ---
            if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) {
                pitchAvg = updateEMA(pitchAvg, local_pitch);
                rollAvg = updateEMA(rollAvg, local_roll);
                yawAvg = updateEMA(yawAvg, local_yaw);
                gForceAvg = updateEMA(gForceAvg, local_gForce);

                // Store Kalman filtered values with bounds checking
                // Check if Kalman values are reasonable (not NaN, not extreme)
                if (!isnan(local_kalman_pitch_deg) && abs(local_kalman_pitch_deg) < 180.0f && 
                    local_kalman_pitch_variance > 0.0f && local_kalman_pitch_variance < 10.0f) {
                    kalmanPitchDeg = local_kalman_pitch_deg;
                    kalmanPitchVariance = local_kalman_pitch_variance;
                } else {
                    // Fallback to EMA (which uses DMP filter as input) if Kalman is unreliable
                    kalmanPitchDeg = pitchAvg;
                    kalmanPitchVariance = 10.0f; // High variance to force EMA usage
                }
                
                if (!isnan(local_kalman_roll_deg) && abs(local_kalman_roll_deg) < 180.0f && 
                    local_kalman_roll_variance > 0.0f && local_kalman_roll_variance < 10.0f) {
                    kalmanRollDeg = local_kalman_roll_deg;
                    kalmanRollVariance = local_kalman_roll_variance;
                } else {
                    // Fallback to EMA (which uses DMP filter as input) if Kalman is unreliable
                    kalmanRollDeg = rollAvg;
                    kalmanRollVariance = 10.0f; // High variance to force EMA usage
                }

                if (!isnan(local_kalman_yaw_deg) && local_kalman_yaw_deg >= 0.0f && local_kalman_yaw_deg < 360.0f && 
                    local_kalman_yaw_variance > 0.0f && local_kalman_yaw_variance < 10.0f) {
                    kalmanYawDeg = local_kalman_yaw_deg;
                    kalmanYawVariance = local_kalman_yaw_variance;
                } else {
                    // Fallback to EMA (which uses DMP filter as input) if Kalman is unreliable
                    kalmanYawDeg = yawAvg;
                    kalmanYawVariance = 10.0f; // High variance to force EMA usage
                }

                // Update speed with Kalman filtered value if variance is low
                if (kalmanFilter.getSpeedVariance() < KALMAN_VARIANCE_THRESHOLD) {
                    if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) {
                        // Update direction based on Kalman filter speed
                        hallDirectionForward = (local_kalman_speed_kmh >= 0);
                        currentSpeedAvg = abs(local_kalman_speed_kmh);
                        xSemaphoreGive(hallDataMutex);
                    }
                }

                accelX = local_ax;
                accelY = local_ay;
                accelZ = local_az;
                gyroX = local_gx;
                gyroY = local_gy;
                gyroZ = local_gz;
                magX = local_mx;
                magY = local_my;
                magZ = local_mz;
                xSemaphoreGive(imuDataMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
                    currentSpeedAvg = updateEMA(currentSpeedAvg, local_speed);
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
             inJumpState = true; 
             jumpStartTime = currentMillis;
             Serial.println("Jump state started - G: " + String(local_gForce, 2));
        }
        
        if (inJumpState && local_gForce > local_landingThreshold) {
             unsigned long jumpDuration = currentMillis - jumpStartTime;
             if (jumpDuration > JUMP_DURATION_MIN) {
                 local_jumpDetected_this_cycle = true; 
                 lastJumpTime = currentMillis;
                 if(!current_jump_state) { 
                     Serial.println("JUMP DETECTED! Duration: " + String(jumpDuration) + "ms G: " + String(local_gForce, 2)); 
                 }
             } else {
                 Serial.println("Jump too short - Duration: " + String(jumpDuration) + "ms (min: " + String(JUMP_DURATION_MIN) + "ms)");
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
            // Kalman values to read
            float local_kalmanPitch = 0.0f, local_kalmanRoll = 0.0f;
            // Note: Thresholds are not currently sent over BLE, but could be added if needed

             if (xSemaphoreTake(hallDataMutex, portMAX_DELAY) == pdTRUE) { local_speed = currentSpeedAvg; local_hallDir = hallDirectionForward; xSemaphoreGive(hallDataMutex); } else { vTaskDelay(1); continue; }
            if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { 
                // EMA Values
                float ema_pitch = pitchAvg;
                float ema_roll = rollAvg;
                float ema_yaw = yawAvg;

                // Kalman Values for pitch, roll, and yaw
                float kf_pitch = kalmanPitchDeg;
                float kf_roll = kalmanRollDeg;
                float kf_yaw = kalmanYawDeg;
                float kf_pitch_variance = kalmanPitchVariance;
                float kf_roll_variance = kalmanRollVariance;
                float kf_yaw_variance = kalmanYawVariance;

                // Decide whether to use Kalman or EMA based on variance
                if (kf_pitch_variance < KALMAN_VARIANCE_THRESHOLD) {
                    local_pitch = kf_pitch; // Use Kalman pitch
                } else {
                    local_pitch = ema_pitch; // Use EMA pitch
                }

                if (kf_roll_variance < KALMAN_VARIANCE_THRESHOLD) {
                    local_roll = kf_roll; // Use Kalman roll
                } else {
                    local_roll = ema_roll; // Use EMA roll
                }

                if (kf_yaw_variance < KALMAN_VARIANCE_THRESHOLD) {
                    local_yaw = kf_yaw; // Use Kalman yaw
                } else {
                    local_yaw = ema_yaw; // Use EMA yaw
                }
        
                local_gForce = gForceAvg; 
                xSemaphoreGive(imuDataMutex); 
            } else { vTaskDelay(1); continue; }
             if (xSemaphoreTake(eventDataMutex, portMAX_DELAY) == pdTRUE) { local_jump = jumpDetected; local_drop = dropDetected; local_imuDir = imuDirectionForward; local_imuState = imuSpeedState; xSemaphoreGive(eventDataMutex); } else { vTaskDelay(1); continue; }
             
             // *** FIX: Read offset values from offsetMutex ***
             if (xSemaphoreTake(offsetMutex, portMAX_DELAY) == pdTRUE) { 
                 local_pitchOffset = pitchOffset; 
                 local_rollOffset = rollOffset; 
                 local_yawOffset = yawOffset; 
                 xSemaphoreGive(offsetMutex); 
             } else { vTaskDelay(1); continue; }

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
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0); display.println("Music Bike RTOS"); display.println("Initializing...");
        display.display();
        xSemaphoreGive(i2cMutex);
    }
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
         if (xSemaphoreTake(imuDataMutex, portMAX_DELAY) == pdTRUE) { 
            // EMA Value for pitch
            float ema_pitch = pitchAvg;

            // Kalman Values for pitch
            float kf_pitch = kalmanPitchDeg;
            float kf_pitch_variance = kalmanPitchVariance;

            // Decide whether to use Kalman or EMA based on variance
            if (kf_pitch_variance < KALMAN_VARIANCE_THRESHOLD) {
                local_pitch = kf_pitch; // Use Kalman pitch
            } else {
                local_pitch = ema_pitch; // Use EMA pitch
            }
            
            local_gForce = gForceAvg; 
            /*Removed roll, yaw reads*/ 
            xSemaphoreGive(imuDataMutex); 
        } else { vTaskDelay(1); continue; }
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
         if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
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
             xSemaphoreGive(i2cMutex);
         }

         vTaskDelayUntil(&xLastWakeTime, xFrequency);
     }
 }


//==============================================================================
// SETUP FUNCTION
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

    // --- Create Mutexes ---
    imuDataMutex = xSemaphoreCreateMutex();
    hallDataMutex = xSemaphoreCreateMutex();
    eventDataMutex = xSemaphoreCreateMutex();
    offsetMutex = xSemaphoreCreateMutex();
    bleConnectionMutex = xSemaphoreCreateMutex();
    configMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();

    if (!imuDataMutex || !hallDataMutex || !eventDataMutex || !offsetMutex || !bleConnectionMutex || !configMutex || !i2cMutex ) {
        Serial.println("Failed to create mutexes!"); for(;;);
    }

    // --- Initialize OLED Display ---
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed")); for(;;);
        }
        display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0); display.println("Starting RTOS..."); display.display();
        xSemaphoreGive(i2cMutex);
    }
    delay(500);

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

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay(); display.setCursor(0,0);
        display.println("BLE Advertising!"); display.println("Creating tasks...");
        display.display();
        xSemaphoreGive(i2cMutex);
    }
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
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        display.clearDisplay(); display.setCursor(0,0); display.println("Tasks Running!"); display.display();
        xSemaphoreGive(i2cMutex);
    }
}

//==============================================================================
// LOOP FUNCTION (Empty)
//==============================================================================
void loop() {
      vTaskDelay(pdMS_TO_TICKS(1000));
}

