/*
 * DEPRECATED: This version (v0.3) is no longer maintained.
 * Please use Bike_Sensor_Filtered_IMU.ino for the current implementation.
 * 
 * This file remains for reference purposes only.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
#define HALL_SENSOR_PIN    5
#define SDA_PIN            8
#define SCL_PIN            9
#define ZERO_BUTTON_PIN    7  // Button for zeroing

// MPU9250 registers
#define MPU9250_ADDRESS   0x68
#define ACCEL_XOUT_H      0x3B
#define GYRO_XOUT_H       0x43

// Physical constants
#define WHEEL_DIAMETER_INCHES   26.0
#define WHEEL_CIRCUMFERENCE_CM  (WHEEL_DIAMETER_INCHES * 2.54 * 3.14159)
#define HALF_CIRCUMFERENCE_CM   (WHEEL_CIRCUMFERENCE_CM / 2.0)

// Thresholds for jump/drop detection
#define JUMP_THRESHOLD       1.5
#define JUMP_DURATION_MIN    100      // milliseconds
#define LANDING_THRESHOLD    1.8
#define DROP_THRESHOLD       0.0
#define DIRECTION_THRESHOLD  0.3

// Timing constants
const int debounceDelay = 50;
const unsigned long SPEED_TIMEOUT = 3000;
const unsigned long EVENT_DISPLAY_DURATION = 2000;

// Global sensor variables (raw and filtered)
static float pitch = 0.0, roll = 0.0, yaw = 0.0;
static float pitchOffset = 0.0, rollOffset = 0.0, yawOffset = 0.0;
static int hallSensorValue = 0, lastHallSensorValue = HIGH;
static unsigned long lastTriggerTime = 0, currentTriggerTime = 0;
static float currentSpeed = 0.0;
static bool movingForward = true;
static float forwardAccel = 0.0;
static bool inJumpState = false, jumpDetected = false, dropDetected = false;
static unsigned long jumpStartTime = 0, lastJumpTime = 0, lastDropTime = 0;

// Sensor raw readings
static int16_t ax, ay, az;
static int16_t gx, gy, gz;
static float accelX, accelY, accelZ;
static float gyroX, gyroY, gyroZ;

// High if accelerometer reset requested
volatile bool zeroResetRequested = false;

// Complementary filter variables
static unsigned long prevTime = 0;
static float alpha = 0.96;

static uint8_t ble_data[20]; // Buffer for BLE data
static uint16_t conn_handle;
static bool notify_enabled;

// Define a struct to hold sensor data for other tasks.
typedef struct {
    float pitch;
    float roll;
    float yaw;
    float accelZ;
    float currentSpeed;
    bool jumpDetected;
    bool dropDetected;
    bool movingForward;
} SensorData;

// Create two queues (one for display updates and one for Serial output)
// Using a length of 1 with xQueueOverwrite so that each task always reads the latest data.
QueueHandle_t displayQueue;
QueueHandle_t serialQueue;
QueueHandle_t bluetoothQueue;

//-------------------------- Function Prototypes --------------------------
void readMPU9250Data();
void calculateAngles();
void detectJumpAndDrop();
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void SerialTask(void *pvParameters);
void BluetoothTask(void *pvParameters);


//-------------------------- Sensor Reading Functions --------------------------
void detectJumpAndDrop() {
    // Vertical acceleration (may need to adjust axis based on sensor orientation)
    float verticalAccel = accelZ - 1.0;  // Subtract 1g to get deviation from gravity

    // Jump detection - Start of jump (weightlessness)
    if (!inJumpState && verticalAccel < JUMP_THRESHOLD) {
        inJumpState = true;
        jumpStartTime = millis();
    }

    // End of jump (landing impact)
    if (inJumpState && verticalAccel > LANDING_THRESHOLD) {
        unsigned long jumpDuration = millis() - jumpStartTime;

        // Only count as jump if duration exceeds minimum
        if (jumpDuration > JUMP_DURATION_MIN) {
            jumpDetected = true;
            lastJumpTime = millis();
            Serial.println("JUMP DETECTED! Duration: " + String(jumpDuration) + "ms");
            Serial.println("Landing G-force: " + String(verticalAccel + 1.0) + "g");
        }

        inJumpState = false;
    }

    // Drop detection (strong impact without prior weightlessness)
    if (!inJumpState && verticalAccel > DROP_THRESHOLD) {
        dropDetected = true;
        lastDropTime = millis();
        Serial.println("DROP DETECTED! Impact G-force: " + String(verticalAccel + 1.0) + "g");
    }

    // Reset jump indicator after display time
    if (jumpDetected && (millis() - lastJumpTime > EVENT_DISPLAY_DURATION)) {
        jumpDetected = false;
    }

    // Reset drop indicator after display time
    if (dropDetected && (millis() - lastDropTime > EVENT_DISPLAY_DURATION)) {
        dropDetected = false;
    }
}

void readMPU9250Data() {
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 6, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();

    // Convert raw values to g's
    accelX = ax / 16384.0;
    accelY = ay / 16384.0;
    accelZ = az / 16384.0;

    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 6, true);

    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    // Convert raw values to degrees/sec
    gyroX = gx / 131.0;
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;
}

void calculateAngles() {
    // Calculate pitch and roll from accelerometer (in degrees)
    float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    float accelRoll = atan2(-accelX, accelZ) * 180.0 / PI;

    // Get time delta
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    // Calculate gyro angles
    float gyroPitch = pitch + gyroX * dt;
    float gyroRoll = roll + gyroY * dt;
    float gyroYaw = yaw + gyroZ * dt;

    // Apply complementary filter
    pitch = alpha * gyroPitch + (1.0 - alpha) * accelPitch;
    roll = alpha * gyroRoll + (1.0 - alpha) * accelRoll;
    yaw = gyroYaw; // No magnetometer correction for yaw

    // Keep yaw between 0-360
    if (yaw < 0) yaw += 360;
    if (yaw >= 360) yaw -= 360;
}

void updateDisplay(SensorData* data) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Display G-force reading from sensor data structure
    display.setCursor(0, 0);
    display.print("G:");
    display.println(data->accelZ, 2);

    // Jump and drop indicators from sensor data structure
    display.setCursor(0, 10);
    display.print("Jump:");
    display.println(data->jumpDetected ? "YES!" : "No");

    display.setCursor(64, 10);
    display.print("Drop:");
    display.println(data->dropDetected ? "YES!" : "No");

    // Orientation values (pitch, roll, yaw)
    display.setCursor(0, 25);
    display.print("P:");
    display.print(data->pitch, 1);

    display.setCursor(64, 25);
    display.print("R:");
    display.println(data->roll, 1);

    display.setCursor(0, 35);
    display.print("Y:");
    display.println(data->yaw, 1);

    // Speed and direction indicators
    display.setCursor(0, 45);
    display.print("Spd:");
    display.print(data->currentSpeed, 1);

    display.setCursor(64, 45);
    display.print("Dir:");
    display.println(data->movingForward ? "Fwd" : "Rev");

    display.display();
}

void printSerialData(float p, float r, float y) {
    Serial.println("--- Sensor Data ---");
    Serial.print("Pitch: "); Serial.print(p, 2);
    Serial.print(" Roll: "); Serial.print(r, 2);
    Serial.print(" Yaw: "); Serial.println(y, 2);

    Serial.print("Speed: "); Serial.print(currentSpeed, 2); Serial.println(" km/h");
    Serial.print("Direction: "); Serial.println(movingForward ? "Forward" : "Reverse");
    Serial.print("Hall Sensor: ");
    Serial.println(hallSensorValue == LOW ? "Magnet Detected" : "No Magnet");

    Serial.print("G-Force: "); Serial.println(accelZ, 2);
    Serial.print("Vertical Accel: "); Serial.println(accelZ - 1.0, 2);
    Serial.print("Forward Accel: "); Serial.println(forwardAccel, 2);

    if (jumpDetected) Serial.println("JUMP DETECTED!");
    if (dropDetected) Serial.println("DROP DETECTED!");

    Serial.println();
}

// Helper function to format sensor data for BT transmission
String formatBluetoothData(const SensorData& data) {
    return String("") + data.pitch + " " + data.roll + " " + data.yaw +
           " " + data.currentSpeed + " " + data.accelZ +
           " " + data.jumpDetected + " " + data.dropDetected;
}

//-------------------------- FreeRTOS Task Functions --------------------------
void SensorTask(void *pvParameters) {
    // Structure to hold sensor data
    SensorData sensorData;

    // Local variables for hall sensor speed calculation
    int lastHallValue = HIGH;
    unsigned long lastTriggerTime = millis(); // Initialize trigger time
    unsigned long currentTriggerTime = 0;

    // FreeRTOS task loop for sensor polling
    while (true) {
        // Read and update MPU9250 sensor data (accelerometer and gyroscope)
        readMPU9250Data();  // Retrieves raw sensor values via I2C [1]
        calculateAngles();  // Applies a complementary filter for pitch, roll, yaw calculation [1]
        detectJumpAndDrop(); // Calculates jump/drop events based on vertical acceleration [1]

        // Process hall sensor input for speed calculation
        int currentHallValue = digitalRead(HALL_SENSOR_PIN);
        if (currentHallValue == LOW && lastHallValue == HIGH) {
            // Falling edge detected: magnet passes the sensor
            currentTriggerTime = millis();
            if (lastTriggerTime > 0) {
                unsigned long timeDiff = currentTriggerTime - lastTriggerTime;
                // Calculate speed in km/h using HALF_CIRCUMFERENCE_CM and time difference
                sensorData.currentSpeed = (HALF_CIRCUMFERENCE_CM / timeDiff) * 36.0;
            }
            lastTriggerTime = currentTriggerTime;
        }
        lastHallValue = currentHallValue;
        // If no hall trigger has occurred within SPEED_TIMEOUT, assume speed is zero
        if (millis() - lastTriggerTime > SPEED_TIMEOUT) {
            sensorData.currentSpeed = 0.0;
        }

        // Populate sensor data structure with processed values and adjusted angles
        sensorData.pitch = pitch - pitchOffset;
        sensorData.roll  = roll  - rollOffset;
        sensorData.yaw   = yaw   - yawOffset;
        sensorData.accelZ = accelZ;
        sensorData.jumpDetected = jumpDetected;
        sensorData.dropDetected = dropDetected;
        sensorData.movingForward = movingForward;

        // Send the updated sensor data to other tasks via queues
        // Using xQueueOverwrite ensures only the most recent data is maintained in each queue
        xQueueOverwrite(displayQueue, &sensorData);
        xQueueOverwrite(serialQueue, &sensorData);
        xQueueSend(bluetoothQueue, &sensorData, 0);

        // Wait for 10ms to achieve roughly a 100Hz sensor polling rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void DisplayTask(void *pvParameters) {
    SensorData sensorData;
    // Use a periodic delay to update the display at about 10Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms interval
    while (true) {
        // Peek the latest sensor data from the queue (non-blocking)
        if (xQueuePeek(displayQueue, &sensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
            updateDisplay(&sensorData);
        }
        // Wait until the next update cycle
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void SerialTask(void *pvParameters) {
    SensorData sensorData; // Structure containing sensor readings
    // Serial update interval set to 1000 ms (1 Hz update rate)
    const TickType_t serialInterval = pdMS_TO_TICKS(1000);

    while (true) {
        // Attempt to get the most recent sensor data from the serial queue
        if (xQueuePeek(serialQueue, &sensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Call the provided helper function with pitch, roll, and yaw
            printSerialData(sensorData.pitch, sensorData.roll, sensorData.yaw);
        }
        // Delay to maintain a 1-second periodic update rate for serial transmission
        vTaskDelay(serialInterval);
    }
}

// Bluetooth Task (runs on core 0)
void BluetoothTask(void *pvParameters) {
    // Create a BLE server and set the device name if not already done in setup()
    NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEServer* pServer = NimBLEDevice::createServer();

    // Create a BLE service using your service UUID (use a 128-bit UUID string in standard format)
    NimBLEService* pService = pServer->createService("020012ac-4202-78b8-ed11-da4642c6bbb2");

    // Create a characteristic within that service with read, write, and notify properties
    NimBLECharacteristic* pCharacteristic = pService->createCharacteristic(
            "020012ac-4202-78b8-ed11-de46769cafc9",
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );

    // Set an initial value for the characteristic
    pCharacteristic->setValue("Hello World");
    pService->start();

    // Start advertising your BLE service so that devices can discover it
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("Music Bike");
    pAdvertising->start();

    while (true) {
        SensorData btData;
        // Wait indefinitely for new sensor data from the bluetoothQueue
        if (xQueueReceive(bluetoothQueue, &btData, portMAX_DELAY) == pdTRUE) {
            // Format the sensor data into a string message
            String data = formatBluetoothData(btData);

            pCharacteristic->setValue(data.c_str());
            pCharacteristic->notify();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Add this task function
void zeroResetTask(void *pvParameters) {
    while (true) {
        // Wait for button press (atomic read)
        if (zeroResetRequested) {
            zeroResetRequested = false;
            pitchOffset = pitch;
            rollOffset = roll;
            yawOffset = yaw;
            Serial.println("Reset pitch/roll/yaw.");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Interrupt Service Routine for accelerometer reset
void IRAM_ATTR zeroButtonISR() {
    zeroResetRequested = true;
}


void setup() {
    // Initialize Serial
    Serial.begin(115200);

    // Initialize Hall Effect Sensor and Zero Button
    pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ZERO_BUTTON_PIN), zeroButtonISR, FALLING);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize MPU9250
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Wake up MPU-9250
    Wire.endTransmission(true);

    // Configure accelerometer sensitivity (±2g)
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Initialize OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    // Display initialization message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Music Bike v0.3");
    display.println("FreeRTOS Enabled");
    display.println("BLE: Music Bike");
    display.display();
    delay(2000);
    // Initialize BLE
    NimBLEDevice::init("Music Bike");
    NimBLEDevice::setDeviceName("Music Bike");
    // Create FreeRTOS queues (add to existing queue definitions)
    displayQueue = xQueueCreate(1, sizeof(SensorData));
    serialQueue = xQueueCreate(1, sizeof(SensorData));
    bluetoothQueue = xQueueCreate(5, sizeof(SensorData));

    // Create tasks with core assignments
    xTaskCreatePinnedToCore(
            SensorTask,      // Task function
            "SensorTask",    // Task name
            4096,            // Stack size (bytes)
            NULL,            // Parameters
            4,               // Priority (5=highest)
            NULL,            // Task handle
            1                // Core 1 (Sensor/Display/Serial tasks)
    );
    // Create task to read reset button for accelerometer
    xTaskCreatePinnedToCore(
            zeroResetTask,    // Task function
            "ZeroReset",      // Task name
            2048,             // Stack size
            NULL,             // Parameters
            5,                // Priority
            NULL,             // Task handle
            1                 // Core 1
    );
    xTaskCreatePinnedToCore(
            DisplayTask,
            "DisplayTask",
            2048,
            NULL,
            3,               // Medium priority
            NULL,
            1                // Core 1
    );
    xTaskCreatePinnedToCore(
            SerialTask,
            "SerialTask",
            2048,
            NULL,
            3,
            NULL,
            1
    );
    xTaskCreatePinnedToCore(
            BluetoothTask,
            "BluetoothTask",
            4096,
            NULL,
            2,               // Lower priority
            NULL,
            0                // Core 0 (BT/WiFi stack)
    );
    Serial.println("FreeRTOS tasks initialized");
    Serial.println("Core 0: Bluetooth");
    Serial.println("Core 1: Sensors/Display/Serial");

    // Initialize timing variables
    prevTime = millis();
    lastTriggerTime = millis();
}

// FreeRTOS doesn't use loop
void loop() {}
