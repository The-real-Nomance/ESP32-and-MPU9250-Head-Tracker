// FUCK THIS CODING LANGUAGE I SPENT 20 Hours of my life on this stupid code just so i can get head tracking
// You can use this code as you like you could copy and paste it and call it yours i couldnt give a fuck
// if you do change this code and it stops working dont call me please there is a chance if i do see this code again i might kill my self
// now if you make this code better also dont call me cause ill kill YOU just edit it for your self cause if i figure out who made my code better ill kill us both
// enjoy this shitshow of a code and i hop all the best while you suffer
// 



#include <EEPROM.h>
#include "MPU9250.h"

#define DEBUG true
#define EEPROM_OFFSET 0
#define UPDATE_RATE 100 

MPU9250 mpu;

struct {
    int16_t begin;    
    uint16_t cpt;     
    float gyro[3];    
    float acc[3];     
    int16_t end;      
} hat;

const uint8_t EEPROM_SIZE = 0x80;

enum EEP_ADDR {
    EEP_CALIB_FLAG = EEPROM_OFFSET + 0x00,
    EEP_ACC_BIAS   = EEPROM_OFFSET + 0x01,
    EEP_GYRO_BIAS  = EEPROM_OFFSET + 0x0D
};

#define LED_PIN 2

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 1000 / UPDATE_RATE;

float yaw_offset = 0, pitch_offset = 0, roll_offset = 0;
float yaw = 0, pitch = 0, roll = 0;
unsigned long lastTime = 0;

// ================================
// YAW DRIFT - ADJUST THIS VALUE  =          
// ================================
// If yaw drifts POSITIVE make this NEGATIVE
// If yaw drifts NEGATIVE make this POSITIVE
// Start with small values: ±0.001, ±0.005, ±0.01, etc.
#define YAW_DRIFT_COMPENSATION -0.08648  // Adjust this value until drift stops    <---------THIS TOOK SO FUCKING LONG TO FIGURE OUT IT WAS ONE OF THE WORST MOMMNETS IN MY WHOLE LIFE I WANNA FUCKING DIEEEEEEEEEEE


#define FILTER_SIZE 5
#define COMPLEMENTARY_ALPHA 0.96f
#define SMOOTHING_FACTOR 0.15f


float yawHistory[FILTER_SIZE] = {0};
float pitchHistory[FILTER_SIZE] = {0};
float rollHistory[FILTER_SIZE] = {0};
int historyIndex = 0;


float smoothYaw = 0, smoothPitch = 0, smoothRoll = 0;


#define MPU9250_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43


float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

byte readByte(int address) {
    byte valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

float readFloat(int address) {
    float valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

bool isCalibrated() {
    return (readByte(EEP_CALIB_FLAG) == 0x01);
}

void saveCalibration() {
    EEPROM.put(EEP_CALIB_FLAG, 1);
    EEPROM.put(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    EEPROM.put(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    EEPROM.put(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    EEPROM.put(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    EEPROM.put(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    EEPROM.put(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    EEPROM.commit();
    
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
}

void loadCalibration() {
    if (isCalibrated()) {
        mpu.setAccBias(
            readFloat(EEP_ACC_BIAS + 0),
            readFloat(EEP_ACC_BIAS + 4),
            readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(
            readFloat(EEP_GYRO_BIAS + 0),
            readFloat(EEP_GYRO_BIAS + 4),
            readFloat(EEP_GYRO_BIAS + 8));
        
        gyroBiasX = readFloat(EEP_GYRO_BIAS + 0);
        gyroBiasY = readFloat(EEP_GYRO_BIAS + 4);
        gyroBiasZ = readFloat(EEP_GYRO_BIAS + 8);
        
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        
        Serial.println("Calibration loaded from EEPROM");
    }
}

bool initMPU9250() {
    Wire.beginTransmission(MPU9250_ADDR);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(100);
    
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
    
    return true;
}

void readSensorData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU9250_ADDR, 14);
    if (Wire.available() >= 14) {
        *ax = (Wire.read() << 8) | Wire.read();
        *ay = (Wire.read() << 8) | Wire.read();
        *az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();
        *gx = (Wire.read() << 8) | Wire.read();
        *gy = (Wire.read() << 8) | Wire.read();
        *gz = (Wire.read() << 8) | Wire.read();
    }
}

void calculateAnglesFromRaw() {
    int16_t ax, ay, az, gx, gy, gz;
    readSensorData(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accX = ax / 16384.0f;
    float accY = ay / 16384.0f;
    float accZ = az / 16384.0f;
    
    float gyroX = (gx - gyroBiasX) / 131.0f;
    float gyroY = (gy - gyroBiasY) / 131.0f;
    float gyroZ = (gz - gyroBiasZ) / 131.0f + YAW_DRIFT_COMPENSATION; // Drift fix applied here
    
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.1f || dt < 0.001f) dt = 0.01f;
    lastTime = now;
    
    float accRoll = atan2f(accY, accZ) * 180.0f / PI;
    float accPitch = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * 180.0f / PI;
    
    float rawRoll = COMPLEMENTARY_ALPHA * (roll + gyroX * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accRoll;
    float rawPitch = COMPLEMENTARY_ALPHA * (pitch + gyroY * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accPitch;
    float rawYaw = yaw + gyroZ * dt;
    
    yawHistory[historyIndex] = rawYaw;
    pitchHistory[historyIndex] = rawPitch;
    rollHistory[historyIndex] = rawRoll;
    historyIndex = (historyIndex + 1) % FILTER_SIZE;
    
    float sumYaw = 0, sumPitch = 0, sumRoll = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sumYaw += yawHistory[i];
        sumPitch += pitchHistory[i];
        sumRoll += rollHistory[i];
    }
    
    float avgYaw = sumYaw / FILTER_SIZE;
    float avgPitch = sumPitch / FILTER_SIZE;
    float avgRoll = sumRoll / FILTER_SIZE;
    
    smoothYaw = smoothYaw * (1.0f - SMOOTHING_FACTOR) + avgYaw * SMOOTHING_FACTOR;
    smoothPitch = smoothPitch * (1.0f - SMOOTHING_FACTOR) + avgPitch * SMOOTHING_FACTOR;
    smoothRoll = smoothRoll * (1.0f - SMOOTHING_FACTOR) + avgRoll * SMOOTHING_FACTOR;
    
    yaw = smoothYaw;
    pitch = smoothPitch;
    roll = smoothRoll;
    
    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
    
    if (pitch > 90.0f) pitch = 90.0f;
    if (pitch < -90.0f) pitch = -90.0f;
    
    if (roll > 180.0f) roll -= 360.0f;
    if (roll < -180.0f) roll += 360.0f;
}

void calibrate() {
    Serial.println("\n=== Calibration Started ===");
    Serial.println("Place device on level surface and keep PERFECTLY STILL");
    digitalWrite(LED_PIN, HIGH);
    
    for (int i = 5; i > 0; i--) {
        Serial.printf("Starting in %d seconds...\n", i);
        delay(1000);
    }
    
    Serial.println("Calibrating - DO NOT MOVE!");
    
    const int numSamples = 1000;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    float sumAx = 0, sumAy = 0, sumAz = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        readSensorData(&ax, &ay, &az, &gx, &gy, &gz);
        
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        
        delay(2);
        
        if (i % 100 == 0) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }
    
    gyroBiasX = sumGx / numSamples;
    gyroBiasY = sumGy / numSamples;
    gyroBiasZ = sumGz / numSamples;
    
    float accBiasX = sumAx / numSamples;
    float accBiasY = sumAy / numSamples;
    float accBiasZ = sumAz / numSamples - 16384.0f;
    
    mpu.setGyroBias(gyroBiasX, gyroBiasY, gyroBiasZ);
    mpu.setAccBias(accBiasX, accBiasY, accBiasZ);
    
    Serial.println("\n✓ Calibration complete!");
    Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.printf("Current drift compensation: %.4f\n", YAW_DRIFT_COMPENSATION);
    
    Serial.println("Saving to EEPROM...");
    saveCalibration();
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        yawHistory[i] = 0;
        pitchHistory[i] = 0;
        rollHistory[i] = 0;
    }
    smoothYaw = smoothPitch = smoothRoll = 0;
    yaw = pitch = roll = 0;
    
    digitalWrite(LED_PIN, LOW);
    Serial.println("=== Calibration Complete ===\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║   ESP32 MPU9250 Head Tracker        ║");
    Serial.println("║   With Yaw Drift Compensation       ║");
    Serial.println("╚══════════════════════════════════════╝");
    
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(100);
    
    EEPROM.begin(EEPROM_SIZE);
    
    Serial.print("Initializing MPU9250");
    
    bool initSuccess = false;
    for (int i = 0; i < 3; i++) {
        Serial.print(".");
        if (initMPU9250()) {
            initSuccess = true;
            break;
        }
        delay(500);
    }
    
    if (!initSuccess) {
        Serial.println("\nMPU9250 initialization failed!");
        while(1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }
    
    Serial.println(".");
    
    mpu.setup(MPU9250_ADDR);
    loadCalibration();
    
    hat.begin = 0xAAAA;
    hat.cpt = 0;
    hat.end = 0x5555;
    hat.acc[0] = hat.acc[1] = hat.acc[2] = 0;
    
    digitalWrite(LED_PIN, LOW);
    lastTime = micros();
    
    Serial.println("\nREADY!");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Commands:");
    Serial.println("  [c] - Calibrate sensors");
    Serial.println("  [z] - Zero/center position");
    Serial.println("  [r] - Reset frame counter");
    Serial.println("  [p] - Print angles");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("Current yaw drift compensation: %.4f\n", YAW_DRIFT_COMPENSATION);
    Serial.println("If yaw drifts positive, make value MORE NEGATIVE");
    Serial.println("If yaw drifts negative, make value MORE POSITIVE");
}

void loop() {
    calculateAnglesFromRaw();
    
    if (Serial.available()) {
        char input = Serial.read();
        switch(input) {
            case 'c':
            case 'C':
                calibrate();
                break;
                
            case 'r':
            case 'R':
                hat.cpt = 0;
                Serial.println("Counter reset");
                break;
                
            case 'z':
            case 'Z':
                yaw_offset = yaw;
                pitch_offset = pitch;
                roll_offset = roll;
                for (int i = 0; i < FILTER_SIZE; i++) {
                    yawHistory[i] = yaw;
                    pitchHistory[i] = pitch;
                    rollHistory[i] = roll;
                }
                smoothYaw = yaw;
                smoothPitch = pitch;
                smoothRoll = roll;
                Serial.println("Zero position set");
                break;
                
            case 'p':
            case 'P':
                Serial.printf("Yaw: %7.2f°  Pitch: %7.2f°  Roll: %7.2f°  (Drift compensation: %.4f)\n", 
                             yaw - yaw_offset, pitch - pitch_offset, roll - roll_offset, 
                             YAW_DRIFT_COMPENSATION);
                break;
        }
    }
    
    unsigned long now = millis();
    if (now - lastUpdate >= updateInterval) {
        hat.gyro[0] = yaw - yaw_offset;
        hat.gyro[1] = pitch - pitch_offset;
        hat.gyro[2] = roll - roll_offset;
        
        Serial.write((byte*)&hat, sizeof(hat));
        
        hat.cpt = (hat.cpt + 1) % 1000;
        lastUpdate = now;
    }
}