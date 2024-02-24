#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Arduino_JSON.h>

#include <Adafruit_LPS35HW.h>

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
// For SPI mode, we need a CS pin
#define LPS_CS  10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LPS_SCK  13
#define LPS_MISO 12
#define LPS_MOSI 11


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(57600);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-156);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-14);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(-2519);
    mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        /* Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
        */

        mpu.dmpGetGravity(&gravity, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
        /*Serial.print("aworld\t");
        Serial.print(aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
        Serial.print("\t");
        Serial.print(aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
        Serial.print("\t");
        Serial.println(aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
        */

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetGyro(&gg, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
        /* Serial.print("ggWorld\t");
        Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
        Serial.print("\t");
        Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
        Serial.print("\t");
        Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);
        */

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        /*Serial.print("ypr\t");
        Serial.print(ypr[0] * RAD_TO_DEG);
        Serial.print("\t");
        Serial.print(ypr[1] * RAD_TO_DEG);
        Serial.print("\t");
        Serial.println(ypr[2] * RAD_TO_DEG);
        
        Serial.println();
        */

        // encode all data satisfying ros sensor_msgs/Imu.msg into json string
        //Serial.println("{\"orientation\":{\"w\":" + String(q.w) + ",\"x\":" + String(q.x) + ",\"y\":" + String(q.y) + ",\"z\":" + String(q.z) + "},\"angular_velocity\":{\"x\":" + String(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD) + ",\"y\":" + String(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD) + ",\"z\":" + String(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD) + "},\"linear_acceleration\":{\"x\":" + String(aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) + ",\"y\":" + String(aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) + ",\"z\":" + String(aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2) + "}}");

        // data to send
        /*
        {
            "imu": {
                "orientation": {
                "w": "",
                "x": "",
                "y": "",
                "z": ""
                },
                "angular_velocity": {
                "x": "",
                "y": "",
                "z": ""
                },
                "linear_acceleration": {
                "x": "",
                "y": "",
                "z": ""
                }
            },
            "pressure": {
                "temperature": "",
                "pressure": ""
            }
        }
        */
        float imu_orientation_w = q.w;
        float imu_orientation_x = q.x;
        float imu_orientation_y = q.y;
        float imu_orientation_z = q.z;

        float imu_angular_velocity_x = ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
        float imu_angular_velocity_y = ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
        float imu_angular_velocity_z = ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;

        float imu_linear_acceleration_x = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
        float imu_linear_acceleration_y = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
        float imu_linear_acceleration_z = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

        float pressure_temperature = lps35hw.readTemperature();
        float pressure_pressure = lps35hw.readPressure();

        JSONVar data;
        data["imu"]["orientation"]["w"] = imu_orientation_w;
        data["imu"]["orientation"]["x"] = imu_orientation_x;
        data["imu"]["orientation"]["y"] = imu_orientation_y;
        data["imu"]["orientation"]["z"] = imu_orientation_z;
        data["imu"]["angular_velocity"]["x"] = imu_angular_velocity_x;
        data["imu"]["angular_velocity"]["y"] = imu_angular_velocity_y;
        data["imu"]["angular_velocity"]["z"] = imu_angular_velocity_z;
        data["imu"]["linear_acceleration"]["x"] = imu_linear_acceleration_x;
        data["imu"]["linear_acceleration"]["y"] = imu_linear_acceleration_y;
        data["imu"]["linear_acceleration"]["z"] = imu_linear_acceleration_z;
        data["pressure"]["temperature"] = pressure_temperature;
        data["pressure"]["pressure"] = pressure_pressure;

        String jsonString = JSON.stringify(data);
        Serial.println(jsonString);

        delay(20);
    }
}
