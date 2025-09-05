#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <CytronMotorDriver.h>
#include "USBHost_t36.h"

// Joystick setup
USBHost myusb;
USBHIDParser hid1(myusb);
JoystickController joystick(myusb);
// BluetoothController bluet(myusb);
BluetoothController bluet(myusb, true, "0000");

// Motor drivers
CytronMD motor1(PWM_DIR, 3, 5);
CytronMD motor2(PWM_DIR, 22, 20);
CytronMD motor3(PWM_DIR, 23, 21);
CytronMD motor4(PWM_DIR, 2, 4);

// Encoders
Encoder myEnc[4] = {Encoder(7, 6), Encoder(11, 10), Encoder(16, 17), Encoder(14, 15)};

// IMU Setup (BNO055)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Parameters
const float encoderResolution = 4784;
const float wheel_diameter = 6.35;  // cm
const float wheel_radius = wheel_diameter / 2.0;
const float wheels_y_distance = 51.0;  // cm
const float sampling_time = 0.075;  // 75ms
const float max_rpm = 100;

// Transformation matrix
const float transformationMatrix[4][3] = {
    {  0,  1, 0.255 },  // Front-left
    { -1,  0, 0.255 },  // Front-right
    {  0, -1, 0.255 },  // Rear-left
    {  1,  0, 0.255 }   // Rear-right
};

// Odometry variables
float x_pos = 0, y_pos = 0, theta = 0;
long lastCount[4] = {0, 0, 0, 0};
float rpm[4] = {0.0, 0.0, 0.0, 0.0};

// Joystick values
int x = 0, y = 0, rotation = 0;

// Timer
IntervalTimer timer;

void updateOdometryAndIK() {
    myusb.Task();

    // Read joystick inputs
    if (joystick.available()) {
        int temp_x = map(joystick.getAxis(2), 0, 255, -60, 60);
        int temp_y = map(joystick.getAxis(5), 0, 255, 60, -60);
        int temp_rotation = map(joystick.getAxis(0), 0, 255, -10, 10);

        if (abs(temp_x) < 5) temp_x = 0;
        if (abs(temp_y) < 5) temp_y = 0;
        if (abs(temp_rotation) < 5) temp_rotation = 0;

        x = temp_x;
        y = temp_y;
        rotation = temp_rotation;
    }

    // Convert joystick values to velocities (m/s)
    float linear_x = x / 100.0;
    float linear_y = y / 100.0;
    float angular_z = rotation / 100.0;

    // Compute tangential velocity for rotation
    float tangential_vel = angular_z * (wheels_y_distance / 2.0);

    // Convert m/s to RPM
    float x_rpm = (linear_x * 60.0) / (PI * wheel_diameter);
    float y_rpm = (linear_y * 60.0) / (PI * wheel_diameter);
    float tan_rpm = (tangential_vel * 60.0) / (PI * wheel_diameter);

    // Compute wheel RPMs
    float sp[4];
    sp[0] = transformationMatrix[0][0] * x_rpm + transformationMatrix[0][1] * y_rpm + transformationMatrix[0][2] * tan_rpm;
    sp[1] = -(transformationMatrix[1][0] * x_rpm + transformationMatrix[1][1] * y_rpm + transformationMatrix[1][2] * tan_rpm);
    sp[2] = transformationMatrix[2][0] * x_rpm + transformationMatrix[2][1] * y_rpm + transformationMatrix[2][2] * tan_rpm;
    sp[3] = -(transformationMatrix[3][0] * x_rpm + transformationMatrix[3][1] * y_rpm + transformationMatrix[3][2] * tan_rpm);

    // Scale RPM if exceeding max speed
    float max_val = max(max(abs(sp[0]), abs(sp[1])), max(abs(sp[2]), abs(sp[3])));
    if (max_val > max_rpm) {
        float scale = max_rpm / max_val;
        for (int i = 0; i < 4; i++) {
            sp[i] *= scale;
        }
    }

    // Map RPM values to motor PWM range (-10000 to 10000)
    for (int i = 0; i < 4; i++) {
        sp[i] = map(sp[i], -max_rpm, max_rpm, -10000, 10000);
    }

    // Send speeds to motors
    motor1.setSpeed(sp[0]);
    motor2.setSpeed(sp[1]);
    motor3.setSpeed(sp[2]);
    motor4.setSpeed(sp[3]);

    // Read encoders and compute RPM
    float omega[4] = {0, 0, 0, 0};
    bool moving = false;

    for (int i = 0; i < 4; i++) {
        long currentCounts = myEnc[i].read();
        long positionChange = currentCounts - lastCount[i];

        rpm[i] = (positionChange / encoderResolution) * (60.0 / sampling_time);
        omega[i] = (rpm[i] * 2.0 * PI) / 60.0;
        lastCount[i] = currentCounts;

        if (fabs(rpm[i]) > 0.5) moving = true;
    }

    // Read IMU acceleration and gyro
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float temp = bno.getTemp();

    float accel_threshold = 0.2;
    float gyro_threshold = 0.1;

    bool imu_motion = (fabs(accel.x()) > accel_threshold || 
                       fabs(accel.y()) > accel_threshold || 
                       fabs(accel.z()) > accel_threshold || 
                       fabs(gyro.z()) > gyro_threshold);

    if (moving || imu_motion) {
        float vx = (wheel_radius / 4) * (0 * omega[0] - 1 * omega[1] + 0 * omega[2] + 1 * omega[3]);
        float vy = (wheel_radius / 4) * (1 * omega[0] + 0 * omega[1] - 1 * omega[2] + 0 * omega[3]);
        float omega_r = (wheel_radius / (4 * wheels_y_distance)) * (omega[0] + omega[1] + omega[2] + omega[3]);

        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        theta = euler.x() * (PI / 180.0);

        x_pos += vx * sampling_time * cos(theta) - vy * sampling_time * sin(theta);
        y_pos += vx * sampling_time * sin(theta) + vy * sampling_time * cos(theta);
    }

    Serial.print(x_pos);
    Serial.print(",");
    Serial.print(y_pos);
    Serial.print(",");
    Serial.println(theta);
  //   Serial.print(", Encoders: ");
  // for (int i = 0; i < 4; i++) {
  //     Serial.print("E");
  //     Serial.print(i+1);
  //     Serial.print(":");
  //     Serial.print(lastCount[i]);
  //     if (i < 3) Serial.print(", ");
  // }
  // // Serial.println();
  //   Serial.print(",");
    // Serial.print(accel.x());
    // Serial.print(",");
    // Serial.print(accel.y());
    // Serial.print(",");
    // Serial.println(euler.z());

    // Serial.print(",Gx:"); Serial.print(gyro.x());
    // Serial.print(",Gy:"); Serial.print(gyro.y());
    // Serial.print(",Gz:"); Serial.print(gyro.z());

    // Serial.print(",Mx:"); Serial.print(mag.x());
    // Serial.print(",My:"); Serial.print(mag.y());
    // Serial.print(",Mz:"); Serial.print(mag.z());

    // Serial.print(",Yaw:"); Serial.print(euler.x());
    // Serial.print(",Pitch:"); Serial.print(euler.y());
    // Serial.print(",Roll:"); Serial.print(euler.z());

    // Serial.print(",Temp:"); Serial.println(temp);
}

void setup() {
    Serial.begin(115200);
    myusb.begin();
    // if (!bno.begin()) {
    //     Serial.println("IMU not detected!");
    //     while (1);
    // }
    Serial.println("Odometry & IMU Running...");
    timer.begin(updateOdometryAndIK, 75000);
}

void loop() {}