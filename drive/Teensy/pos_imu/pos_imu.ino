#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <CytronMotorDriver.h>
#include "USBHost_t36.h"

// Joystick setup
USBHost myusb;
USBHIDParser hid1(myusb);
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");
// BluetoothController bluet(myusb);

// Motor drivers
CytronMD motor1(PWM_DIR, 3, 5);
CytronMD motor2(PWM_DIR, 22, 20);
CytronMD motor3(PWM_DIR, 23, 21);
CytronMD motor4(PWM_DIR, 2, 4);

// BNO055 IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Motion variables
double xPos = 0, yPos = 0, headingVel = 0;
const float BNO055_SAMPLERATE_DELAY_MS = 10;
const float PRINT_DELAY_MS = 500;
uint16_t printCount = 0;

const float ACCEL_VEL_TRANSITION = BNO055_SAMPLERATE_DELAY_MS / 1000.0;
const float ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
const float DEG_2_RAD = 0.01745329251;

// Joystick commands
int x = 0, y = 0, rotation = 0;
const float max_rpm = 100;
const float wheel_diameter = 6.35;
const float wheels_y_distance = 51.0;

// Transformation matrix for holonomic motion
const float transformationMatrix[4][3] = {
    {  0,  1, 0.255 },  // Front-left
    { -1,  0, 0.255 },  // Front-right
    {  0, -1, 0.255 },  // Rear-left
    {  1,  0, 0.255 }   // Rear-right
};

IntervalTimer timer;

void setup() {
    Serial.begin(115200);
    myusb.begin();

    while (!Serial) delay(10);

    if (!bno.begin()) {
        Serial.println("No BNO055 detected");
        while (1);
    }

    Serial.println("IMU and Inverse Kinematics Running...");
    timer.begin(calculateIK, 75000);
}

void loop() {
    unsigned long tStart = micros();
    sensors_event_t orientationData, linearAccelData;

    // Read IMU data
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Compute position from acceleration
    xPos += ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos += ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

    // Compute heading velocity
    headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

    // Print data at intervals
    if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
        Serial.print("Heading: ");
        Serial.println(orientationData.orientation.x);
        Serial.print("Position: ");
        Serial.print(xPos);
        Serial.print(" , ");
        Serial.println(yPos);
        Serial.print("Speed: ");
        Serial.println(headingVel);
        Serial.println("-------");

        printCount = 0;
    } else {
        printCount++;
    }

    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
        // Wait for the next sample
    }
}

// Inverse Kinematics Calculation
void calculateIK() {
    myusb.Task();

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

    // Convert joystick values to velocities
    float linear_x = x / 100.0;
    float linear_y = y / 100.0;
    float angular_z = rotation / 100.0;

    // Compute tangential velocity for rotation
    float tangential_vel = angular_z * (wheels_y_distance / 2.0);

    // Convert to RPM
    float x_rpm = (linear_x * 60.0) / (PI * wheel_diameter);
    float y_rpm = (linear_y * 60.0) / (PI * wheel_diameter);
    float tan_rpm = (tangential_vel * 60.0) / (PI * wheel_diameter);

    // Compute wheel RPMs using transformation matrix
    float sp[4];
    sp[0] = transformationMatrix[0][0] * x_rpm + transformationMatrix[0][1] * y_rpm + transformationMatrix[0][2] * tan_rpm;
    sp[1] = -(transformationMatrix[1][0] * x_rpm + transformationMatrix[1][1] * y_rpm + transformationMatrix[1][2] * tan_rpm);
    sp[2] = transformationMatrix[2][0] * x_rpm + transformationMatrix[2][1] * y_rpm + transformationMatrix[2][2] * tan_rpm;
    sp[3] = -(transformationMatrix[3][0] * x_rpm + transformationMatrix[3][1] * y_rpm + transformationMatrix[3][2] * tan_rpm);

    // Scale RPMs if they exceed max speed
    float max_val = max(max(abs(sp[0]), abs(sp[1])), max(abs(sp[2]), abs(sp[3])));
    if (max_val > max_rpm) {
        float scale = max_rpm / max_val;
        for (int i = 0; i < 4; i++) {
            sp[i] *= scale;
        }
    }

    // Map RPM values to Cytron motor range (-10000 to 10000)
    for (int i = 0; i < 4; i++) {
        sp[i] = map(sp[i], -max_rpm, max_rpm, -10000, 10000);
    }

    // Apply speed to motors
    motor1.setSpeed(sp[0]);
    motor2.setSpeed(sp[1]);
    motor3.setSpeed(sp[2]);
    motor4.setSpeed(sp[3]);

    // Debugging motor values
    Serial.print("Motor1 (FR): "); Serial.print(sp[0]);
    Serial.print(" | Motor2 (RR): "); Serial.print(sp[1]);
    Serial.print(" | Motor3 (RL): "); Serial.print(sp[2]);
    Serial.print(" | Motor4 (FL): "); Serial.println(sp[3]);
}
