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
CytronMD motor4(PWM_DIR, 2, 4); 
CytronMD motor3(PWM_DIR, 23, 21);

// Joystick commands
int x = 0, y = 0, rotation = 0; 
const float max_rpm = 120;
const float wheel_diameter = 6.35;  // in cm
const float wheels_y_distance = 51.0; // in cm

IntervalTimer timer;

// Transformation matrix
const float transformationMatrix[4][3] = {
    {  0,  1, 0.255 },  // Front-left
    { -1,  0, 0.255 },  // Front-right
    {  0, -1, 0.255 },  // Rear-left
    {  1,  0, 0.255 }   // Rear-right
};

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
        Serial.print("X: "); Serial.print(x);
        Serial.print(" | Y: "); Serial.print(y);
        Serial.print(" | Rotation: "); Serial.println(rotation);

    }

    // Convert joystick values to physical velocities (m/s)
    float linear_x = x / 100.0;  // Convert to meters per second
    float linear_y = y / 100.0;
    float angular_z = rotation / 100.0;

    // Compute tangential velocity for rotation
    float tangential_vel = angular_z * (wheels_y_distance / 2.0);

    // Convert m/s to RPM
    float x_rpm = (linear_x * 60.0) / (PI * wheel_diameter);
    float y_rpm = (linear_y * 60.0) / (PI * wheel_diameter);
    float tan_rpm = (tangential_vel * 60.0) / (PI * wheel_diameter);

    // Compute wheel RPMs using the transformation matrix
    float sp[4];
    sp[0] = transformationMatrix[0][0] * x_rpm + transformationMatrix[0][1] * y_rpm + transformationMatrix[0][2] * tan_rpm;
    sp[1] = -(transformationMatrix[1][0] * x_rpm + transformationMatrix[1][1] * y_rpm + transformationMatrix[1][2] * tan_rpm);
    sp[2] = transformationMatrix[2][0] * x_rpm + transformationMatrix[2][1] * y_rpm + transformationMatrix[2][2] * tan_rpm;
    sp[3] = -(transformationMatrix[3][0] * x_rpm + transformationMatrix[3][1] * y_rpm + transformationMatrix[3][2] * tan_rpm);

    // Find max RPM to scale down if needed
    float max_val = max(max(abs(sp[0]), abs(sp[1])), max(abs(sp[2]), abs(sp[3])));
    if (max_val > max_rpm) {
        float scale = max_rpm / max_val;
        for (int i = 0; i < 4; i++) {
            sp[i] *= scale;
        }
    }

    // Map RPM values to Cytron motor driver range (-16383 to 16383)
    for (int i = 0; i < 4; i++) {
        sp[i] = map(sp[i], -max_rpm, max_rpm, -10000, 10000);
    }

    // Send speeds to motors
    motor1.setSpeed(sp[0]);
    motor2.setSpeed(sp[1]);
    motor3.setSpeed(sp[2]);
    motor4.setSpeed(sp[3]);

    // Debugging
    Serial.print("Motor1 (FR): "); Serial.print(sp[0]);
    Serial.print(" | Motor2 (RR): "); Serial.print(sp[1]);
    Serial.print(" | Motor3 (RL): "); Serial.print(sp[2]);
    Serial.print(" | Motor4 (FL): "); Serial.println(sp[3]);
}

void setup() {
    Serial.begin(115200);
    myusb.begin();
    while (!Serial) delay(10);
    Serial.println("Inverse Kinematics Running");
    timer.begin(calculateIK, 75000);
}

void loop() {}
