import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController


def setZeroPosition(motor):
    print("Resetting motor to zero position...")
    motor.set_zero_position()
    time.sleep(0.5)  # Allow some time for the motor to register the zero position

def main():
    print("Using Socket can0 for CAN communication")

    # Fixed motor IDs and types
    motor_configs = {
        2: "AK70_10_V1p1",
        10: "AK60_6_V1p1",
        14: "AK60_6_V1p1"
    }

    motor_limits = {
        "AK70_10_V1p1": (0.0, 150.0),  # min, max degrees
        "AK60_6_V1p1": (0.0, 90.0)     # min, max degrees
    }

    motor_controller_dict = {
        motor_id: CanMotorController("can0", motor_id, motor_type=motor_type)
        for motor_id, motor_type in motor_configs.items()
    }

    print("Enabling Motors...")
    for motor_id, motor_controller in motor_controller_dict.items():
        motor_controller.enable_motor()
        print("Motor {} enabled.".format(motor_id))

    print("Setting Motors to Zero Position...")
    for motor_controller in motor_controller_dict.values():
        setZeroPosition(motor_controller)

    time.sleep(1)
    print("Ready to accept commands. Enter target positions in degrees for each motor (separated by space, e.g., 90 45 60).")

    try:
        while True:
            try:
                user_input = input("Enter target positions (degrees for motors 2 10 14): ")
                angles = [float(x) for x in user_input.strip().split()]
                if len(angles) != 3:
                    print("Please enter exactly 3 values.")
                    continue

                valid = True
                for motor_id, angle in zip(motor_configs.keys(), angles):
                    motor_type = motor_configs[motor_id]
                    min_limit, max_limit = motor_limits[motor_type]
                    if not (min_limit <= angle <= max_limit):
                        print(f"Angle {angle}° is out of bounds for motor {motor_id} ({motor_type} range: {min_limit}° to {max_limit}°)")
                        valid = False

                if not valid:
                    continue

                for motor_id, angle in zip(motor_configs.keys(), angles):
                    motor_controller_dict[motor_id].send_rad_command(np.radians(angle), 0, 5, 0.5, 0)
                    print("Moving Motor {} to {}°".format(motor_id, angle))

            except ValueError:
                print("Invalid input. Please enter numeric values.")

    except KeyboardInterrupt:
        print("\nDisabling Motors and Exiting...")
        for motor_id, motor_controller in motor_controller_dict.items():
            motor_controller.disable_motor()
            time.sleep(0.2)
            print("Motor {} disabled.".format(motor_id))
        sys.exit(0)

if __name__ == "__main__":
    main()
