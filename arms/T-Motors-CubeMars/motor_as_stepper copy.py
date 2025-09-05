import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

def setZeroPosition(motor):
    print("Resetting motor to zero position...")
    motor.set_zero_position()
    time.sleep(0.5)  # Allow some time for the motor to register the zero position

def main():
    if len(sys.argv) < 3:
        print("Provide CAN device name (can0, slcan0 etc.) and motor IDs. E.g. python3 can_motorlib_test.py can0 2 3")
        sys.exit(0)

    print("Using Socket {} for CAN communication".format(sys.argv[1]))
    motor_controller_dict = {}
    for i in range(2, len(sys.argv)):
        motor_controller_dict[int(sys.argv[i])] = CanMotorController(sys.argv[1], int(sys.argv[i]), motor_type="AK70_10_V1p1")

    print("Enabling Motors...")
    for motor_id, motor_controller in motor_controller_dict.items():
        motor_controller.enable_motor()
        print("Motor {} enabled.".format(motor_id))

    print("Setting Motors to Zero Position...")
    for motor_id, motor_controller in motor_controller_dict.items():
        setZeroPosition(motor_controller)

    time.sleep(1)
    print("Ready to accept commands. Enter target positions in degrees.")
    
    try:
        while True:
            try:
                target_pos = float(input("Enter target position (degrees): "))
                for motor_id, motor_controller in motor_controller_dict.items():
                    pos, vel, curr = motor_controller.send_deg_command(target_pos, 2, 20, 3, 0)  # Send position command
                    # print(vel)
                    # time.sleep(5)
                    print("Moving Motor {} to {}°".format(motor_id, target_pos))
                    # time.sleep(2)
                    # print("Posistion: {}" .format(pos_deg))
                    # print("Velocity: {}" .format(vel))
                    # print("Current: {}" .format(curr))
                    # time.sleep(2)
                    can_id, can_dlc, receive_data = motor_controller._recv_can_frame()
                    raw_motor_data = motor_controller.decode_motor_status(receive_data)
                    pos_deg, vel_ms, curr_a = motor_controller.convert_raw_to_physical_rad(raw_motor_data[0], raw_motor_data[1], raw_motor_data[2])
                    print("Posistion: {}" .format(pos_deg))
                    print("Velocity: {}" .format(vel_ms))
                    print("Current: {}" .format(curr_a))
                    time.sleep(1)
            
            except ValueError:
                print("Invalid input. Please enter a valid number.")
    
    except KeyboardInterrupt:
        print("\nDisabling Motors and Exiting...")
        for motor_id, motor_controller in motor_controller_dict.items():
            motor_controller.disable_motor()
            time.sleep(0.2)
            print("Motor {} disabled.".format(motor_id))
        sys.exit(0)

if __name__ == "__main__":
    main()
