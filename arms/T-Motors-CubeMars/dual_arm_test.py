import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

def setZeroPosition(motor):
    print(f"Resetting motor {motor.motor_id} to zero position...")
    motor.set_zero_position()
    time.sleep(0.5)

def ramp_to_position(motor, target_deg, duration=0.5, steps=100):
    current_pos, _, _ = motor.send_deg_command(0, 0, 0, 0, 0)
    pos_array = np.linspace(current_pos, target_deg, steps)

    for pos in pos_array:
        motor.send_deg_command(pos, 2, 30, 3, 0)
        time.sleep(duration / steps)

    final_pos, vel, curr = motor.send_deg_command(target_deg, 2, 30, 3, 0)
    return final_pos, vel, curr

def initialize_arm(can_interface, motor_config):
    controller_dict = {}
    for motor_id, motor_type in motor_config.items():
        controller_dict[motor_id] = CanMotorController(can_interface, motor_id, motor_type=motor_type)
    return controller_dict

def enable_and_zero_motors(motor_dict, name="Arm"):
    print(f"Enabling motors for {name}...")
    for motor_id, controller in motor_dict.items():
        controller.enable_motor()
        print(f"Motor {motor_id} enabled.")
    print(f"Setting {name} motors to zero position...")
    for controller in motor_dict.values():
        setZeroPosition(controller)

def main():
    # if len(sys.argv) < 3:
    #     print("Usage: python3 dual_arm_motor_control.py <can_interface1> <can_interface2>")
    #     sys.exit(1)

    can_interface_1 = 0
    can_interface_2 = 1

    print(f"Using {can_interface_1} for Arm 1 and {can_interface_2} for Arm 2")

    # 4 Motors per arm
    motor_config_arm1 = {
        1: "AK70_10_V1p1", 2: "AK70_10_V1p1", 3: "AK60_6_V1p1", 4: "AK60_6_V1p1"
    }
    motor_config_arm2 = {
        5: "AK70_10_V1p1", 6: "AK70_10_V1p1", 7: "AK60_6_V1p1", 8: "AK60_6_V1p1"
    }

    motor_limits = {
        1: (-90, 90),
        2: (-90, 90),
        3: (-90, 90),
        4: (-180, 180),
        5: (-90, 90),
        6: (-90, 90),
        7: (-90, 90),
        8: (-180, 180)
    }

    kps = {
        1: 20,
        2: 20,
        3: 10,
        4: 10,
        5: 20,
        6: 20,
        7: 10,
        8: 10
    }

    arm1 = initialize_arm(can_interface_1, motor_config_arm1)
    arm2 = initialize_arm(can_interface_2, motor_config_arm2)

    enable_and_zero_motors(arm1, "Arm 1")
    enable_and_zero_motors(arm2, "Arm 2")

    all_motor_ids = list(motor_config_arm1.keys()) + list(motor_config_arm2.keys())

    print("\nReady to accept commands.")
    print(f"Enter 8 space-separated angles for motors {all_motor_ids} (Arm 1 + Arm 2):")

    try:
        while True:
            try:
                input_str = input(f"Target positions ({len(all_motor_ids)} values): ")
                target_pos_list = input_str.strip().split()

                if len(target_pos_list) != len(all_motor_ids):
                    print(f"Please enter exactly {len(all_motor_ids)} values.")
                    continue

                try:
                    target_pos_list = [float(p) for p in target_pos_list]
                except ValueError:
                    print("Invalid input. Please enter valid numbers.")
                    continue

                targets_arm1 = dict(zip(motor_config_arm1.keys(), target_pos_list[:4]))
                targets_arm2 = dict(zip(motor_config_arm2.keys(), target_pos_list[4:]))

                for motor_id, target_deg in targets_arm1.items():
                    if not (motor_limits[motor_id][0] <= target_deg <= motor_limits[motor_id][1]):
                        print(f"Motor {motor_id}: Target {target_deg}° out of bounds. Skipping.")
                        continue
                    controller = arm1[motor_id]
                    if arm1[motor_id].motor_type == "AK60_6_V1p1":
                        print(f"Ramping Arm 1 Motor {motor_id} to {target_deg}°")
                        pos_deg, vel, curr = controller.send_deg_command(target_deg, 0, kps[motor_id], 2, 0)
                        print(f"Motor {motor_id} — Position: {pos_deg:.2f}°, Velocity: {vel:.2f}, Current: {curr:.2f} A\n")
                    else:
                        print(f"Ramping Arm 1 Motor {motor_id} to {target_deg}°")
                        ramp_to_position(controller, target_deg)

                for motor_id, target_deg in targets_arm2.items():
                    if not (motor_limits[motor_id][0] <= target_deg <= motor_limits[motor_id][1]):
                        print(f"Motor {motor_id}: Target {target_deg}° out of bounds. Skipping.")
                        continue
                    controller = arm2[motor_id]
                    if arm1[motor_id].motor_type == "AK60_6_V1p1":
                        print(f"Ramping Arm 1 Motor {motor_id} to {target_deg}°")
                        pos_deg, vel, curr = controller.send_deg_command(target_deg, 0, kps[motor_id], 2, 0)
                        print(f"Motor {motor_id} — Position: {pos_deg:.2f}°, Velocity: {vel:.2f}, Current: {curr:.2f} A\n")
                    else:
                        print(f"Ramping Arm 1 Motor {motor_id} to {target_deg}°")
                        ramp_to_position(controller, target_deg)

            except Exception as e:
                print(f"Runtime Error: {e}")

    except KeyboardInterrupt:
        print("\nShutting down...")
        for motor_id, ctrl in {**arm1, **arm2}.items():
            ctrl.disable_motor()
            print(f"Motor {motor_id} disabled.")
        sys.exit(0)

if __name__ == "__main__":
    main()
