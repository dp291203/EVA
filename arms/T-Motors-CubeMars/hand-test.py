import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

def setZeroPosition(motor):
    print(f"Resetting motor {motor.motor_id} to zero position...")
    motor.set_zero_position()
    time.sleep(0.5)

def ramp_to_position(motor, target_deg, duration=0.5, steps=100):
    current_pos, _, _ = motor.send_deg_command(0, 0, 0, 0, 0)  # Use 0s to get current pos
    pos_array = np.linspace(current_pos, target_deg, steps)

    for pos in pos_array:
        motor.send_deg_command(pos, 2, 30, 3, 0)
        time.sleep(duration / steps)

    final_pos, vel, curr = motor.send_deg_command(target_deg, 2, 30, 3, 0)
    return final_pos, vel, curr

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 motor_script.py <can_interface> (e.g., can0)")
        sys.exit(1)

    can_interface = sys.argv[1]
    print(f"Using Socket {can_interface} for CAN communication")

    # Motor ID to type mapping
    motor_config = {
        5: "AK70_10_V1p1",
        2: "AK70_10_V1p1",
        14: "AK60_6_V1p1",
        10: "AK60_6_V1p1"
        
    }

    # Motor ID to angle limits (in degrees)
    motor_limits = {
        5: (-90, 90),
        2: (-90, 90),
        14: (-90, 90),
        10: (-180, 180)    
    }

    motor_controller_dict = {}
    motor_ids = list(motor_config.keys())

    for motor_id, motor_type in motor_config.items():
        motor_controller_dict[motor_id] = CanMotorController(can_interface, motor_id, motor_type=motor_type)

    print("Enabling Motors...")
    for motor_id, controller in motor_controller_dict.items():
        controller.enable_motor()
        print(f"Motor {motor_id} ({motor_config[motor_id]}) enabled.")

    print("Setting Motors to Zero Position...")
    for controller in motor_controller_dict.values():
        setZeroPosition(controller)

    time.sleep(1)
    print("Ready to accept commands.")
    print(f"Enter {len(motor_ids)} space-separated target positions in degrees for motors {motor_ids}")

    try:
        while True:
            try:
                input_str = input(f"Target positions ({len(motor_ids)} values): ")
                target_pos_list = input_str.strip().split()

                if len(target_pos_list) != len(motor_ids):
                    print(f"Please enter exactly {len(motor_ids)} values.")
                    continue

                try:
                    target_pos_list = [float(p) for p in target_pos_list]
                except ValueError:
                    print("Invalid input. Please enter valid numbers.")
                    continue
                
                kps = [20, 20, 10, 10]
                for motor_id, target_deg, i in zip(motor_ids, target_pos_list, kps):
                    min_deg, max_deg = motor_limits[motor_id]
                    if not (min_deg <= target_deg <= max_deg):
                        print(f"Motor {motor_id}: Target {target_deg}° out of bounds ({min_deg}° to {max_deg}°). Skipping.")
                        continue

                    controller = motor_controller_dict[motor_id]
                    print(f"Ramping Motor {motor_id} to {target_deg}°")
                    if motor_id == 10 or motor_id == 14:
                        pos_deg, vel, curr = motor_controller_dict[motor_id].send_deg_command(target_deg, 0, i, 2, 0)
                        print("Motor {motor_id} — Position: {pos_deg:.2f}°, Velocity: {vel:.2f} rad/s, Current: {curr:.2f} A\n")
                    else:
                        pos_deg, vel, curr = ramp_to_position(controller, target_deg)
                        print(f"Motor {motor_id} — Position: {pos_deg:.2f}°, Velocity: {vel:.2f} rad/s, Current: {curr:.2f} A\n")

            except Exception as e:
                print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\nDisabling Motors and Exiting...")
        for motor_id, controller in motor_controller_dict.items():
            controller.disable_motor()
            time.sleep(0.2)
            print(f"Motor {motor_id} disabled.")
        sys.exit(0)

if __name__ == "__main__":
    main()
