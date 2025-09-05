import time
from src.motor_driver.canmotorlib import CanMotorController

def ramp_to_position(motor, target_deg, duration=0.5, steps=100):
    current_pos, _, _ = motor.send_deg_command(0, 0, 0, 0, 0)  # Use 0s to get current pos
    pos_array = np.linspace(current_pos, target_deg, steps)

    for pos in pos_array:
        motor.send_deg_command(pos, 2, 30, 3, 0)
        time.sleep(duration / steps)

    final_pos, vel, curr = motor.send_deg_command(target_deg, 2, 30, 3, 0)
    return final_pos, vel, curr

def main():
    can_interface = "can0"

    # Define motors
    motor_ids = {
        5: "AK70_10_V1p1",   # Motor 1
        2: "AK70_10_V1p1",
        14: "AK60_6_V1p1",    # Motor 3 (oscillating)
        10: "AK60_6_V1p1",    # Motor 4
    }

    motor_controllers = {}
    for motor_id, motor_type in motor_ids.items():
        motor_controllers[motor_id] = CanMotorController(can_interface, motor_id, motor_type=motor_type)
        motor_controllers[motor_id].enable_motor()
        motor_controllers[motor_id].set_zero_position()
        print(f"Motor {motor_id} enabled")

    time.sleep(1)
    
    ramp_to_position(motor_controllers[2], 5)
    print("Moving Motor 1 to -60° and Motor 4 to -120°")
    motor_controllers[10].send_deg_command(0, 0, 10, 2, 0)
    motor_controllers[5].send_deg_command(0, 0, 10, 2, 0)
    motor_controllers[14].send_deg_command(0, 0, 10, 2, 0)
    ramp_to_position(motor_controllers[5], -60)
    motor_controllers[10].send_deg_command(-120, 0, 10, 2, 0)
    time.sleep(2)

    print("Oscillating Motor 3 between -20° and 20°")
    for _ in range(2):
        motor_controllers[14].send_deg_command(-20, 0, 10, 2, 0)
        time.sleep(0.5)
        motor_controllers[14].send_deg_command(20, 0, 10, 2, 0)
        time.sleep(0.5)

    motor_controllers[14].send_deg_command(0, 0, 10, 2, 0)
    ramp_to_position(motor_controllers[5], 0)
    motor_controllers[10].send_deg_command(0, 0, 10, 2, 0)
    # print("Returning all motors to 0°")
    # for motor_id, controller in motor_controllers.items():
    #     controller.send_deg_command(0, 0, 20, 2, 0)
    time.sleep(1.5)

    print("Disabling motors...")
    
    
    for motor_id, controller in motor_controllers.items():
        controller.disable_motor()
        print(f"Motor {motor_id} disabled.")

if __name__ == "__main__":
    import numpy as np
    main()
