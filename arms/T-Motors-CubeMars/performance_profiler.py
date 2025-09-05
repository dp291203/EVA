import cProfile
import time
import numpy as np
from canmotorlib import CanMotorController

# Motor ID
motor_id = 0x02
can_port = 'can0'
motor_controller = CanMotorController(can_port, motor_id)


def motor_send_n_commands(numTimes):
    for i in range(numTimes):
        pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)


if __name__ == "__main__":
    motor_controller.enable_motor()
    steps_array = np.linspace(1000, 1000, 1)
    startdtTest = time.time()
    # steps_array = np.linspace(0, 5, 6)
    for i in steps_array:
        print("Starting Profiler for {} Commands".format(i))
        cmdString ="motor_send_n_commands({})".format(int(i))
        profiler = cProfile.Profile()
        profiler.run(cmdString)
        profiler.print_stats()

    enddtTest = time.time()
    motor_controller.disable_motor()
    dt = (enddtTest - startdtTest) / 1000
    cmd_freq = 1 / dt
    print("Dt = {}".format(dt))
    print("Command Frequency: {} Hz".format(cmd_freq))
