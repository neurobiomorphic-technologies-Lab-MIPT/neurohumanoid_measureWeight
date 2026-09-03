import time
import sys
import numpy as np
from unitree_g1 import UnitreeG1
from pinfuncs import MassEstimator
import threading

# Example usage (if run as main)
def main():
    print("WARNING: Ensure the robot is in a safe position and no obstacles are nearby.")
    input("Press Enter to continue...")

    robot = UnitreeG1()

    q_hist_r = np.loadtxt("q_hist_r.txt")
    q_hist_l = np.loadtxt("q_hist_l.txt")
    q_r = q_hist_r[0]
    q_l = q_hist_r[0]

    measured_mass = None

    robot.set_arm_r(q_r)
    robot.set_arm_l(q_l)
    robot.enable_arm_sdk(duration=2.0)

    time.sleep(3.0)
    print("replay")

    # measure!

    result_container = [None]

    def measure_wrapper():
        # Call the existing measurement method and store the result
        result_container[0] = MassEstimator.measure(freq=100, side='b', api=robot)

    mass_thread = threading.Thread(target=measure_wrapper, daemon=True)
    # mass_thread.start()

    for i in range(np.shape(q_hist_r)[0]):

        if i == 101:
            mass_thread.start()

        q_r = q_hist_r[i]
        q_l = q_hist_r[i]

        robot.set_arm_r(q_r)
        robot.set_arm_l(q_l)

        time.sleep(0.02)

    print("Diasabling")
    robot.disable_arm_sdk(duration=5.0)
    time.sleep(6.0)


main()