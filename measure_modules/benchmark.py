from arm_planning_api import ArmPlanningAPI
import time
import numpy as np
import threading
from pinfuncs import MassEstimator
import csv
import sys

IP = "192.168.192.5"

arm_planning_api = ArmPlanningAPI(IP)

# Pose definitions (kept as in original)
right_posture = [np.deg2rad(-90), np.deg2rad(90), np.deg2rad(90), np.deg2rad(90), 0, 0, 0]
left_posture = [np.deg2rad(90), np.deg2rad(90), np.deg2rad(-90), np.deg2rad(90), 0, 0, 0]

left_posture = [ 1.06350388,  0.90959531, -1.10078531,  1.27921272, -0.77327757, 0.01631627, -1.25595592]
right_posture = [-1.06436089,  0.90639292,  1.10016505,  1.27544475,  0.77602652, 0.02069887,  1.2567242 ]

# Standby pose
left_posture = [0.822, 0.418, -1.283, 1.625, -1.200, -0.464, -0.360]
right_posture = [-0.822, 0.418, 1.283, 1.625, 1.200, -0.464, 0.360]

# PrePick
left_posture1 = [0.824, 0.913, -0.559, 1.216, -0.987, 0.221, -1.172]
right_posture1 = [-0.824, 0.913, 0.559, 1.216, 0.987, 0.221, 1.172]
# Pick
left_posture2 = [0.811, 0.992, -0.532, 1.229, -0.945, 0.196, -1.082]
right_posture2 = [-0.811, 0.992, 0.532, 1.229, 0.945, 0.196, 1.082]
# Lift
left_posture3 = [0.608, 0.905, -0.635, 1.461, -1.078, -0.155, -1.007]
right_posture3 =  [-0.608, 0.905, 0.635, 1.461, 1.078, -0.155, 1.007]

ROTATION_VELOCITY = np.deg2rad(90)
JOINT_VELOCITY = np.pi
JOINT_ACCELERATION = 1.5

DELAY=0.2

points = {
    'standby': (right_posture, left_posture),
    'pre_pick': (right_posture1, left_posture1),
    'pick': (right_posture2, left_posture2),
    'lift': (right_posture3, left_posture3),
}

commands = [
    (0, 'pre_pick', points['pre_pick']),
    (0, 'pick', points['pick']),
    (0, 'lift', points['lift']),
    # (5, 'rotate', np.deg2rad(-95)),
    (0, 'pick', points['pick']),
    (0, 'pre_pick', points['pre_pick']),
    (0, 'standby', points['standby']),
    # (5, 'rotate', np.deg2rad(95)),
]

def run_one_cycle():
    """
    Executes one full motion cycle (pre_pick -> pick -> lift -> rotate -> pick -> pre_pick -> standby -> rotate)
    and returns the measured mass obtained during the 'lift' phase.
    """
    measured_mass = None

    for T, cmd, p in commands:
        if cmd == 'rotate':
            arm_planning_api.robot_rotate(p, ROTATION_VELOCITY)
            start = time.perf_counter()
            while time.perf_counter() - start <= T:
                arm_planning_api.waitMove()
        else:
            qr, ql = p
            arm_planning_api.controlDualArm(qr, ql, vel=JOINT_VELOCITY, acc=JOINT_ACCELERATION)

            if cmd == 'lift':
                # Short pause before starting measurement
                time.sleep(0.3)
                # Container for measurement result
                result_container = [None]

                def measure_wrapper():
                    # Call the existing measurement method and store the result
                    result_container[0] = MassEstimator.measure(freq=100, side='b', api=arm_planning_api)

                mass_thread = threading.Thread(target=measure_wrapper, daemon=True)
                mass_thread.start()

            # Wait for the movement to complete
            arm_planning_api.waitMove()

            if cmd == 'lift':
                # Let the measurement thread finish (max 2 seconds)
                mass_thread.join(timeout=2.0)
                measured_mass = result_container[0]
                # If measurement did not return a value, set to 0
                if measured_mass is None:
                    measured_mass = (0.0, 0.0, 0.0)

    return measured_mass[0]

def save_csv(records, filename="measurements.csv"):
    """Saves records to a CSV file with headers."""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['file_name', 'datestamp', 'ground_truth', 'output', 'inference_time'])
        writer.writerows(records)
    print(f"Data saved to {filename}")

def print_statistics(records, measurements):
    """
    Prints a table with results and accuracy metrics.
    records: list of records [file_name, datestamp, ground_truth, output, inference_time]
    measurements: list of tuples (real_mass, measured_mass)
    """
    if not records or not measurements:
        print("No data for statistics.")
        return

    print("\n" + "="*80)
    print("MEASUREMENT STATISTICS")
    print("="*80)

    # Table headers
    header = f"{'No':<4} {'Actual mass (kg)':<16} {'Measured mass (kg)':<16} {'Error (kg)':<14} {'Actually defective?':<18} {'Module said defective?':<20}"
    print(header)
    print("-"*80)

    tp = tn = fp = fn = 0

    for idx, (rec, meas) in enumerate(zip(records, measurements), start=1):
        _, _, ground_truth, output, _ = rec
        real_mass, measured_mass = meas
        
        # Calculate absolute error
        error = abs(real_mass - measured_mass)

        # Determine defective labels (0 - defective, 1 - non-defective)
        real_defective = "yes" if real_mass > 5.0 else "no"
        pred_defective = "yes" if measured_mass > 4.8 else "no"

        # Update confusion matrix
        if ground_truth == 1 and output == 1:
            tp += 1
        elif ground_truth == 0 and output == 0:
            tn += 1
        elif ground_truth == 0 and output == 1:
            fp += 1
        elif ground_truth == 1 and output == 0:
            fn += 1

        # Print table row with error column
        print(f"{idx:<4} {real_mass:<16.3f} {measured_mass:<16.3f} {error:<14.3f} {real_defective:<18} {pred_defective:<20}")

    print("-"*80)

    total = tp + tn + fp + fn
    if total > 0:
        accuracy = (tp + tn) / total
        print(f"Accuracy: {accuracy:.4f} ({accuracy*100:.2f}%)")
    else:
        accuracy = 0.0
        print("Accuracy: N/A (no data)")

    print("\nConfusion Matrix:")
    print(f"  TP (true non-defective, correctly identified): {tp}")
    print(f"  TN (true defective, correctly identified):   {tn}")
    print(f"  FP (false non-defective, type I error):      {fp}")
    print(f"  FN (false defective, type II error):         {fn}")
    print("="*80 + "\n")

def main():
    records = []
    all_measurements = []   # list to store (real_mass, measured_mass)
    experiment_number = 1
    print("Mass measurement program.")
    print("Press Enter to start the next measurement cycle.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # Wait for Enter press
            input("Press Enter to start measurement...")
            print("Executing movement cycle...")

            measured_mass = run_one_cycle()
            if measured_mass is None:
                measured_mass = 0.0
            print(f"Measured mass: {measured_mass:.3f} kg")

            if (measured_mass < 4.8):
                print("Non-defective SKU")
            else:
                print("Defective SKU") 

            # Ask user for actual mass
            real_mass_str = input("Enter actual mass in kg: ")
            try:
                real_mass = float(real_mass_str)
            except ValueError:
                print("Invalid input, assuming actual mass = 0")
                real_mass = 0.0

            # Compute binary labels
            ground_truth = 1 if real_mass <= 5.0 else 0
            output = 1 if measured_mass <= 4.8 else 0
            timestamp = int(time.time())
            file_name = f"{experiment_number:03d}"

            record = [file_name, timestamp, ground_truth, output, 0.8]
            records.append(record)
            all_measurements.append((real_mass, measured_mass))
            experiment_number += 1

            print(f"Record #{experiment_number-1} added.\n")

    except KeyboardInterrupt:
        print("\nTerminated by Ctrl+C.")
        if records:
            print_statistics(records, all_measurements)
            save_csv(records)
        else:
            print("No data to save.")
        sys.exit(0)

if __name__ == "__main__":
    main()