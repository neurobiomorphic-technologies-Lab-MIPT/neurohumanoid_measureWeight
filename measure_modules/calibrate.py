import json
import shutil
import time
import numpy as np
import threading
from arm_planning_api import ArmPlanningAPI
from pinfuncs import MassEstimator, CONFIG_PATH

IP = "192.168.192.5"

# Параметры движения (копируем из основного кода)
ROTATION_VELOCITY = np.deg2rad(90)
JOINT_VELOCITY = np.pi
JOINT_ACCELERATION = 1.5
DELAY = 0.2

left_posture = [0.822, 0.418, -1.283, 1.625, -1.200, -0.464, -0.360]
right_posture = [-0.822, 0.418, 1.283, 1.625, 1.200, -0.464, 0.360]
left_posture1 = [0.824, 0.913, -0.559, 1.216, -0.987, 0.221, -1.172]
right_posture1 = [-0.824, 0.913, 0.559, 1.216, 0.987, 0.221, 1.172]
left_posture2 = [0.811, 0.992, -0.532, 1.229, -0.945, 0.196, -1.082]
right_posture2 = [-0.811, 0.992, 0.532, 1.229, 0.945, 0.196, 1.082]
left_posture3 = [0.608, 0.905, -0.635, 1.461, -1.078, -0.155, -1.007]
right_posture3 = [-0.608, 0.905, 0.635, 1.461, 1.078, -0.155, 1.007]

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

def calibrate_cycle(api):
    """Выполняет один цикл и возвращает (mass_b, mass_r, mass_l)"""
    measured = None
    for T, cmd, p in commands:
        if cmd == 'rotate':
            api.robot_rotate(p, ROTATION_VELOCITY)
            start = time.perf_counter()
            while time.perf_counter() - start <= T:
                api.waitMove()
        else:
            qr, ql = p
            api.controlDualArm(qr, ql, vel=JOINT_VELOCITY, acc=JOINT_ACCELERATION)
            if cmd == 'lift':
                time.sleep(0.3)
                result = [None]
                def measure():
                    result[0] = MassEstimator.measure(freq=100, side='b', api=api)
                th = threading.Thread(target=measure, daemon=True)
                th.start()
            api.waitMove()
            if cmd == 'lift':
                th.join(timeout=2.0)
                measured = result[0]
                if measured is None:
                    measured = (0.0, 0.0, 0.0)
    return measured

def calibrate():
    # 0. Резервная копия
    backup_path = CONFIG_PATH + ".backup"
    shutil.copy(CONFIG_PATH, backup_path)
    print(f"Резервная копия сохранена: {backup_path}")

    # 1. Зануляем bias в конфиге
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
    config['bias_r'] = 0.0
    config['bias_l'] = 0.0
    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=4)
    print("bias временно обнулены.")

    # 2. Подключаемся к роботу и выполняем цикл
    api = ArmPlanningAPI(IP)
    api.out_switch_real_time_mode()
    input("Убедитесь, что манипуляторы пусты, и нажмите Enter для начала калибровки...")
    mass_b, mass_r, mass_l = calibrate_cycle(api)
    print(f"Измеренные массы при нулевых bias: R={mass_r:.3f} кг, L={mass_l:.3f} кг, сумма={mass_b:.3f} кг")

    # 3. Вычисляем новые bias (со знаком минус)
    new_bias_r = -mass_r - 0.05
    new_bias_l = -mass_l - 0.05

    # 4. Обновляем конфиг
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
    config['bias_r'] = new_bias_r
    config['bias_l'] = new_bias_l
    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=4)
    print(f"Конфиг обновлён: bias_r={new_bias_r:.3f}, bias_l={new_bias_l:.3f}")

    print("Калибровка завершена.")

if __name__ == "__main__":
    calibrate()