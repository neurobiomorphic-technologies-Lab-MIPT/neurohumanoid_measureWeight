import pinocchio as pin
import numpy as np
import csv
import time
import os
from datetime import datetime
import json
from unitree_g1 import UnitreeG1

# from arm_planning_api import ArmPlanningAPI  

# DFTP HANDS:
# URDF_PATH = r"/home/sr/RomanSentsov_ws/measure_weight/onRobot/Models/urdfs/urdf/robot_testCorrectArm.urdf"

# OMNIPICKERS:
# URDF_PATH = r"/home/sr/RomanSentsov_ws/measure_weight/onRobot/TestMass/Models/urdfs/urdf/robot_testCorrectArm_omnipicker.urdf"
URDF_PATH = r"Models/g1_robot_description/urdf/g1_29dof.urdf"
# URDF_PATH = r"C:\\Users\\CodeCompileUser\\Documents\\MIPT\\Seer\\Models\\urdfs\\urdf\\robot_testCorrectArm_noHand.urdf"
# URDF_PATH = r"C:\\Users\\CodeCompileUser\\Documents\\MIPT\\Seer\\Models\\urdfs\\urdf\\robot_testCorrectArm_rHand.urdf"

CONFIG_PATH = r"measure_modules/config.json"

class MassEstimator():

    def __init__(self):

                # Загружаем конфиг
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
            self.bias_r = config.get("bias_r", -1.05)   # значение по умолчанию, если поле отсутствует
            self.bias_l = config.get("bias_l", -1.22)
        except FileNotFoundError:
            print(f"Конфиг {CONFIG_PATH} не найден, используются значения по умолчанию.")
            self.bias_r = 0.0
            self.bias_l = 0.0

        # TODO    
        self.joints_to_lock_forleft = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", 
                                    #    "left_shoulder_pitch_joint", 
                                    #    "left_shoulder_roll_joint", 
                                    #    "left_shoulder_yaw_joint", 
                                    #    "left_elbow_joint", 
                                    #    "left_wrist_roll_joint", 
                                    #    "left_wrist_pitch_joint", 
                                    #    "left_wrist_yaw_joint", 

                                       "right_shoulder_pitch_joint", 
                                       "right_shoulder_roll_joint", 
                                       "right_shoulder_yaw_joint", 
                                       "right_elbow_joint", 
                                       "right_wrist_roll_joint", 
                                       "right_wrist_pitch_joint", 
                                       "right_wrist_yaw_joint",
                                       ]

        self.joints_to_lock_forright = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", 
                                       "left_shoulder_pitch_joint", 
                                       "left_shoulder_roll_joint", 
                                       "left_shoulder_yaw_joint", 
                                       "left_elbow_joint", 
                                       "left_wrist_roll_joint", 
                                       "left_wrist_pitch_joint", 
                                       "left_wrist_yaw_joint", 

                                    #    "right_shoulder_pitch_joint", 
                                    #    "right_shoulder_roll_joint", 
                                    #    "right_shoulder_yaw_joint", 
                                    #    "right_elbow_joint", 
                                    #    "right_wrist_roll_joint", 
                                    #    "right_wrist_pitch_joint", 
                                    #    "right_wrist_yaw_joint",
                                       ]
        
        # TODO
        self.left_ee = "left_rubber_hand"
        self.right_ee = "right_rubber_hand"

        self.model = pin.buildModelFromUrdf(URDF_PATH)
        self.data = self.model.createData()

        print("model name: " + self.model.name)

        for joint_id in range(0, self.model.njoints): # Print joint names, for tests
            print(self.model.names[joint_id])

        # for name in self.joints_to_lock:
        #     if self.model.existJointName(name): print(f"{name} found")

        # R hand:
        self.joints_to_lock_ids_R = [self.model.getJointId(name) for name in self.joints_to_lock_forright if self.model.existJointName(name)]
        self.reference_config_R = np.zeros(self.model.nq)
        self.model_reduced_R = pin.buildReducedModel(self.model, self.joints_to_lock_ids_R, self.reference_config_R)
        self.data_reduced_R = self.model_reduced_R.createData()
        self.ee_frame_id_R = self.model_reduced_R.getFrameId(self.right_ee)
        self.direction = -1

        # L hand:
        self.joints_to_lock_ids_L = [self.model.getJointId(name) for name in self.joints_to_lock_forleft if self.model.existJointName(name)]
        self.reference_config_L = np.zeros(self.model.nq)
        self.model_reduced_L = pin.buildReducedModel(self.model, self.joints_to_lock_ids_L, self.reference_config_L)
        self.data_reduced_L = self.model_reduced_L.createData()
        self.ee_frame_id_L = self.model_reduced_L.getFrameId(self.left_ee)
        self.direction = -1
        

    def calc_mass_1(self, q, v, a, tau_real, q_prev = 0, 
                    Static = False, v_prev = 0, t_step = 1/100., 
                    AccountForFric = True, returnTauFree = False, q_sign = None, 
                    side = 'r'):

        if side == 'r':
            self.model_reduced = self.model_reduced_R
            self.data_reduced = self.data_reduced_R
            self.ee_frame_id = self.ee_frame_id_R

        elif side == 'l':
            self.model_reduced = self.model_reduced_L
            self.data_reduced = self.data_reduced_L
            self.ee_frame_id = self.ee_frame_id_L

        q = np.array(q)
        v = np.array(v)
        a = np.array(a)

        q_prev = np.array(q_prev)
        v_prev = np.array(v_prev)

        tau_real = np.array(tau_real)

        a_ee = None

        # ee acceleration
        if not Static:
            pin.forwardKinematics(self.model_reduced, self.data_reduced, q, v, np.zeros(7))
            pin.updateFramePlacements(self.model_reduced, self.data_reduced)
            lin_vel = pin.getFrameVelocity(self.model_reduced, self.data_reduced, self.ee_frame_id, pin.ReferenceFrame.WORLD)
            lin_vel = lin_vel.linear

            pin.forwardKinematics(self.model_reduced, self.data_reduced, q_prev, v_prev, np.zeros(7))
            pin.updateFramePlacements(self.model_reduced, self.data_reduced)
            lin_vel_prev = pin.getFrameVelocity(self.model_reduced, self.data_reduced, self.ee_frame_id, pin.ReferenceFrame.WORLD)
            lin_vel_prev = lin_vel_prev.linear

            a_ee = (lin_vel - lin_vel_prev) / t_step

        #inverse dynamics (tau if there were no SKU)
        tau_free = pin.rnea(self.model_reduced, self.data_reduced, q, v, a)
        if returnTauFree:
            return(tau_free)

        tau_fric_dry = np.array([0, 0, 0, 0, 0, 0, 0])
        tau_fric_visc = np.array([0, 0, 0, 0, 0, 0, 0])
        tau_bias = np.array([0, 0, 0, 0, 0, 0, 0])


        if AccountForFric == True:
            # lin_regr: (Best params)

            # tau_fric_dry = np.array([-0.04225309,  0.22515251, -0.21561908,  0.11466957,  0.11522171,  0.04716471, -0.01614801]) * q_sign
            # tau_fric_visc = np.array([ 0.8918382,   0.59947135,  1.11521509,  0.97388678,  0.20399482,  0.64007138, 0.31469004]) * v

            tau_fric_dry = np.zeros(7)
            tau_fric_visc = np.zeros(7)

        tau_ext = tau_real - tau_free - tau_fric_dry - tau_fric_visc - tau_bias

        # calc J for all joints and frames
        pin.computeJointJacobians(self.model_reduced, self.data_reduced, q)
        pin.updateFramePlacements(self.model_reduced, self.data_reduced)

        J_world = pin.getFrameJacobian(self.model_reduced, self.data_reduced, self.ee_frame_id, pin.ReferenceFrame.WORLD)

        F_ext = (np.linalg.pinv(J_world.T) @ tau_ext)
        mass_approx = 0

        if Static:
            mass_approx = F_ext[2] / (9.81)
        else:
            mass_approx = F_ext[2] / (9.81 + a_ee[2])

        return mass_approx
    

    @staticmethod
    def measure(freq=100, side='r', api=None):
        '''
        # Method to measure mass in a period of time:

        ## Args:
            freq: 100 hz is recommended when running on robot
            side: 'r', 'l', 'b' for right, left or both resp
            api: optional ArmPlanningAPI instance, if None creates new one
        '''

        TIME = 5.0 # IS TUNED DEPENDING ON FINAL TRAJECTORY AND SPEED OF THE ROBOT
        PRINT_DEBUG = True
        WRITE_CSV = True

        num_ticks = int(freq * TIME)

        if api is None:
            api = UnitreeG1()
        estimator = MassEstimator()
        
        # cycle params
        dt_target = 1.0 / freq 

        prev_vel_r = np.zeros(7)
        prev_vel_l = np.zeros(7)

        prev_q_r = np.zeros(7)
        prev_q_l = np.zeros(7)

        prev_time = None
        sign_dq_prev_l = np.ones(7)
        sign_dq_prev_r = np.ones(7)

        m_av_r = 0
        m_av_l = 0

        ticks = 0
        m_hist_r = []
        q_hist_r = []
        dq_hist_r = []
        ddq_hist_r = []
        tau_hist_r = []

        m_hist_l = []
        q_hist_l = []
        dq_hist_l = []
        ddq_hist_l = []
        tau_hist_l = []

        # Create a directory named with the current date and time to save the results
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        save_dir = os.path.join("mass_measurements", timestamp)
        os.makedirs(save_dir, exist_ok=True)
        print(f"Results will be saved in: {os.path.abspath(save_dir)}")

        for ticks in range(1, num_ticks + 1):

            loop_start = time.time()
            
            # data = None

            # data = api.getALL()
            # data_l = data['left_joints_state']
            # data_r = data['right_joints_state']

            # q_l   = np.array(data_l["pos"])
            # v_l   = np.array(data_l["vel"])
            # tau_l = np.array(data_l["effort"])

            # q_r   = np.array(data_r["pos"])
            # v_r   = np.array(data_r["vel"])
            # tau_r = np.array(data_r["effort"])
            
            q_r, v_r, tau_r = api.get_arm_r()
            q_l, v_l, tau_l = api.get_arm_l()

            q_r = np.array(q_r)
            q_l = np.array(q_l)
            v_r = np.array(v_r)
            v_l = np.array(v_l)
            tau_r = np.array(tau_r)
            tau_l = np.array(tau_l)
            
            # 2. Stiction direction
            eps = 0.005

            sign_dq_r = sign_dq_prev_r
            for j in range(7):
                if np.abs(v_r[j]) > eps:
                    sign_dq_r[j] = np.sign(v_r[j])

            sign_dq_l = sign_dq_prev_l
            for j in range(7):
                if np.abs(v_l[j]) > eps:
                    sign_dq_l[j] = np.sign(v_l[j])


            # 3. Accel calculation
            if prev_time is None:
                a_r = np.zeros_like(v_r)
                a_l = np.zeros_like(v_l)
                dt = dt_target
            else:
                dt = loop_start - prev_time
                if dt <= 0:
                    dt = dt_target
                a_r = (v_r - prev_vel_r) / dt
                a_l = (v_l - prev_vel_l) / dt
            
            # 4. Mass calc for each moment
            estimated_mass_r = estimator.bias_r + estimator.calc_mass_1(q_r, v_r, a_r, tau_r, Static=False, q_prev=prev_q_r, v_prev=prev_vel_r, t_step=1/freq, AccountForFric=True, q_sign=sign_dq_r, side='r')
            estimated_mass_l = estimator.bias_l + estimator.calc_mass_1(q_l, v_l, a_l, tau_l, Static=False, q_prev=prev_q_l, v_prev=prev_vel_l, t_step=1/freq, AccountForFric=True, q_sign=sign_dq_l, side='l')
            m_av_r = m_av_r * (ticks - 1)/(ticks) + estimated_mass_r / ticks
            m_av_l = m_av_l * (ticks - 1)/(ticks) + estimated_mass_l / ticks
            
            # 5. Print:
            if PRINT_DEBUG:
                 print("-----------------------------")
                 print(f"Immidate:{(estimated_mass_l):.2f} === {(estimated_mass_r):.2f} =+= {(estimated_mass_r + estimated_mass_l):.2f}")
                 print(f"Iters:{ticks} / {num_ticks}")
                 print(f"Sliding avarage: {(m_av_l):.2f} === {(m_av_r):.2f} =+= {(m_av_r + m_av_l):.2f}")
                 print()
            
            # Updating prev values
            prev_vel_r = v_r.copy()
            prev_vel_l = v_l.copy()

            prev_q_r = q_r.copy()
            prev_q_l = q_l.copy()

            prev_time = loop_start
            
            # 6. Sleep until next tick
            elapsed = time.time() - loop_start
            # print(elapsed)
            sleep_time = dt_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            m_hist_r.append(estimated_mass_r)
            q_hist_r.append(q_r)
            dq_hist_r.append(v_r)
            ddq_hist_r.append(a_r)
            tau_hist_r.append(tau_r)

            m_hist_l.append(estimated_mass_l)
            q_hist_l.append(q_l)
            dq_hist_l.append(v_l)
            ddq_hist_l.append(a_l)
            tau_hist_l.append(tau_l)
        
        mass_r = np.array(m_hist_r).sum()/num_ticks
        mass_l = np.array(m_hist_l).sum()/num_ticks
        mass_b = mass_r + mass_l

        # Save the collected history data to CSV files for easy loading via numpy
        if side in ['r', 'b']:
            np.savetxt(os.path.join(save_dir, "q_r.csv"), np.array(q_hist_r), delimiter=",")
            np.savetxt(os.path.join(save_dir, "dq_r.csv"), np.array(dq_hist_r), delimiter=",")
            np.savetxt(os.path.join(save_dir, "ddq_r.csv"), np.array(ddq_hist_r), delimiter=",")
            np.savetxt(os.path.join(save_dir, "tau_r.csv"), np.array(tau_hist_r), delimiter=",")
            np.savetxt(os.path.join(save_dir, "mass_r.csv"), np.array(m_hist_r), delimiter=",")
            
        if side in ['l', 'b']:
            np.savetxt(os.path.join(save_dir, "q_l.csv"), np.array(q_hist_l), delimiter=",")
            np.savetxt(os.path.join(save_dir, "dq_l.csv"), np.array(dq_hist_l), delimiter=",")
            np.savetxt(os.path.join(save_dir, "ddq_l.csv"), np.array(ddq_hist_l), delimiter=",")
            np.savetxt(os.path.join(save_dir, "tau_l.csv"), np.array(tau_hist_l), delimiter=",")
            np.savetxt(os.path.join(save_dir, "mass_l.csv"), np.array(m_hist_l), delimiter=",")

        if side == 'r':
            # return (mass_r)
            print(f"Measure for RIGHT HAND: {(mass_r):.3f}")
            return mass_r
        
        if side =='l':
            # return (mass_l)
            print(f"Measure for LEFT HAND: {(mass_l):.3f}")
            return mass_l
        
        # return mass_b
        print(f"Measure for BOTH HANDS: {(mass_b):.3f}")
        return mass_b, mass_r, mass_l