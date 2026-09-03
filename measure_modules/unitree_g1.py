import time
import sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13
    WaistPitch = 14
    WaistB = 14

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    kNotUsedJoint = 29  # used for arm_sdk enable/disable


# Lists of joint indices for left and right arms (each has 7 DOF)
LEFT_ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,
]

RIGHT_ARM_JOINTS = [
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]


class UnitreeG1:
    def __init__(self, ifname="enxc84d4427fee8", control_dt=0.02, WaistZeros = True):
        """
        Initialize connection to the G1 robot and start the control thread.

        Args:
            ifname (str, optional): Network interface name (e.g., 'eth0'). If None, uses default.
            control_dt (float): Control loop period in seconds.
        """
        # Initialize DDS channel
        if ifname is not None:
            ChannelFactoryInitialize(0, ifname)
        else:
            print("Insert interface name as an argument to class init!!!")
            ChannelFactoryInitialize(0)

        self.WaistZeros = WaistZeros

        # Create publisher for arm commands
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()

        # Create subscriber for low state
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._low_state_handler, 10)

        # Command and state containers
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False
        self.crc = CRC()

        # Control parameters
        self.kp = 60.0
        self.kd = 1.5
        self.control_dt = control_dt

        # Target positions for left and right arms (7 elements each)
        self.target_pos_l = None
        self.target_pos_r = None

        # State for smooth enabling/disabling of arm_sdk
        self.enable_current = 0.0          # current value sent to kNotUsedJoint
        self.enable_target = 0.0           # desired value
        self.enable_start_time = None
        self.enable_start_val = 0.0
        self.enable_duration = 2.0

        # Control thread
        self.thread = None
        self.running = False

        # Wait for first low state message
        while not self.first_update:
            time.sleep(0.1)

        # Initialize target positions to current joint angles
        self.target_pos_l = [self.low_state.motor_state[j].q for j in LEFT_ARM_JOINTS]
        self.target_pos_r = [self.low_state.motor_state[j].q for j in RIGHT_ARM_JOINTS]

        # Start the control thread
        self.thread = RecurrentThread(
            interval=self.control_dt,
            target=self._low_cmd_write,
            name="g1_control"
        )
        self.thread.Start()
        self.running = True

    def _low_state_handler(self, msg: LowState_):
        """Callback for low state messages."""
        self.low_state = msg
        if not self.first_update:
            self.first_update = True

    def _low_cmd_write(self):
        """Periodic control function executed in the background thread."""
        if not self.first_update:
            return

        # ---- Smooth arm_sdk enable/disable ----
        if self.enable_current != self.enable_target:
            if self.enable_start_time is None:
                self.enable_start_time = time.time()
                self.enable_start_val = self.enable_current

            elapsed = time.time() - self.enable_start_time
            ratio = min(elapsed / self.enable_duration, 1.0)
            new_val = self.enable_start_val + (self.enable_target - self.enable_start_val) * ratio
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = new_val

            if ratio >= 1.0:
                self.enable_current = self.enable_target
                self.enable_start_time = None
        else:
            # Keep sending the current enable value
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = self.enable_current

        # ---- Send commands for left arm ----
        if self.target_pos_l is not None:
            for i, joint in enumerate(LEFT_ARM_JOINTS):
                self.low_cmd.motor_cmd[joint].q = self.target_pos_l[i]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        # ---- Send commands for right arm ----
        if self.target_pos_r is not None:
            for i, joint in enumerate(RIGHT_ARM_JOINTS):
                self.low_cmd.motor_cmd[joint].q = self.target_pos_r[i]
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        # Command for waist if is zero:
        if self.WaistZeros:
            for joint in [G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch]:
                self.low_cmd.motor_cmd[joint].q = 0.0
                self.low_cmd.motor_cmd[joint].dq = 0.0
                self.low_cmd.motor_cmd[joint].tau = 0.0
                self.low_cmd.motor_cmd[joint].kp = 100
                self.low_cmd.motor_cmd[joint].kd = self.kd

        # Compute CRC and publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)

    # ---- Public API ----

    def get_arm_l(self):
        """
        Get current state of the left arm.

        Returns:
            tuple: (q_list, v_list, tau_list) each a list of 7 floats.
        """
        if self.low_state is None:
            return None, None, None
        q = [self.low_state.motor_state[j].q for j in LEFT_ARM_JOINTS]
        v = [self.low_state.motor_state[j].dq for j in LEFT_ARM_JOINTS]
        tau = [self.low_state.motor_state[j].tau_est for j in LEFT_ARM_JOINTS]
        return q, v, tau

    def get_arm_r(self):
        """
        Get current state of the right arm.

        Returns:
            tuple: (q_list, v_list, tau_list) each a list of 7 floats.
        """
        if self.low_state is None:
            return None, None, None
        q = [self.low_state.motor_state[j].q for j in RIGHT_ARM_JOINTS]
        v = [self.low_state.motor_state[j].dq for j in RIGHT_ARM_JOINTS]
        tau = [self.low_state.motor_state[j].tau_est for j in RIGHT_ARM_JOINTS]
        return q, v, tau

    def set_arm_l(self, q_l):
        """
        Send target positions to the left arm.

        Args:
            q_l (list): List of 7 target joint angles [rad].
        """
        if len(q_l) != 7:
            raise ValueError("q_l must have exactly 7 elements")
        self.target_pos_l = list(q_l)

    def set_arm_r(self, q_r):
        """
        Send target positions to the right arm.

        Args:
            q_r (list): List of 7 target joint angles [rad].
        """
        if len(q_r) != 7:
            raise ValueError("q_r must have exactly 7 elements")
        self.target_pos_r = list(q_r)

    def enable_arm_sdk(self, duration=2.0):
        """
        Smoothly enable arm_sdk (sets kNotUsedJoint from 0 to 1).

        Args:
            duration (float): Transition time in seconds.
        """
        if self.enable_current == 1.0 and self.enable_target == 1.0:
            return  # already enabled
        self.enable_target = 1.0
        self.enable_duration = duration
        self.enable_start_time = None  # will be set in _low_cmd_write
        self.enable_start_val = self.enable_current

    def disable_arm_sdk(self, duration=2.0):
        """
        Smoothly disable arm_sdk (sets kNotUsedJoint from 1 to 0).

        Args:
            duration (float): Transition time in seconds.
        """
        if self.enable_current == 0.0 and self.enable_target == 0.0:
            return  # already disabled
        self.enable_target = 0.0
        self.enable_duration = duration
        self.enable_start_time = None
        self.enable_start_val = self.enable_current

    def shutdown(self):
        """Stop the control thread and clean up."""
        if self.thread is not None:
            self.thread.Stop()
            self.running = False

    def __del__(self):
        self.shutdown()


