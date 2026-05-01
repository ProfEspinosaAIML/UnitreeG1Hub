import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

kPi = 3.141592654

class G1JointIndex:
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    kNotUsedJoint = 29


class LeftArmRaise:
    def __init__(self):
        self.dt = 0.02
        self.time_ = 0.0

        self.stage_duration = 3.0

        self.kp = 60.0
        self.kd = 1.5

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update = False

        self.crc = CRC()
        self.done = False

        self.current_stage = -1

        self.joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
        ]

        self.target_raise = [-0.3, 0.2, 0.0, -1.0, 0.0, 0.5, 0.0]
        self.target_extend = [-0.3, 0.2, 0.0, -1.0, 0.0, 1.0, 0.2]

        self.initial_pose = None

    def Init(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateHandler, 10)

    def LowStateHandler(self, msg):
        self.low_state = msg

        if not self.first_update:
            self.first_update = True
            self.initial_pose = [msg.motor_state[j].q for j in self.joints]

    def log_stage(self, stage, description):
        if self.current_stage != stage:
            self.current_stage = stage
            print(f"\n[Time {self.time_:.2f}s] Stage {stage}: {description}")

    def Start(self):
        while not self.first_update:
            time.sleep(0.5)

        self.thread = RecurrentThread(
            interval=self.dt,
            target=self.ControlLoop,
            name="arm_control"
        )
        self.thread.Start()

    def interp(self, a, b, r):
        return (1 - r) * a + r * b

    def ControlLoop(self):
        self.time_ += self.dt

        t = self.time_
        d = self.stage_duration

        # Enable SDK (default ON)
        enable_value = 1.0

        # -------- Stage 1 --------
        if t < d:
            self.log_stage(1, "Stabilizing (holding current pose)")

            for j in self.joints:
                q = self.low_state.motor_state[j].q
                self.low_cmd.motor_cmd[j].q = q
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

        # -------- Stage 2 --------
        elif t < 2*d:
            self.log_stage(2, "Raising left arm")

            r = (t - d) / d
            for i, j in enumerate(self.joints):
                self.low_cmd.motor_cmd[j].q = self.interp(self.initial_pose[i], self.target_raise[i], r)
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

        # -------- Stage 3 --------
        elif t < 3*d:
            self.log_stage(3, "Extending wrist (simulated hand open)")

            r = (t - 2*d) / d
            for i, j in enumerate(self.joints):
                self.low_cmd.motor_cmd[j].q = self.interp(self.target_raise[i], self.target_extend[i], r)
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

        # -------- Stage 4 --------
        elif t < 5*d:
            self.log_stage(4, "Returning to initial pose")

            r = (t - 3*d) / (2*d)
            for i, j in enumerate(self.joints):
                self.low_cmd.motor_cmd[j].q = self.interp(self.target_extend[i], self.initial_pose[i], r)
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

        # -------- Stage 5 (FIXED) --------
        elif t < 6*d:
            self.log_stage(5, "Releasing arm SDK control (holding final pose)")

            r = (t - 5*d) / d
            enable_value = (1 - r)

            # CRITICAL FIX: hold final pose rigidly
            for i, j in enumerate(self.joints):
                self.low_cmd.motor_cmd[j].q = self.initial_pose[i]
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

        else:
            if not self.done:
                print("\nMotion complete. Robot should be stable at rest.")
            self.done = True
            enable_value = 0.0

        # Apply enable flag LAST
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = enable_value

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)


if __name__ == '__main__':
    print("WARNING: Ensure robot is safely supported (gantry attached).")
    input("Press Enter to start...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = LeftArmRaise()
    ctrl.Init()
    ctrl.Start()

    while True:
        time.sleep(1)
        if ctrl.done:
            sys.exit(0)
