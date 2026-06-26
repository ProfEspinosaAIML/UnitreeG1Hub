USE_ISAAC_SIM = True  # Toggle between Isaac Sim and physical G1

import numpy as np

if not USE_ISAAC_SIM:
    import time
    import sys

    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
        ChannelSubscriber,
        ChannelFactoryInitialize,
    )

    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
        LowCmd_,
        LowState_,
    )

    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
    )

    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread


kPi = 3.141592654


class G1JointIndex:
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

    kNotUsedJoint = 29


class DualArmWave:

    def __init__(self, sim_robot=None):

        # =========================
        # Backend
        # =========================
        self.use_sim = USE_ISAAC_SIM
        self.sim_robot = sim_robot

        # =========================
        # Timing
        # =========================
        self.dt = 0.02
        self.time_ = 0.0
        self.stage_duration = 3.0

        # =========================
        # PD gains
        # =========================
        self.kp = 60.0
        self.kd = 1.5

        # =========================
        # Runtime flags
        # =========================
        self.first_update = False
        self.done = False
        self.current_stage = -1

        # =========================
        # Semantic joint names
        # =========================
        self.joint_names = [
            # Left arm
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_shoulder_yaw",
            "left_elbow",

            # Right arm
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_shoulder_yaw",
            "right_elbow",
        ]

        # =========================
        # Hardware indices
        # =========================
        self.joints = [
            # Left arm
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,

            # Right arm
            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
        ]

        # =========================
        # Motion targets
        # =========================

        # Arms forward
        self.target_front = [
            -0.8,  0.15, 0.0, -1.2,
            -0.8, -0.15, 0.0,  1.2
        ]

        # Arms higher
        self.target_up = [
            -1.2,  0.15, 0.0, -1.0,
            -1.2, -0.15, 0.0,  1.0
        ]

        # Arms lower
        self.target_down = [
            -0.5,  0.15, 0.0, -1.4,
            -0.5, -0.15, 0.0,  1.4
        ]

        self.initial_pose = None

        # =========================
        # DDS-only resources
        # =========================
        if not self.use_sim:

            self.low_cmd = unitree_hg_msg_dds__LowCmd_()
            self.low_state = None

            self.crc = CRC()

    # ============================================================
    # Initialization
    # ============================================================

    def Init(self):

        # --------------------------------------------------------
        # Isaac Sim backend
        # --------------------------------------------------------
        if self.use_sim:

            self.initial_pose = self.sim_robot.get_joint_positions(
                self.joint_names
            )

            self.first_update = True

            print("[INFO] Isaac Sim controller initialized.")
            print("[INFO] Initial pose captured.")

        # --------------------------------------------------------
        # Physical robot backend
        # --------------------------------------------------------
        else:

            self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self.pub.Init()

            self.sub = ChannelSubscriber(
                "rt/lowstate",
                LowState_
            )

            self.sub.Init(self.LowStateHandler, 10)

    # ============================================================
    # DDS low-state callback
    # ============================================================

    def LowStateHandler(self, msg):

        self.low_state = msg

        if not self.first_update:

            self.first_update = True

            self.initial_pose = [
                msg.motor_state[j].q
                for j in self.joints
            ]

            print("[INFO] Initial hardware pose captured.")

    # ============================================================
    # Logging
    # ============================================================

    def log_stage(self, stage, description):

        if self.current_stage != stage:

            self.current_stage = stage

            print(
                f"\n[Time {self.time_:.2f}s] "
                f"Stage {stage}: {description}"
            )

    # ============================================================
    # Hardware thread start
    # ============================================================

    def Start(self):

        if self.use_sim:
            return

        while not self.first_update:
            time.sleep(0.5)

        self.thread = RecurrentThread(
            interval=self.dt,
            target=self.ControlStep,
            name="arm_control"
        )

        self.thread.Start()

    # ============================================================
    # Helpers
    # ============================================================

    def interp(self, a, b, r):
        return (1 - r) * a + r * b

    # ============================================================
    # Backend abstraction layer
    # ============================================================

    def apply_targets(self, targets, enable_value=1.0):

        # --------------------------------------------------------
        # Isaac Sim backend
        # --------------------------------------------------------
        if self.use_sim:

            self.sim_robot.set_joint_positions(
                self.joint_names,
                targets
            )

        # --------------------------------------------------------
        # Physical G1 backend
        # --------------------------------------------------------
        else:

            for i, j in enumerate(self.joints):

                self.low_cmd.motor_cmd[j].q = targets[i]
                self.low_cmd.motor_cmd[j].dq = 0.0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0.0

            self.low_cmd.motor_cmd[
                G1JointIndex.kNotUsedJoint
            ].q = enable_value

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)

            self.pub.Write(self.low_cmd)

    # ============================================================
    # Unified control logic
    # ============================================================

    def ControlStep(self):

        if self.done:
            return

        self.time_ += self.dt

        t = self.time_
        d = self.stage_duration

        enable_value = 1.0

        # ========================================================
        # Stage 1: Stabilization
        # ========================================================
        if t < d:

            self.log_stage(1, "Stabilizing")

            targets = list(self.initial_pose)

        # ========================================================
        # Stage 2: Move arms forward
        # ========================================================
        elif t < 2 * d:

            self.log_stage(2, "Moving both arms forward")

            r = (t - d) / d

            targets = []

            for i in range(len(self.joints)):

                q_target = self.interp(
                    self.initial_pose[i],
                    self.target_front[i],
                    r
                )

                targets.append(q_target)

        # ========================================================
        # Stage 3: Arms move up/down
        # ========================================================
        elif t < 6 * d:

            self.log_stage(3, "Moving both arms up/down")

            cycle = np.sin(
                2.0 * np.pi * 0.5 * (t - 2 * d)
            )

            targets = []

            for i in range(len(self.joints)):

                q_high = self.target_up[i]
                q_low = self.target_down[i]

                alpha = (cycle + 1.0) / 2.0

                q_target = self.interp(
                    q_low,
                    q_high,
                    alpha
                )

                targets.append(q_target)

        # ========================================================
        # Stage 4: Return to initial pose
        # ========================================================
        elif t < 8 * d:

            self.log_stage(4, "Returning to initial pose")

            r = (t - 6 * d) / (2 * d)

            targets = []

            for i in range(len(self.joints)):

                q_target = self.interp(
                    self.target_front[i],
                    self.initial_pose[i],
                    r
                )

                targets.append(q_target)

        # ========================================================
        # Stage 5: Release control
        # ========================================================
        elif t < 9 * d:

            self.log_stage(5, "Releasing SDK control")

            r = (t - 8 * d) / d

            enable_value = (1.0 - r)

            targets = list(self.initial_pose)

        # ========================================================
        # Motion complete
        # ========================================================
        else:

            print("\nMotion complete.")

            self.done = True

            enable_value = 0.0

            targets = list(self.initial_pose)

        # ========================================================
        # Unified backend execution
        # ========================================================
        self.apply_targets(
            targets,
            enable_value
        )


# ================================================================
# Hardware-only entry point
# ================================================================

if not USE_ISAAC_SIM:

    if __name__ == '__main__':

        print(
            "WARNING: Ensure robot is safely supported "
            "(gantry attached)."
        )

        input("Press Enter to start...")

        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        ctrl = DualArmWave()

        ctrl.Init()
        ctrl.Start()

        while True:

            time.sleep(1)

            if ctrl.done:
                sys.exit(0)

else:

    print("INFO: Robot in Isaac Sim mode")
