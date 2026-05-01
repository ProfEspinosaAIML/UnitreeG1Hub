# g1_hand7_isaacsim.py
USE_ISAAC_SIM = True  # Toggle

if USE_ISAAC_SIM:
    import numpy as np
else:
    import time, sys, numpy as np
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread


class LeftArmRaise:
    def __init__(self, sim_robot=None):
        self.dt = 0.02
        self.time_ = 0.0
        self.stage_duration = 3.0
        self.kp = 80.0
        self.kd = 2.0
        self.done = False

        # SIM names (with _joint suffix)
        self.joint_names = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint"
        ]

        # Target motions (same for sim + real)
        self.target_raise = [-0.3, 0.2, 0.0, -1.0, 0.0, 0.5, 0.0]
        self.target_extend = [-0.3, 0.2, 0.0, -1.0, 0.0, 1.0, 0.2]

        self.initial_pose = None
        self.first_update = False
        self.sim_robot = sim_robot

        if not USE_ISAAC_SIM:
            self.low_cmd = unitree_hg_msg_dds__LowCmd_()
            self.low_state = None
            self.crc = CRC()

            # Hardware mapping (NO _joint suffix)
            self.name_to_index = {
                "left_shoulder_pitch": 15,
                "left_shoulder_roll": 16,
                "left_shoulder_yaw": 17,
                "left_elbow": 18,
                "left_wrist_roll": 19,
                "left_wrist_pitch": 20,
                "left_wrist_yaw": 21,
            }

    def Init(self):
        if USE_ISAAC_SIM:
            self.initial_pose = self.sim_robot.get_joint_positions(self.joint_names)

            # HARD stabilization (gantry equivalent)
            self.sim_robot.set_joint_positions(self.joint_names, self.initial_pose)

            self.first_update = True
        else:
            self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            self.pub.Init()

            self.sub = ChannelSubscriber("rt/lowstate", LowState_)
            self.sub.Init(self.LowStateHandler, 10)

    def LowStateHandler(self, msg):
        self.low_state = msg

        if not self.first_update:
            self.first_update = True
            self.initial_pose = [
                msg.motor_state[self.name_to_index[name.replace("_joint", "")]].q
                for name in self.joint_names
            ]

    def interp(self, a, b, r):
        return (1 - r) * a + r * b

    def apply_targets(self, targets, enable_value):
        if USE_ISAAC_SIM:
            self.sim_robot.set_joint_positions(self.joint_names, targets)
        else:
            for i, name in enumerate(self.joint_names):
                j = self.name_to_index[name.replace("_joint", "")]

                self.low_cmd.motor_cmd[j].q = targets[i]
                self.low_cmd.motor_cmd[j].dq = 0
                self.low_cmd.motor_cmd[j].kp = self.kp
                self.low_cmd.motor_cmd[j].kd = self.kd
                self.low_cmd.motor_cmd[j].tau = 0

            self.low_cmd.motor_cmd[29].q = enable_value
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)

    def ControlStep(self):
        self.time_ += self.dt

        t = self.time_
        d = self.stage_duration
        enable_value = 1.0

        # ===== Stabilization phase (NEW) =====
        if t < 1.0:
            targets = self.initial_pose

        # ===== Stage 1 =====
        elif t < d:
            targets = self.initial_pose

        # ===== Stage 2 =====
        elif t < 2 * d:
            r = (t - d) / d
            targets = [
                self.interp(self.initial_pose[i], self.target_raise[i], r)
                for i in range(len(self.joint_names))
            ]

        # ===== Stage 3 =====
        elif t < 3 * d:
            r = (t - 2 * d) / d
            targets = [
                self.interp(self.target_raise[i], self.target_extend[i], r)
                for i in range(len(self.joint_names))
            ]

        # ===== Stage 4 =====
        elif t < 5 * d:
            r = (t - 3 * d) / (2 * d)
            targets = [
                self.interp(self.target_extend[i], self.initial_pose[i], r)
                for i in range(len(self.joint_names))
            ]

        # ===== Stage 5 =====
        elif t < 6 * d:
            r = (t - 5 * d) / d
            enable_value = (1 - r)
            targets = self.initial_pose

        else:
            self.done = True
            enable_value = 0.0
            targets = self.initial_pose

        self.apply_targets(targets, enable_value)

    def Start(self):
        if not USE_ISAAC_SIM:
            while not self.first_update:
                time.sleep(0.5)

            self.thread = RecurrentThread(
                interval=self.dt,
                target=self.ControlStep
            )
            self.thread.Start()


if USE_ISAAC_SIM:
    print("INFO: Robot in IsaacSim simulation mode")
else:
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

