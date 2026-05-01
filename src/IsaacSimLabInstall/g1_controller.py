import numpy as np


class HighCmd:
    def __init__(self, dof):
        self.q = np.zeros(dof)      # desired positions
        self.dq = np.zeros(dof)     # desired velocities
        self.tau = np.zeros(dof)    # feedforward torque


class HighState:
    def __init__(self, dof):
        self.q = np.zeros(dof)
        self.dq = np.zeros(dof)


class SimulatedUnitreeG1:
    def __init__(self, articulation, kp=150.0, kd=40.0):
        self.robot = articulation
        self.dof = articulation.num_dof

        self.cmd = HighCmd(self.dof)
        self.state = HighState(self.dof)

        self.kp = kp
        self.kd = kd

        print(f"[INFO] Controller initialized with DOF: {self.dof}")

    def update_state(self):
        self.state.q = self.robot.get_joint_positions()
        self.state.dq = self.robot.get_joint_velocities()

    def compute_torque(self):
        pos_error = self.cmd.q - self.state.q
        vel_error = self.cmd.dq - self.state.dq

        tau = self.kp * pos_error + self.kd * vel_error

        # Strong damping
        tau -= 15.0 * self.state.dq

        # -----------------------------
        # NEW: torso stabilization
        # -----------------------------
        pitch = self.get_torso_pitch()

        # Correct using hips (index may vary!)
        if self.dof >= 6:
            tau[2] -= 80 * pitch   # hip pitch correction
            tau[3] += 50 * pitch   # knee correction(NEW)
            tau[4] += 40 * pitch   # ankle compensation

        tau = np.tanh(tau / 100.0) * 100.0
        tau += 0.5 * np.random.randn(self.dof)
        return tau
    
    def get_torso_pitch(self):
        # Get base orientation quaternion
        quat = self.robot.get_world_pose()[1]

        # Convert to pitch (approx)
        # Isaac returns (w, x, y, z)
        w, x, y, z = quat

        # Pitch approximation
        pitch = 2 * (w * y - z * x)

        return pitch

    def send_command(self):
        tau = self.compute_torque()
        self.robot.set_joint_efforts(tau)

    def step(self):
        self.update_state()
        self.send_command()