# run_g1_sim.py

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

import numpy as np
import os, sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from g1_hand7_isaacsim import LeftArmRaise, USE_ISAAC_SIM

assert USE_ISAAC_SIM


# =========================
# Simulation Adapter
# =========================
class SimulatedUnitreeG1:
    def __init__(self, articulation):
        self.robot = articulation

        self.joint_names = list(self.robot._articulation_view.joint_names)
        self.name_to_index = {n: i for i, n in enumerate(self.joint_names)}

    def get_joint_positions(self, names):
        q = self.robot.get_joint_positions()
        return [q[self.name_to_index[n]] for n in names]

    def set_joint_positions(self, names, targets):
        q = self.robot.get_joint_positions()
        for n, t in zip(names, targets):
            q[self.name_to_index[n]] = t
        self.robot.set_joint_positions(q)

    def step(self):
        pass


# =========================
# World Setup
# =========================
world = World()
world.scene.add_default_ground_plane()

# TODO: Colsolidate all Unitree G1 assets and update the path here...
usd_path = "C:/IsaacAssets/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
add_reference_to_stage(usd_path, "/World/G1")

robot = world.scene.add(
    Articulation(prim_path="/World/G1", name="g1_robot")
)

world.reset()

# 🔥 GANTRY EQUIVALENT (CRITICAL)
robot.set_world_pose(position=np.array([0, 0, 1.0]))


# =========================
# Controller
# =========================
sim_robot = SimulatedUnitreeG1(robot)

controller = LeftArmRaise(sim_robot=sim_robot)
controller.Init()

print("[INFO] Running simulation...")


# =========================
# Main Loop
# =========================
while simulation_app.is_running():
    controller.ControlStep()
    sim_robot.step()
    world.step(render=True)

simulation_app.close()
