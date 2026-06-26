# run_g1_sim.py

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False
})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

from isaacsim.core.utils.stage import (
    add_reference_to_stage
)

import numpy as np
import os
import sys

# ============================================================
# Local imports
# ============================================================

sys.path.append(
    os.path.dirname(os.path.abspath(__file__))
)

from g1_botharms7_isaacsim import (
    DualArmWave,
    USE_ISAAC_SIM
)

assert USE_ISAAC_SIM

# ============================================================
# Simulation Adapter
# ============================================================

class SimulatedUnitreeG1:

    def __init__(self, articulation):

        self.robot = articulation

        self.joint_names = list(
            self.robot._articulation_view.joint_names
        )

        # Isaac joints usually end with "_joint"
        self.name_to_index = {
            n.replace("_joint", ""): i
            for i, n in enumerate(self.joint_names)
        }

        print("\n[INFO] Isaac joint mapping:")
        for name in self.name_to_index:
            print("   ", name)

    # ========================================================
    # Semantic-space APIs
    # ========================================================

    def get_joint_positions(self, names):

        q = self.robot.get_joint_positions()

        return [
            q[self.name_to_index[n]]
            for n in names
        ]

    def set_joint_positions(self, names, targets):

        q = self.robot.get_joint_positions()

        for n, t in zip(names, targets):

            idx = self.name_to_index[n]

            q[idx] = t

        self.robot.set_joint_positions(q)

    def step(self):
        pass


# ============================================================
# World setup
# ============================================================

world = World()

world.scene.add_default_ground_plane()

# ============================================================
# USD asset
# ============================================================

usd_path = (
    "C:/IsaacAssets/unitree_model/G1/29dof/usd/"
    "g1_29dof_rev_1_0/"
    "g1_29dof_rev_1_0.usd"
)

add_reference_to_stage(
    usd_path,
    "/World/G1"
)

# ============================================================
# Articulation
# ============================================================

robot = world.scene.add(
    Articulation(
        prim_path="/World/G1",
        name="g1_robot"
    )
)

world.reset()

# ============================================================
# Gantry equivalent
# ============================================================

robot.set_world_pose(
    position=np.array([0.0, 0.0, 1.0])
)

# ============================================================
# Simulation backend
# ============================================================

sim_robot = SimulatedUnitreeG1(robot)

# ============================================================
# Unified controller
# ============================================================

controller = DualArmWave(
    sim_robot=sim_robot
)

controller.Init()

print("\n[INFO] Running Isaac Sim controller...")

# ============================================================
# Main simulation loop
# ============================================================

while simulation_app.is_running():

    controller.ControlStep()

    sim_robot.step()

    world.step(render=True)

simulation_app.close()
