import sys
import os

# Ensure local imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import is_prim_path_valid

import numpy as np

from g1_controller import SimulatedUnitreeG1


# -----------------------------
# 1. Create World
# -----------------------------
world = World()
world.scene.add_default_ground_plane()


# -----------------------------
# 2. Load G1 USD
# -----------------------------
usd_path = "C:/Documentation/g1_12dof_clean.usd"

print("[INFO] Loading USD:", usd_path)

add_reference_to_stage(usd_path, "/World/G1")

exists = is_prim_path_valid("/World/G1")
print("[INFO] G1 exists in stage:", exists)

if not exists:
    raise RuntimeError("G1 failed to load. Check USD path.")


# -----------------------------
# 3. Create Articulation
# -----------------------------
robot = world.scene.add(
    Articulation(
        prim_path="/World/G1",
        name="g1_robot"
    )
)


# -----------------------------
# 4. Initialize Simulation
# -----------------------------
world.reset()

# Lift robot AFTER reset (important!)
robot.set_world_pose(position=np.array([0.0, 0.0, 0.78]))
robot.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
robot.set_angular_velocity(np.array([0.0, 0.0, 0.0]))

print("[INFO] DOF:", robot.num_dof)

if robot.num_dof == 0:
    raise RuntimeError("Robot has 0 DOF — import failed.")


# -----------------------------
# 5. Create Controller
# -----------------------------
controller = SimulatedUnitreeG1(robot)

# Use a REAL Standing Pose
default_pose = np.zeros(robot.num_dof)

# Legs (example mapping — may need tuning)
if robot.num_dof >= 6:
    default_pose[0] = 0.0   # hip roll
    default_pose[1] = 0.0   # hip yaw
    default_pose[2] = 0.15  # hip pitch
    default_pose[3] = -0.3  # knee
    default_pose[4] = 0.15  # ankle pitch
    default_pose[5] = 0.0   # ankle roll

controller.cmd.dq = np.zeros(robot.num_dof)
controller.cmd.q = default_pose

# -----------------------------
# 6. Simulation Loop
# -----------------------------
print("[INFO] Starting simulation loop...")

# Let robot settle FIRST (no control)
for _ in range(100):
    world.step(render=True)

while simulation_app.is_running():
    t = world.current_time

    # Small sinusoidal motion (safe test)
    controller.cmd.q = default_pose
    controller.cmd.dq = np.zeros(robot.num_dof)

    controller.step()
    world.step(render=True)


# -----------------------------
# 7. Shutdown
# -----------------------------
simulation_app.close()