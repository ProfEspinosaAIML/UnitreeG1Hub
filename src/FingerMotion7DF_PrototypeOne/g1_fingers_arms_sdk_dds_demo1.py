#!/usr/bin/env python3

import sys
import time
import threading
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)

from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,
    LowState_,
    HandCmd_,
    HandState_,
    MotorCmd_,
)

from unitree_sdk2py.utils.crc import CRC

# ============================================================
# INITIALIZE DDS
# ============================================================

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

# ============================================================
# JOINT MAP
# ============================================================

class G1JointIndex:

    LeftShoulderPitch = 15
    LeftShoulderRoll  = 16
    LeftShoulderYaw   = 17
    LeftElbow         = 18
    LeftWristRoll     = 19
    LeftWristPitch    = 20
    LeftWristYaw      = 21

    RightShoulderPitch = 22
    RightShoulderRoll  = 23
    RightShoulderYaw   = 24
    RightElbow         = 25
    RightWristRoll     = 26
    RightWristPitch    = 27
    RightWristYaw      = 28

    kNotUsedJoint = 29

ARM_JOINTS = [

    G1JointIndex.LeftShoulderPitch,
    G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,
    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.LeftWristPitch,
    G1JointIndex.LeftWristYaw,

    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]

# ============================================================
# ARM POSES
# ============================================================

FORWARD_PRESENTATION = [

    0.0,  np.pi/2, 0.0, np.pi/2, 0.0, 0.0, 0.0,
    0.0, -np.pi/2, 0.0, np.pi/2, 0.0, 0.0, 0.0,
]

WIDE_PRESENTATION = [

    -0.30,  1.10,  0.00, 1.20, 0.0, 0.0, 0.0,
    -0.30, -1.10,  0.00, 1.20, 0.0, 0.0, 0.0,
]

# ============================================================
# GLOBALS
# ============================================================

running = True

low_state = None
hand_state = None

arm_initial_pose = None

crc = CRC()

# ============================================================
# ARM DDS
# ============================================================

arm_pub = ChannelPublisher(
    "rt/arm_sdk",
    LowCmd_
)
arm_pub.Init()

# ============================================================
# RIGHT HAND DDS
# ============================================================

hand_pub = ChannelPublisher(
    "rt/dex3/right/cmd",
    HandCmd_
)
hand_pub.Init()

# ============================================================
# SUBSCRIBERS
# ============================================================

def lowstate_callback(msg):

    global low_state
    global arm_initial_pose

    low_state = msg

    if arm_initial_pose is None:

        arm_initial_pose = [
            msg.motor_state[j].q
            for j in ARM_JOINTS
        ]

def handstate_callback(msg):

    global hand_state
    hand_state = msg

low_sub = ChannelSubscriber(
    "rt/lowstate",
    LowState_
)

low_sub.Init(
    lowstate_callback,
    10
)

hand_sub = ChannelSubscriber(
    "rt/lf/dex3/right/state",
    HandState_
)

hand_sub.Init(
    handstate_callback,
    10
)

# ============================================================
# WAIT FOR STATE
# ============================================================

print("Waiting for LowState and HandState...")

while low_state is None or hand_state is None:
    time.sleep(0.1)

print("State streams received.")

# ============================================================
# ARM COMMAND OBJECT
# ============================================================

arm_cmd = unitree_hg_msg_dds__LowCmd_()

# ============================================================
# HAND COMMAND
# ============================================================

current_hand_q = [0.0] * 7

def build_hand_cmd(q):

    motors = []

    for i in range(7):

        motors.append(
            MotorCmd_(
                mode=0x10 | i,
                q=float(q[i]),
                dq=0.0,
                tau=0.0,
                kp=10.0,
                kd=1.0,
                reserve=0,
            )
        )

    return HandCmd_(
        motor_cmd=motors,
        reserve=[0,0,0,0]
    )

# ============================================================
# CONTINUOUS HAND PUBLISHER
# ============================================================

def hand_thread():

    global running

    while running:

        hand_pub.Write(
            build_hand_cmd(current_hand_q)
        )

        time.sleep(0.02)

threading.Thread(
    target=hand_thread,
    daemon=True
).start()

# ============================================================
# ARM HELPERS
# ============================================================

def set_arm_pose(pose):

    arm_cmd.motor_cmd[
        G1JointIndex.kNotUsedJoint
    ].q = 1.0

    for i, joint in enumerate(ARM_JOINTS):

        arm_cmd.motor_cmd[joint].q = pose[i]
        arm_cmd.motor_cmd[joint].dq = 0.0
        arm_cmd.motor_cmd[joint].tau = 0.0
        arm_cmd.motor_cmd[joint].kp = 60.0
        arm_cmd.motor_cmd[joint].kd = 1.5

    arm_cmd.crc = crc.Crc(arm_cmd)

    arm_pub.Write(arm_cmd)

def interpolate_pose(a, b, r):

    return [
        (1-r)*x + r*y
        for x, y in zip(a, b)
    ]

def move_arm_pose(target_pose,
                  duration=3.0,
                  label=""):

    print()
    print("=" * 60)
    print(label)
    print("=" * 60)

    start_pose = [

        low_state.motor_state[j].q

        for j in ARM_JOINTS
    ]

    t0 = time.time()

    while True:

        dt = time.time() - t0

        r = min(
            1.0,
            dt / duration
        )

        pose = interpolate_pose(
            start_pose,
            target_pose,
            r
        )

        set_arm_pose(pose)

        if r >= 1.0:
            break

        time.sleep(0.02)

# ============================================================
# HAND HELPERS
# ============================================================

def hand_pose(
    thumb=0.0,
    right=0.0,
    left=0.0
):

    global current_hand_q

    current_hand_q = [

        thumb,  # q0

        0.0,    # q1 unknown
        0.0,    # q2 unknown

        right,  # q3
        right,  # q4

        left,   # q5
        left,   # q6
    ]

def hold(seconds):

    time.sleep(seconds)

# ============================================================
# MAIN DEMO
# ============================================================

input(
    "\nRobot READY. Press ENTER to start..."
)

print()
print("=" * 60)
print("G1 FINGERS + ARMS DEMO 1")
print("=" * 60)

# ------------------------------------------------------------
# STAGE 1
# ------------------------------------------------------------

print("\nSTAGE 1 : WAKE UP")

hand_pose(
    thumb=0,
    right=0,
    left=0
)

hold(2)

# ------------------------------------------------------------
# STAGE 2
# ------------------------------------------------------------

move_arm_pose(
    FORWARD_PRESENTATION,
    4.0,
    "STAGE 2 : FORWARD PRESENTATION"
)

hold(1)

# ------------------------------------------------------------
# STAGE 3
# ------------------------------------------------------------

print("\nSTAGE 3 : RIGHT FINGER")

hand_pose(right=1.0)
hold(1.5)

hand_pose(right=0.0)
hold(1.0)

# ------------------------------------------------------------
# STAGE 4
# ------------------------------------------------------------

print("\nSTAGE 4 : LEFT FINGER")

hand_pose(left=1.0)
hold(1.5)

hand_pose(left=0.0)
hold(1.0)

# ------------------------------------------------------------
# STAGE 5
# ------------------------------------------------------------

print("\nSTAGE 5 : THUMB")

hand_pose(thumb=1.0)
hold(1.5)

hand_pose(thumb=0.0)
hold(1.0)

hand_pose(thumb=-1.0)
hold(1.5)

hand_pose(thumb=0.0)
hold(1.0)

# ------------------------------------------------------------
# STAGE 6
# ------------------------------------------------------------

move_arm_pose(
    WIDE_PRESENTATION,
    3.0,
    "STAGE 6 : WIDE PRESENTATION"
)

hold(1)

# ------------------------------------------------------------
# STAGE 7
# ------------------------------------------------------------

print("\nSTAGE 7 : DEXTERITY")

for _ in range(2):

    hand_pose(right=1)
    hold(0.5)

    hand_pose(right=0)
    hold(0.3)

    hand_pose(left=1)
    hold(0.5)

    hand_pose(left=0)
    hold(0.3)

hand_pose(
    right=1,
    left=1
)
hold(1)

hand_pose(
    right=0,
    left=0
)
hold(1)

# ------------------------------------------------------------
# STAGE 8
# ------------------------------------------------------------

print("\nSTAGE 8 : THUMB SWEEP")

hand_pose(
    thumb=1.0
)
hold(1)

hand_pose(
    thumb=-1.0
)
hold(1)

hand_pose(
    thumb=1.0
)
hold(1)

hand_pose(
    thumb=0.0
)
hold(1)

# ------------------------------------------------------------
# STAGE 9
# ------------------------------------------------------------

print("\nSTAGE 9 : SIMULATED GRASP")

hand_pose(
    thumb=-1.0,
    right=1.0,
    left=1.0
)

hold(3)

# ------------------------------------------------------------
# STAGE 10
# ------------------------------------------------------------

print("\nSTAGE 10 : RELEASE")

hand_pose(
    thumb=0.0,
    right=0.0,
    left=0.0
)

hold(2)

# ------------------------------------------------------------
# STAGE 11
# ------------------------------------------------------------

move_arm_pose(
    FORWARD_PRESENTATION,
    3.0,
    "STAGE 11 : RETURN TO PRESENTATION"
)

# ------------------------------------------------------------
# STAGE 12
# ------------------------------------------------------------

move_arm_pose(
    arm_initial_pose,
    4.0,
    "STAGE 12 : RETURN HOME"
)

# ------------------------------------------------------------
# STAGE 13
# ------------------------------------------------------------

print("\nSTAGE 13 : RELEASE ARM SDK")

for i in range(100):

    r = 1.0 - (i / 99.0)

    arm_cmd.motor_cmd[
        G1JointIndex.kNotUsedJoint
    ].q = r

    arm_cmd.crc = crc.Crc(arm_cmd)

    arm_pub.Write(arm_cmd)

    time.sleep(0.02)

# ------------------------------------------------------------
# CLEANUP
# ------------------------------------------------------------

running = False

hand_pose(
    thumb=0,
    right=0,
    left=0
)

for _ in range(100):

    hand_pub.Write(
        build_hand_cmd(current_hand_q)
    )

    time.sleep(0.02)

print()
print("=" * 60)
print("DEMO COMPLETE")
print("=" * 60)