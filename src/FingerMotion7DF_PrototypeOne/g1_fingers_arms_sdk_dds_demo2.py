#!/usr/bin/env python3

# ============================================================
# g1_fingers_arms_sdk_dds_showcase.py
#
# FINAL TRACK 1 SHOWCASE
#
# Validated Features:
#   Right Hand Dex3 DDS Control
#   Arm SDK Control
#   Coordinated Arm + Hand Motion
#
# Best Current Hand Model:
#
#   q0      -> Thumb Rotation
#   q1      -> Unknown
#   q2      -> Unknown
#   q3+q4   -> Right Finger
#   q5+q6   -> Left Finger
#
# ============================================================

import sys
import time
import threading
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    MotorCmd_,
    LowCmd_,
    LowState_,
)

from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
)

from unitree_sdk2py.utils.crc import CRC

# ============================================================
# JOINTS
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


# ============================================================
# DDS
# ============================================================

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

# ------------------------------------------------------------
# Hand DDS
# ------------------------------------------------------------

HAND_CMD_TOPIC   = "rt/dex3/right/cmd"
HAND_STATE_TOPIC = "rt/lf/dex3/right/state"

hand_state = None

# ------------------------------------------------------------
# Arm DDS
# ------------------------------------------------------------

low_state = None

# ============================================================
# HAND PUB/SUB
# ============================================================

hand_pub = ChannelPublisher(
    HAND_CMD_TOPIC,
    HandCmd_
)
hand_pub.Init()

def hand_state_callback(msg):

    global hand_state
    hand_state = msg

hand_sub = ChannelSubscriber(
    HAND_STATE_TOPIC,
    HandState_
)

hand_sub.Init(
    hand_state_callback,
    10
)

# ============================================================
# ARM PUB/SUB
# ============================================================

arm_pub = ChannelPublisher(
    "rt/arm_sdk",
    LowCmd_
)

arm_pub.Init()

def low_state_callback(msg):

    global low_state
    low_state = msg

low_sub = ChannelSubscriber(
    "rt/lowstate",
    LowState_
)

low_sub.Init(
    low_state_callback,
    10
)

# ============================================================
# WAIT FOR STATES
# ============================================================

print()
print("=" * 60)
print("G1 EDU FULL SHOWCASE")
print("=" * 60)
print()

print("Waiting for right hand state...")

while hand_state is None:
    time.sleep(0.1)

print("Right hand state received.")

print("Waiting for arm state...")

while low_state is None:
    time.sleep(0.1)

print("Arm state received.")
print()

input("Robot READY. Press ENTER to start showcase...")

# ============================================================
# HAND CONTROL
# ============================================================

running = True

def make_hand_cmd(q):

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

current_hand_cmd = make_hand_cmd([0]*7)

def hand_publisher():

    global running

    while running:

        hand_pub.Write(current_hand_cmd)

        time.sleep(0.02)

threading.Thread(
    target=hand_publisher,
    daemon=True
).start()

# ============================================================
# ARM CONTROL
# ============================================================

crc = CRC()

arm_cmd = unitree_hg_msg_dds__LowCmd_()

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

initial_arm_pose = [
    low_state.motor_state[j].q
    for j in ARM_JOINTS
]

# validated presentation pose

presentation_pose = [

    -0.30,  0.20, 0.00, -1.00, 0.00, 0.50, 0.00,
    -0.30, -0.20, 0.00,  1.00, 0.00, 0.50, 0.00,
]

kp = 60.0
kd = 1.5

def set_arm_pose(target):

    arm_cmd.motor_cmd[
        G1JointIndex.kNotUsedJoint
    ].q = 1.0

    for i, joint in enumerate(ARM_JOINTS):

        arm_cmd.motor_cmd[joint].q = target[i]
        arm_cmd.motor_cmd[joint].dq = 0.0
        arm_cmd.motor_cmd[joint].kp = kp
        arm_cmd.motor_cmd[joint].kd = kd
        arm_cmd.motor_cmd[joint].tau = 0.0

    arm_cmd.crc = crc.Crc(arm_cmd)

    arm_pub.Write(arm_cmd)

def move_arm(target, duration=3.0):

    start_pose = [
        low_state.motor_state[j].q
        for j in ARM_JOINTS
    ]

    t0 = time.time()

    while True:

        r = min(
            1.0,
            (time.time() - t0) / duration
        )

        pose = []

        for a, b in zip(start_pose, target):

            pose.append(
                (1-r)*a + r*b
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

    global current_hand_cmd

    current_hand_cmd = make_hand_cmd([

        thumb,   # q0
        0.0,     # q1 unknown
        0.0,     # q2 unknown

        right,   # q3
        right,   # q4

        left,    # q5
        left,    # q6
    ])

def hold(seconds):

    time.sleep(seconds)

# ============================================================
# SHOWCASE
# ============================================================

try:

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 1] WAKE UP")
    print("=" * 60)

    hand_pose(
        thumb=0,
        right=0,
        left=0
    )

    move_arm(
        presentation_pose,
        duration=4.0
    )

    hold(2)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 2] FINGER INTRODUCTION")
    print("=" * 60)

    print("q3+q4 -> RIGHT finger")

    hand_pose(right=1.0)
    hold(1.5)

    hand_pose(right=0.0)
    hold(1.0)

    print("q5+q6 -> LEFT finger")

    hand_pose(left=1.0)
    hold(1.5)

    hand_pose(left=0.0)
    hold(1.0)

    print("q3+q4 + q5+q6 -> BOTH fingers")

    hand_pose(
        right=1.0,
        left=1.0
    )

    hold(2.0)

    hand_pose()
    hold(1.0)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 3] THUMB DEMONSTRATION")
    print("=" * 60)

    print("q0 = thumb rotation")

    hand_pose(thumb=1.0)
    hold(2.0)

    hand_pose(thumb=0.0)
    hold(1.0)

    hand_pose(thumb=-1.0)
    hold(2.0)

    hand_pose(thumb=0.0)
    hold(1.0)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 4] DEXTERITY ROUTINE")
    print("=" * 60)

    for i in range(3):

        print(f"Pattern cycle {i+1}")

        hand_pose(right=1.0)
        hold(0.5)

        hand_pose(left=1.0)
        hold(0.5)

        hand_pose(right=0.0)
        hold(0.4)

        hand_pose(left=0.0)
        hold(0.4)

        hand_pose(
            right=1.0,
            left=1.0
        )
        hold(0.6)

        hand_pose()
        hold(0.4)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 5] ARM PRESENTATION")
    print("=" * 60)

    hand_pose()

    move_arm(
        initial_arm_pose,
        duration=3.0
    )

    hold(1.0)

    move_arm(
        presentation_pose,
        duration=3.0
    )

    hold(1.0)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 6] SIMULATED GRASP")
    print("=" * 60)

    hand_pose(
        thumb=-1.0,
        right=1.0,
        left=1.0
    )

    hold(4.0)

    hand_pose()
    hold(2.0)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 7] CONCURRENT MOTION")
    print("=" * 60)

    for i in range(4):

        thumb = 1.0 if i % 2 == 0 else -1.0

        hand_pose(
            thumb=thumb,
            right=1.0,
            left=0.0
        )

        hold(0.8)

        hand_pose(
            thumb=-thumb,
            right=0.0,
            left=1.0
        )

        hold(0.8)

    hand_pose()

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 8] TECHNICAL FINALE")
    print("=" * 60)

    sequence = [

        ("RIGHT", 1,0,0),
        ("LEFT", 0,1,0),
        ("BOTH", 1,1,0),
        ("THUMB +", 0,0,1),
        ("THUMB -", 0,0,-1),
    ]

    for name,r,l,t in sequence:

        print(name)

        hand_pose(
            thumb=t,
            right=r,
            left=l
        )

        hold(1.0)

    hand_pose()
    hold(1.0)

    # ========================================================
    print()
    print("=" * 60)
    print("[STAGE 9] FINAL PRESENTATION")
    print("=" * 60)

    move_arm(
        presentation_pose,
        duration=2.0
    )

    hand_pose()

    hold(5.0)

    print()
    print("=" * 60)
    print("SHOWCASE COMPLETE")
    print("=" * 60)
    print()

    print("Validated Right-Hand Model")
    print()
    print("q0      -> Thumb Rotation")
    print("q1      -> Unknown")
    print("q2      -> Unknown")
    print("q3+q4   -> Right Finger")
    print("q5+q6   -> Left Finger")
    print()

finally:

    running = False

    hand_pose()

    for _ in range(100):

        hand_pub.Write(
            make_hand_cmd([0]*7)
        )

        time.sleep(0.02)

    arm_cmd.motor_cmd[
        G1JointIndex.kNotUsedJoint
    ].q = 0.0

    arm_cmd.crc = crc.Crc(arm_cmd)

    for _ in range(100):

        arm_pub.Write(arm_cmd)

        time.sleep(0.02)

    print()
    print("Arm SDK released.")
    print("Hand returned to neutral.")
    print("Done.")
