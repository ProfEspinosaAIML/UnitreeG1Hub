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

    -0.30,  1.10, 0.00, 1.20, 0.0, 0.0, 0.0,
    -0.30, -1.10, 0.00, 1.20, 0.0, 0.0, 0.0,
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
# LEFT HAND DDS
# ============================================================

left_hand_pub = ChannelPublisher(
    "rt/dex3/left/cmd",
    HandCmd_
)
left_hand_pub.Init()

# ============================================================
# RIGHT HAND DDS
# ============================================================

right_hand_pub = ChannelPublisher(
    "rt/dex3/right/cmd",
    HandCmd_
)
right_hand_pub.Init()

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
    "rt/lf/dex3/left/state",
    HandState_
)

hand_sub.Init(
    handstate_callback,
    10
)

# ============================================================
# WAIT FOR STATE
# ============================================================

print("Waiting for LowState and LEFT HandState...")

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

left_hand_q = [0.0] * 7
right_hand_q = [0.0] * 7

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
        reserve=[0, 0, 0, 0]
    )

# ============================================================
# CONTINUOUS HAND PUBLISHERS
# ============================================================

def left_hand_thread():

    global running

    while running:

        left_hand_pub.Write(
            build_hand_cmd(left_hand_q)
        )

        time.sleep(0.02)

def right_hand_thread():

    global running

    while running:

        right_hand_pub.Write(
            build_hand_cmd(right_hand_q)
        )

        time.sleep(0.02)

threading.Thread(
    target=left_hand_thread,
    daemon=True
).start()

threading.Thread(
    target=right_hand_thread,
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

def move_arm_pose(
    target_pose,
    duration=3.0,
    label=""
):

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

def left_hand_pose(
    thumb=0.0,
    finger_a_base=0.0,
    finger_a_tip=0.0,
    finger_b_base=0.0,
    finger_b_tip=0.0
):

    global left_hand_q

    left_hand_q = [

        thumb,

        0.0,
        0.0,

        finger_a_base,
        finger_a_tip,

        finger_b_base,
        finger_b_tip,
    ]


def right_hand_pose(
    thumb=0.0,
    finger_a=0.0,
    finger_b=0.0
):

    global right_hand_q

    right_hand_q = [

        thumb,

        0.0,
        0.0,

        finger_a,
        finger_a,

        finger_b,
        finger_b,
    ]


def hand_pose(
    q0=0.0,
    q3=0.0,
    q4=0.0,
    q5=0.0,
    q6=0.0
):

    left_hand_pose(
        thumb=q0,
        finger_a_base=q3,
        finger_a_tip=q4,
        finger_b_base=q5,
        finger_b_tip=q6,
    )

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
print("G1 FINGERS + ARMS DEMO 3")
print("=" * 60)

# ------------------------------------------------------------
# STAGE 1
# ------------------------------------------------------------

print("\nSTAGE 1 : WAKE UP")

left_hand_pose()
right_hand_pose()

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

print("\nSTAGE 3 : LEFT FINGER A")

left_hand_pose(
    finger_a_base=-1.0,
    finger_a_tip=-1.0
)

hold(2)

left_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 4
# ------------------------------------------------------------

print("\nSTAGE 4 : RIGHT FINGER A")

right_hand_pose(
    finger_a=1.0
)

hold(2)

right_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 5
# ------------------------------------------------------------

print("\nSTAGE 5 : LEFT FINGER B")

left_hand_pose(
    finger_b_base=-1.0,
    finger_b_tip=-1.0
)

hold(2)

left_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 6
# ------------------------------------------------------------

print("\nSTAGE 6 : RIGHT FINGER B")

right_hand_pose(
    finger_b=1.0
)

hold(2)

right_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 7
# ------------------------------------------------------------

print("\nSTAGE 7 : LEFT THUMB")

left_hand_pose(
    thumb=-1.0
)

hold(2)

left_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 8
# ------------------------------------------------------------

print("\nSTAGE 8 : RIGHT THUMB")

right_hand_pose(
    thumb=1.0
)

hold(2)

right_hand_pose()
hold(1)

# ------------------------------------------------------------
# STAGE 9
# ------------------------------------------------------------

print("\nSTAGE 9 : BOTH THUMBS")

left_hand_pose(
    thumb=-1.0
)

right_hand_pose(
    thumb=1.0
)

hold(3)

left_hand_pose()
right_hand_pose()

hold(1)

# ------------------------------------------------------------
# STAGE 10
# ------------------------------------------------------------

print("\nSTAGE 10 : BOTH GRASP")

left_hand_pose(
    thumb=-1.0,
    finger_a_base=-1.0,
    finger_a_tip=-1.0,
    finger_b_base=-1.0,
    finger_b_tip=-1.0
)

right_hand_pose(
    thumb=1.0,
    finger_a=1.0,
    finger_b=1.0
)

hold(4)

left_hand_pose()
right_hand_pose()

hold(1)

# ------------------------------------------------------------
# STAGE 11
# ------------------------------------------------------------

print("\nSTAGE 11 : DUAL DEXTERITY")

for _ in range(3):

    left_hand_pose(
        finger_a_base=-1.0,
        finger_a_tip=-1.0
    )

    right_hand_pose(
        finger_b=1.0
    )

    hold(0.8)

    left_hand_pose(
        finger_b_base=-1.0,
        finger_b_tip=-1.0
    )

    right_hand_pose(
        finger_a=1.0
    )

    hold(0.8)

left_hand_pose()
right_hand_pose()

hold(1)

# ------------------------------------------------------------
# STAGE 12
# ------------------------------------------------------------

print("\nSTAGE 12 : FINAL DUAL GRASP")

left_hand_pose(
    thumb=-1.0,
    finger_a_base=-1.0,
    finger_a_tip=-1.0,
    finger_b_base=-1.0,
    finger_b_tip=-1.0
)

right_hand_pose(
    thumb=1.0,
    finger_a=1.0,
    finger_b=1.0
)

hold(3)

left_hand_pose()
right_hand_pose()

hold(1)

# ------------------------------------------------------------
# STAGE 13
# ------------------------------------------------------------

move_arm_pose(
    WIDE_PRESENTATION,
    3.0,
    "STAGE 13 : WIDE PRESENTATION"
)

hold(1)

# ------------------------------------------------------------
# STAGE 14
# ------------------------------------------------------------

print("\nSTAGE 14 : DUAL THUMB SHOWCASE")

left_hand_pose(
    thumb=-1.0
)

right_hand_pose(
    thumb=1.0
)

hold(1.5)

left_hand_pose()

right_hand_pose()

hold(0.8)

left_hand_pose(
    thumb=-1.0
)

right_hand_pose(
    thumb=1.0
)

hold(1.5)

left_hand_pose()
right_hand_pose()

hold(1)

# ------------------------------------------------------------
# STAGE 15
# ------------------------------------------------------------

print("\nSTAGE 15 : FINAL DUAL GRASP")

left_hand_pose(
    thumb=-1.0,
    finger_a_base=-1.0,
    finger_a_tip=-1.0,
    finger_b_base=-1.0,
    finger_b_tip=-1.0
)

right_hand_pose(
    thumb=1.0,
    finger_a=1.0,
    finger_b=1.0
)

hold(4)

# ------------------------------------------------------------
# STAGE 16
# ------------------------------------------------------------

print("\nSTAGE 16 : RELEASE")

left_hand_pose()
right_hand_pose()

hold(2)

# ------------------------------------------------------------
# STAGE 17
# ------------------------------------------------------------

move_arm_pose(
    FORWARD_PRESENTATION,
    3.0,
    "STAGE 17 : RETURN TO PRESENTATION"
)

# ------------------------------------------------------------
# STAGE 18
# ------------------------------------------------------------

move_arm_pose(
    arm_initial_pose,
    4.0,
    "STAGE 18 : RETURN HOME"
)

# ------------------------------------------------------------
# STAGE 19
# ------------------------------------------------------------

print("\nSTAGE 19 : RELEASE ARM SDK")

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

left_hand_pose()
right_hand_pose()

for _ in range(100):

    left_hand_pub.Write(
        build_hand_cmd(left_hand_q)
    )

    right_hand_pub.Write(
        build_hand_cmd(right_hand_q)
    )

    time.sleep(0.02)

print()
print("=" * 60)
print("DEMO COMPLETE")
print("=" * 60)