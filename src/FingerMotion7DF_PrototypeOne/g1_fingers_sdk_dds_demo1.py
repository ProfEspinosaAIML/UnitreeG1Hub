#!/usr/bin/env python3

import sys
import time
import threading

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_,
    HandState_,
    MotorCmd_,
)

# ============================================================
# DDS
# ============================================================

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

CMD_TOPIC = "rt/dex3/right/cmd"
STATE_TOPIC = "rt/lf/dex3/right/state"

latest_state = None
running = True

# ============================================================
# PUBLISHER
# ============================================================

pub = ChannelPublisher(
    CMD_TOPIC,
    HandCmd_
)
pub.Init()

# ============================================================
# SUBSCRIBER
# ============================================================

def state_callback(msg):

    global latest_state
    latest_state = msg

sub = ChannelSubscriber(
    STATE_TOPIC,
    HandState_
)

sub.Init(
    state_callback,
    10
)

# ============================================================
# COMMAND CREATION
# ============================================================

def make_cmd(q):

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

current_cmd = make_cmd([0] * 7)

# ============================================================
# CONTINUOUS PUBLISHER
# ============================================================

def publisher():

    global running
    global current_cmd

    while running:

        pub.Write(current_cmd)

        time.sleep(0.02)

threading.Thread(
    target=publisher,
    daemon=True
).start()

# ============================================================
# HELPERS
# ============================================================

def wait_for_state():

    print("Waiting for HandState...")

    while latest_state is None:
        time.sleep(0.1)

    print("HandState received.")

def get_state():

    if latest_state is None:
        return None

    return [
        round(
            latest_state.motor_state[i].q,
            4
        )
        for i in range(7)
    ]

def print_state():

    s = get_state()

    if s is not None:
        print(s)

def move_pose(
    q0=0.0,
    right=0.0,
    left=0.0,
    hold=1.0,
    label=None
):

    global current_cmd

    pose = [
        q0,     # thumb rotation
        0.0,    # unknown
        0.0,    # unknown
        right,  # right finger joint A
        right,  # right finger joint B
        left,   # left finger joint A
        left,   # left finger joint B
    ]

    if label is not None:

        print()
        print("-" * 60)
        print(label)
        print("-" * 60)

    current_cmd = make_cmd(pose)

    start = time.time()

    while time.time() - start < hold:

        time.sleep(0.5)

        s = get_state()

        if s is not None:

            print(
                f"t={time.time()-start:4.1f}s",
                f"q0={s[0]:7.4f}",
                f"R=({s[3]:.3f},{s[4]:.3f})",
                f"L=({s[5]:.3f},{s[6]:.3f})",
            )

# ============================================================
# DEMO
# ============================================================

def demo_v0():

    print()
    print("=" * 60)
    print("DEX3 HAND DEMO V0")
    print("=" * 60)
    print()
    print("Working model:")
    print("q0      = thumb rotation")
    print("q3+q4   = RIGHT finger")
    print("q5+q6   = LEFT finger")
    print()

    # --------------------------------------------------------
    # HOME
    # --------------------------------------------------------

    move_pose(
        q0=0.0,
        right=0.0,
        left=0.0,
        hold=2.0,
        label="HOME / OPEN HAND"
    )

    # --------------------------------------------------------
    # RIGHT FINGER
    # --------------------------------------------------------

    move_pose(
        right=1.0,
        hold=1.5,
        label="RIGHT FINGER CLOSE"
    )

    move_pose(
        right=0.0,
        hold=1.0,
        label="RIGHT FINGER OPEN"
    )

    # --------------------------------------------------------
    # LEFT FINGER
    # --------------------------------------------------------

    move_pose(
        left=1.0,
        hold=1.5,
        label="LEFT FINGER CLOSE"
    )

    move_pose(
        left=0.0,
        hold=1.0,
        label="LEFT FINGER OPEN"
    )

    # --------------------------------------------------------
    # ALTERNATION
    # --------------------------------------------------------

    move_pose(
        right=1.0,
        hold=0.8,
        label="ALTERNATE 1"
    )

    move_pose(
        left=1.0,
        hold=0.8,
        label="ALTERNATE 2"
    )

    move_pose(
        right=0.0,
        hold=0.8,
        label="ALTERNATE 3"
    )

    move_pose(
        left=0.0,
        hold=0.8,
        label="ALTERNATE 4"
    )

    move_pose(
        right=1.0,
        hold=0.8,
        label="ALTERNATE 5"
    )

    move_pose(
        left=1.0,
        hold=0.8,
        label="ALTERNATE 6"
    )

    move_pose(
        right=0.0,
        left=0.0,
        hold=1.0,
        label="OPEN"
    )

    # --------------------------------------------------------
    # THUMB SHOWCASE
    # --------------------------------------------------------

    move_pose(
        q0=1.0,
        hold=2.0,
        label="THUMB CLOCKWISE"
    )

    move_pose(
        q0=0.0,
        hold=1.0,
        label="THUMB CENTER"
    )

    move_pose(
        q0=-1.0,
        hold=2.0,
        label="THUMB COUNTER-CLOCKWISE"
    )

    move_pose(
        q0=0.0,
        hold=1.0,
        label="THUMB CENTER"
    )

    # --------------------------------------------------------
    # GRASP ILLUSION
    # --------------------------------------------------------

    move_pose(
        q0=0.0,
        right=0.0,
        left=0.0,
        hold=1.0,
        label="PREPARE GRASP"
    )

    move_pose(
        right=1.0,
        hold=1.0,
        label="RIGHT FINGER CLOSE"
    )

    move_pose(
        right=1.0,
        left=1.0,
        hold=1.0,
        label="LEFT FINGER CLOSE"
    )

    move_pose(
        q0=-1.0,
        right=1.0,
        left=1.0,
        hold=2.0,
        label="THUMB INWARD / HOLD"
    )

    move_pose(
        q0=0.0,
        right=0.0,
        left=0.0,
        hold=2.0,
        label="RELEASE"
    )

    # --------------------------------------------------------
    # DEXTERITY FINALE
    # --------------------------------------------------------

    move_pose(
        right=1.0,
        hold=0.5,
        label="FINALE 1"
    )

    move_pose(
        right=0.0,
        hold=0.3
    )

    move_pose(
        left=1.0,
        hold=0.5,
        label="FINALE 2"
    )

    move_pose(
        left=0.0,
        hold=0.3
    )

    move_pose(
        right=1.0,
        hold=0.5,
        label="FINALE 3"
    )

    move_pose(
        right=0.0,
        hold=0.3
    )

    move_pose(
        left=1.0,
        hold=0.5,
        label="FINALE 4"
    )

    move_pose(
        left=0.0,
        hold=0.3
    )

    move_pose(
        right=1.0,
        left=1.0,
        hold=1.0,
        label="BOTH FINGERS"
    )

    move_pose(
        q0=1.0,
        right=1.0,
        left=1.0,
        hold=1.0,
        label="THUMB SWEEP"
    )

    move_pose(
        q0=-1.0,
        right=1.0,
        left=1.0,
        hold=1.0
    )

    move_pose(
        q0=0.0,
        right=0.0,
        left=0.0,
        hold=3.0,
        label="FINAL OPEN HAND"
    )

# ============================================================
# MAIN
# ============================================================

print()
print("=" * 60)
print("DEX3 HAND DEMO V0")
print("=" * 60)
print()

input("Robot READY mode. Press ENTER...")

wait_for_state()

print()
print("INITIAL STATE")
print_state()

try:

    demo_v0()

    print()
    print("=" * 60)
    print("DEMO COMPLETE")
    print("=" * 60)
    print()

finally:

    running = False

    current_cmd = make_cmd(
        [0, 0, 0, 0, 0, 0, 0]
    )

    for _ in range(100):

        pub.Write(current_cmd)

        time.sleep(0.02)