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

def move_pose(
    q0=0.0,
    right=0.0,
    left=0.0,
    hold=1.0,
    label=None
):

    global current_cmd

    pose = [
        q0,
        0.0,
        0.0,
        right,
        right,
        left,
        left,
    ]

    if label:

        print()
        print("=" * 60)
        print(label)
        print("=" * 60)

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
                f"L=({s[5]:.3f},{s[6]:.3f})"
            )

# ============================================================
# SCENE 1
# ACTIVATION
# ============================================================

def scene_activation():

    move_pose(
        q0=0,
        right=0,
        left=0,
        hold=2.0,
        label="SCENE 1 : ACTIVATION"
    )

    move_pose(
        q0=1,
        hold=1.5
    )

    move_pose(
        q0=0,
        hold=1.0
    )

    move_pose(
        q0=-1,
        hold=1.5
    )

    move_pose(
        q0=0,
        hold=1.5
    )

# ============================================================
# SCENE 2
# FINGER DISCOVERY
# ============================================================

def scene_discovery():

    move_pose(
        right=1,
        hold=1.0,
        label="SCENE 2 : FINGER DISCOVERY"
    )

    move_pose(
        right=0,
        hold=0.8
    )

    move_pose(
        right=1,
        hold=1.0
    )

    move_pose(
        right=0,
        hold=0.8
    )

    move_pose(
        left=1,
        hold=1.0
    )

    move_pose(
        left=0,
        hold=0.8
    )

    move_pose(
        left=1,
        hold=1.0
    )

    move_pose(
        left=0,
        hold=0.8
    )

# ============================================================
# SCENE 3
# DEXTERITY EXERCISE
# ============================================================

def scene_dexterity():

    print()
    print("=" * 60)
    print("SCENE 3 : DEXTERITY EXERCISE")
    print("=" * 60)

    for _ in range(3):

        move_pose(
            right=1,
            hold=0.5
        )

        move_pose(
            left=1,
            hold=0.5
        )

        move_pose(
            right=1,
            left=1,
            hold=0.5
        )

        move_pose(
            right=0,
            left=0,
            hold=0.5
        )

# ============================================================
# SCENE 4
# PINCH ILLUSIONS
# ============================================================

def scene_pinch():

    print()
    print("=" * 60)
    print("SCENE 4 : PINCH ILLUSIONS")
    print("=" * 60)

    move_pose(
        q0=1,
        hold=1.0
    )

    move_pose(
        q0=1,
        right=1,
        hold=1.5
    )

    move_pose(
        q0=1,
        right=0,
        hold=0.8
    )

    move_pose(
        q0=0,
        hold=0.8
    )

    move_pose(
        q0=-1,
        hold=1.0
    )

    move_pose(
        q0=-1,
        left=1,
        hold=1.5
    )

    move_pose(
        q0=-1,
        left=0,
        hold=0.8
    )

    move_pose(
        q0=0,
        hold=0.8
    )

# ============================================================
# SCENE 5
# POWER GRASP
# ============================================================

def scene_power_grasp():

    print()
    print("=" * 60)
    print("SCENE 5 : POWER GRASP")
    print("=" * 60)

    for _ in range(2):

        move_pose(
            q0=-1,
            right=1,
            left=1,
            hold=2.0
        )

        move_pose(
            q0=0,
            right=0,
            left=0,
            hold=1.5
        )

# ============================================================
# SCENE 6
# FINGER WAVE
# ============================================================

def scene_wave():

    print()
    print("=" * 60)
    print("SCENE 6 : FINGER WAVE")
    print("=" * 60)

    for _ in range(2):

        move_pose(
            right=1,
            hold=0.6
        )

        move_pose(
            right=1,
            left=1,
            hold=0.6
        )

        move_pose(
            left=1,
            hold=0.6
        )

        move_pose(
            right=0,
            left=0,
            hold=0.6
        )

    for _ in range(2):

        move_pose(
            left=1,
            hold=0.6
        )

        move_pose(
            right=1,
            left=1,
            hold=0.6
        )

        move_pose(
            right=1,
            hold=0.6
        )

        move_pose(
            right=0,
            left=0,
            hold=0.6
        )

# ============================================================
# SCENE 7
# FINALE
# ============================================================

def scene_finale():

    print()
    print("=" * 60)
    print("SCENE 7 : FINALE")
    print("=" * 60)

    move_pose(
        right=1,
        hold=0.4
    )

    move_pose(
        right=0,
        hold=0.2
    )

    move_pose(
        left=1,
        hold=0.4
    )

    move_pose(
        left=0,
        hold=0.2
    )

    move_pose(
        right=1,
        hold=0.4
    )

    move_pose(
        right=0,
        hold=0.2
    )

    move_pose(
        left=1,
        hold=0.4
    )

    move_pose(
        left=0,
        hold=0.2
    )

    move_pose(
        right=1,
        left=1,
        hold=1.0
    )

    move_pose(
        q0=1,
        right=1,
        left=1,
        hold=1.0
    )

    move_pose(
        q0=-1,
        right=1,
        left=1,
        hold=1.0
    )

    move_pose(
        q0=0,
        right=0,
        left=0,
        hold=4.0,
        label="FINAL OPEN HAND"
    )

# ============================================================
# DEMO V2
# ============================================================

def demo_v2():

    print()
    print("=" * 60)
    print("DEX3 HAND DEMO V2")
    print("=" * 60)
    print()
    print("Current verified model:")
    print("q0      = thumb rotation")
    print("q3+q4   = RIGHT finger")
    print("q5+q6   = LEFT finger")
    print()

    scene_activation()

    scene_discovery()

    scene_dexterity()

    scene_pinch()

    scene_power_grasp()

    scene_wave()

    scene_finale()

# ============================================================
# MAIN
# ============================================================

print()
print("=" * 60)
print("DEX3 HAND DEMO V2")
print("=" * 60)
print()

input("Robot READY mode. Press ENTER...")

wait_for_state()

try:

    demo_v2()

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
