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
# DDS INITIALIZATION
# ============================================================

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)

CMD_TOPIC   = "rt/dex3/right/cmd"
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

current_cmd = make_cmd([0]*7)

# ============================================================
# CONTINUOUS DDS STREAM
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

def move_pose(
    q,
    hold_time,
    title=""
):

    global current_cmd

    if title:
        print(title)

    current_cmd = make_cmd(q)

    t0 = time.time()

    while time.time() - t0 < hold_time:
        time.sleep(0.05)

def open_hand():

    move_pose(
        [0,0,0,0,0,0,0],
        1.0
    )

def close_right():

    move_pose(
        [0,0,0,1,1,0,0],
        0.7
    )

def open_right():

    move_pose(
        [0,0,0,0,0,0,0],
        0.7
    )

def close_left():

    move_pose(
        [0,0,0,0,0,1,1],
        0.7
    )

def open_left():

    move_pose(
        [0,0,0,0,0,0,0],
        0.7
    )

def close_both():

    move_pose(
        [0,0,0,1,1,1,1],
        0.8
    )

def open_both():

    move_pose(
        [0,0,0,0,0,0,0],
        0.8
    )

def thumb_clockwise():

    move_pose(
        [1,0,0,0,0,0,0],
        0.8
    )

def thumb_counterclockwise():

    move_pose(
        [-1,0,0,0,0,0,0],
        0.8
    )

def thumb_center():

    move_pose(
        [0,0,0,0,0,0,0],
        0.8
    )

# ============================================================
# DEMO PHASES
# ============================================================

def phase_wakeup():

    print("\nPHASE 1 : WAKE UP")

    open_hand()

    thumb_clockwise()
    thumb_center()

    thumb_counterclockwise()
    thumb_center()

def phase_finger_identification():

    print("\nPHASE 2 : FINGER IDENTIFICATION")

    close_right()
    open_right()

    close_left()
    open_left()

    close_right()
    open_right()

    close_left()
    open_left()

def phase_alternating():

    print("\nPHASE 3 : ALTERNATING DEXTERITY")

    for _ in range(3):

        close_right()

        move_pose(
            [0,0,0,1,1,1,1],
            0.4
        )

        move_pose(
            [0,0,0,0,0,1,1],
            0.4
        )

        open_both()

def phase_pinch_illusion():

    print("\nPHASE 4 : PINCH ILLUSION")

    move_pose(
        [1,0,0,0,0,0,0],
        0.5
    )

    move_pose(
        [1,0,0,1,1,0,0],
        1.0
    )

    move_pose(
        [1,0,0,0,0,0,0],
        0.6
    )

    move_pose(
        [0,0,0,0,0,0,0],
        0.6
    )

    move_pose(
        [-1,0,0,0,0,0,0],
        0.5
    )

    move_pose(
        [-1,0,0,0,0,1,1],
        1.0
    )

    move_pose(
        [-1,0,0,0,0,0,0],
        0.6
    )

    move_pose(
        [0,0,0,0,0,0,0],
        0.6
    )

def phase_full_grasp():

    print("\nPHASE 5 : FULL GRASP")

    for _ in range(2):

        move_pose(
            [1,0,0,1,1,1,1],
            1.2
        )

        move_pose(
            [0,0,0,0,0,0,0],
            1.0
        )

def phase_finale():

    print("\nPHASE 6 : FINALE")

    close_right()

    close_left()

    close_both()

    thumb_clockwise()

    thumb_counterclockwise()

    move_pose(
        [0,0,0,1,1,1,1],
        0.6
    )

    move_pose(
        [0,0,0,0,0,0,0],
        1.5
    )

# ============================================================
# MAIN
# ============================================================

print()
print("====================================================")
print("G1 RIGHT HAND DEMO v1")
print("====================================================")
print()

input("Robot READY mode. Press ENTER...")

wait_for_state()

try:

    phase_wakeup()

    phase_finger_identification()

    phase_alternating()

    phase_pinch_illusion()

    phase_full_grasp()

    phase_finale()

    print()
    print("Demo complete.")

finally:

    running = False

    current_cmd = make_cmd(
        [0,0,0,0,0,0,0]
    )

    for _ in range(100):

        pub.Write(current_cmd)

        time.sleep(0.02)
