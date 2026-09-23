"""
Manual gait controller for a 12-DOF quadruped robot, simulated in PyBullet.

Written as the fallback controller for the SpiderBot project: if the team's
reinforcement-learning policy failed, this scripted crawl gait could still walk
the robot. Hold the UP arrow key in the simulation window to walk forward.

Joint naming: L<leg>_J<joint>, legs 1-4, joints 1-3 (hip swing, thigh, knee).

Author: Faycal Toumi
"""

import time

import pybullet as p
import pybullet_data

# --- Robot description -------------------------------------------------------

JOINTS_ID = [
    "L1_J1", "L1_J2", "L1_J3",
    "L2_J1", "L2_J2", "L2_J3",
    "L3_J1", "L3_J2", "L3_J3",
    "L4_J1", "L4_J2", "L4_J3",
]

# Neutral standing pose the gait offsets are applied on top of.
# L2 uses an inverted thigh axis, hence the opposite sign on L2_J2.
STANDING_STANCE = {
    "L1_J1": 0.0, "L1_J2": -0.5, "L1_J3": -0.5,   # front left
    "L2_J1": 0.0, "L2_J2": 0.6,  "L2_J3": -0.5,   # back left (inverted thigh axis)
    "L3_J1": 0.0, "L3_J2": -0.5, "L3_J3": 0.7,    # front right
    "L4_J1": 0.0, "L4_J2": -0.5, "L4_J3": 0.1,    # back right
}

# --- Gait tuning -------------------------------------------------------------

LIFT_J2 = 0.40          # how far the thigh lifts a leg off the ground (rad)
SWING_J1 = 0.30         # how far the hip swings a leg forward/backward (rad)
TICKS_PER_STATE = 25    # simulation steps spent in each gait state
SMOOTHING = 0.08        # 0-1: how quickly current offsets approach their target
JOINT_FORCE = 4.0       # holding torque per joint
MAX_VELOCITY = 3.0
TIME_STEP = 1.0 / 240.0
LATERAL_FRICTION = 1.5
NUM_STATES = 10


def build_joint_index(body, client):
    """Map joint names from the URDF to the joint indices PyBullet uses."""
    mapping = {}
    for i in range(p.getNumJoints(body, physicsClientId=client)):
        name = p.getJointInfo(body, i, physicsClientId=client)[1].decode("utf-8")
        mapping[name] = i
    return mapping


def apply_gait_state(state, target_offsets):
    """Set the joint targets for one state of the 10-state crawl cycle.

    States 0-3 step the two left legs forward one at a time, state 4 strokes the
    body forward, states 5-8 do the same on the right side, and state 9 returns
    every hip to neutral, which pushes the body forward again and restarts the
    cycle. Only the joints that change are written: everything else keeps the
    value it already holds, which is what makes the gait continuous.
    """
    if state == 0:      # lift back-left leg and swing it forward
        target_offsets["L2_J2"] = -LIFT_J2
        target_offsets["L2_J1"] = -SWING_J1
    elif state == 1:    # plant it
        target_offsets["L2_J2"] = 0.0
    elif state == 2:    # lift front-left leg and swing it forward
        target_offsets["L1_J2"] = LIFT_J2
        target_offsets["L1_J1"] = -SWING_J1
    elif state == 3:    # plant it
        target_offsets["L1_J2"] = 0.0
    elif state == 4:    # first body push: left legs stroke back, right legs match
        target_offsets["L2_J1"] = SWING_J1
        target_offsets["L1_J1"] = SWING_J1
        target_offsets["L4_J1"] = -SWING_J1
        target_offsets["L3_J1"] = -SWING_J1
    elif state == 5:    # lift back-right leg and swing it forward
        target_offsets["L4_J2"] = LIFT_J2
        target_offsets["L4_J1"] = SWING_J1
    elif state == 6:    # plant it
        target_offsets["L4_J2"] = 0.0
    elif state == 7:    # lift front-right leg and swing it forward
        target_offsets["L3_J2"] = LIFT_J2
        target_offsets["L3_J1"] = SWING_J1
    elif state == 8:    # plant it
        target_offsets["L3_J2"] = 0.0
    elif state == 9:    # second body push and cycle reset
        for leg in ("L1_J1", "L2_J1", "L3_J1", "L4_J1"):
            target_offsets[leg] = 0.0


def main():
    client = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    floor = p.loadURDF("plane.urdf", physicsClientId=client)
    spider = p.loadURDF("urdf/Spider_URDF.urdf", [0, 0, 0.25], physicsClientId=client)

    # Without enough friction the feet slide and the gait goes nowhere.
    p.changeDynamics(floor, -1, lateralFriction=LATERAL_FRICTION)
    for i in range(p.getNumJoints(spider, physicsClientId=client)):
        p.changeDynamics(spider, i, lateralFriction=LATERAL_FRICTION)

    joint_name_to_id = build_joint_index(spider, client)

    for name in JOINTS_ID:
        if name in joint_name_to_id:
            p.resetJointState(
                spider, joint_name_to_id[name],
                STANDING_STANCE.get(name, 0.0), physicsClientId=client,
            )

    # These live outside the loop on purpose. When they were rebuilt every
    # iteration, each leg forgot its target as soon as its state ended, the legs
    # snapped back to neutral mid-stride and the robot bounced off the ground.
    target_offsets = {name: 0.0 for name in JOINTS_ID}
    current_offsets = {name: 0.0 for name in JOINTS_ID}

    step_timer = 0
    current_state = 0

    print("\n>>> Hold the UP arrow key to walk forward. Ctrl+C to quit. <<<\n")

    try:
        while True:
            keys = p.getKeyboardEvents()
            moving = (
                p.B3G_UP_ARROW in keys
                and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN
            )

            if moving:
                step_timer += 1
                if step_timer > TICKS_PER_STATE:
                    step_timer = 0
                    current_state = (current_state + 1) % NUM_STATES
                apply_gait_state(current_state, target_offsets)

            # Ease each joint toward its target instead of jumping to it:
            # a step change would make the physics engine pop the robot.
            for name in JOINTS_ID:
                if name not in joint_name_to_id:
                    continue
                current_offsets[name] += (
                    target_offsets[name] - current_offsets[name]
                ) * SMOOTHING
                p.setJointMotorControl2(
                    bodyUniqueId=spider,
                    jointIndex=joint_name_to_id[name],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=STANDING_STANCE.get(name, 0.0) + current_offsets[name],
                    force=JOINT_FORCE,
                    maxVelocity=MAX_VELOCITY,
                    physicsClientId=client,
                )

            p.stepSimulation(client)
            time.sleep(TIME_STEP)

    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
