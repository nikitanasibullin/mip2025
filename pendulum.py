import os
import time

import numpy as np
import pybullet as p
import pybullet_data


GUI = True
DT = 1.0 / 240.0
GRAVITY = 9.81

MOVE_DURATION = 4.0
HOLD_DURATION = 2.0

# Цель в мировой системе координат.
# Она должна быть достижимой для длины двух звеньев.
TARGET_POSITION = np.array([0.70, 0.0, 2.65], dtype=float)

# Стартовая конфигурация двух суставов.
INITIAL_JOINT_POSITIONS = [0.0, 0.0]

MAX_FORCE = 250.0
POSITION_GAIN = 0.35
VELOCITY_GAIN = 1.0


def load_robot():
    urdf_path = os.path.join(os.path.dirname(__file__), "two-link.urdf.xml")
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)

    movable_joints = []
    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_REVOLUTE:
            movable_joints.append(joint_index)

    if len(movable_joints) != 2:
        raise RuntimeError(f"Expected 2 revolute joints, got {len(movable_joints)}")

    # Последний joint в URDF — это fixed joint для маркера eef.
    end_effector_link = p.getNumJoints(robot_id) - 1
    return robot_id, movable_joints, end_effector_link


def draw_target(target_position):
    x, y, z = target_position.tolist()
    s = 0.05
    color = [1, 0, 0]
    p.addUserDebugLine([x - s, y, z], [x + s, y, z], color, 2.5, 0)
    p.addUserDebugLine([x, y - s, z], [x, y + s, z], color, 2.5, 0)
    p.addUserDebugLine([x, y, z - s], [x, y, z + s], color, 2.5, 0)


def apply_position_control(robot_id, joint_ids, joint_positions):
    for joint_index in joint_ids:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=float(joint_positions[joint_index]),
            force=MAX_FORCE,
            positionGain=POSITION_GAIN,
            velocityGain=VELOCITY_GAIN,
        )


def main():
    p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -GRAVITY)
    p.setTimeStep(DT)

    p.loadURDF("plane.urdf")
    robot_id, joint_ids, end_effector_link = load_robot()

    for joint_index, q0 in zip(joint_ids, INITIAL_JOINT_POSITIONS):
        p.resetJointState(robot_id, joint_index, q0)
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=q0,
            force=0,
        )

    for _ in range(120):
        p.stepSimulation()
        if GUI:
            time.sleep(DT)

    draw_target(TARGET_POSITION)

    start_ee = np.array(
        p.getLinkState(robot_id, end_effector_link, computeForwardKinematics=True)[0],
        dtype=float,
    )

    total_steps = int(MOVE_DURATION / DT)
    hold_steps = int(HOLD_DURATION / DT)

    try:
        for step in range(total_steps):
            alpha = (step + 1) / total_steps
            desired_ee = (1.0 - alpha) * start_ee + alpha * TARGET_POSITION

            joint_positions = p.calculateInverseKinematics(
                robot_id,
                end_effector_link,
                desired_ee.tolist(),
                maxNumIterations=100,
                residualThreshold=1e-6,
            )

            apply_position_control(robot_id, joint_ids, joint_positions)
            p.stepSimulation()

            if step % 120 == 0 or step == total_steps - 1:
                ee_pos = np.array(
                    p.getLinkState(
                        robot_id, end_effector_link, computeForwardKinematics=True
                    )[0],
                    dtype=float,
                )
                err = np.linalg.norm(TARGET_POSITION - ee_pos)
                print(
                    f"step={step:4d} ee={ee_pos.round(4)} "
                    f"target={TARGET_POSITION.round(4)} err={err:.6f}"
                )

            if GUI:
                time.sleep(DT)

        for _ in range(hold_steps):
            joint_positions = p.calculateInverseKinematics(
                robot_id,
                end_effector_link,
                TARGET_POSITION.tolist(),
                maxNumIterations=100,
                residualThreshold=1e-6,
            )
            apply_position_control(robot_id, joint_ids, joint_positions)
            p.stepSimulation()
            if GUI:
                time.sleep(DT)

        ee_pos = np.array(
            p.getLinkState(robot_id, end_effector_link, computeForwardKinematics=True)[0],
            dtype=float,
        )
        err = np.linalg.norm(TARGET_POSITION - ee_pos)
        print(f"Final ee position: {ee_pos.round(4)}")
        print(f"Final error: {err:.6f}")

        if GUI:
            time.sleep(3)

    finally:
        p.disconnect()


if __name__ == "__main__":
    main()