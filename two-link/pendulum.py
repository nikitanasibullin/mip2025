import os
import time

import numpy as np
import pybullet as p
import pybullet_data


GUI = True
DT = 1.0 / 240.0
GRAVITY = 9.81

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

    # lifeTime=0.1, чтобы маркер обновлялся каждый кадр и не копился
    p.addUserDebugLine([x - s, y, z], [x + s, y, z], color, 2.5, 0.1)
    p.addUserDebugLine([x, y - s, z], [x, y + s, z], color, 2.5, 0.1)
    p.addUserDebugLine([x, y, z - s], [x, y, z + s], color, 2.5, 0.1)


def apply_position_control(robot_id, joint_ids, joint_positions):
    for i, joint_id in enumerate(joint_ids):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=float(joint_positions[i]),
            force=MAX_FORCE,
            positionGain=POSITION_GAIN,
            velocityGain=VELOCITY_GAIN,
        )


def main():
    p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -GRAVITY)
    p.setTimeStep(DT)
    p.resetDebugVisualizerCamera(
    cameraDistance=3.5,
    cameraYaw=0,
    cameraPitch=0,
    cameraTargetPosition=[0, 0, 2]
)

    # Ползунки для задания целевой декартовой точки
    slider_x = p.addUserDebugParameter("target_x", -2, 2, 0.7)
    slider_z = p.addUserDebugParameter("target_z", 0, 3, 2.65)

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

    try:
        while True:
            target_position = np.array(
                [
                    p.readUserDebugParameter(slider_x),
                    0.0,
                    p.readUserDebugParameter(slider_z),
                ],
                dtype=float,
            )

            draw_target(target_position)

            joint_positions = p.calculateInverseKinematics(
                robot_id,
                end_effector_link,
                target_position.tolist(),
                maxNumIterations=100,
                residualThreshold=1e-6,
            )

            apply_position_control(robot_id, joint_ids, joint_positions)
            p.stepSimulation()

            ee_pos = np.array(
                p.getLinkState(
                    robot_id, end_effector_link, computeForwardKinematics=True
                )[0],
                dtype=float,
            )
            err = np.linalg.norm(target_position - ee_pos)

            print(
                f"ee={ee_pos.round(4)} "
                f"target={target_position.round(4)} "
                f"err={err:.6f}"
            )

            if GUI:
                time.sleep(DT)

    finally:
        p.disconnect()


if __name__ == "__main__":
    main()