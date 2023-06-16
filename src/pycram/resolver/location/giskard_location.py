from pycram.external_interfaces.giskard import achieve_cartesian_goal
from pycram.designators.location_designator import CostmapLocation
from pycram.bullet_world import Use_shadow_world, BulletWorld
from pycram.helper import _apply_ik

import tf
import numpy as np


def costmap_giskard_resolver(desig):
    if desig.reachable_for:
        giskard_result = achieve_cartesian_goal(desig.target, "r_wrist_roll_link", "map")
        joints = giskard_result.trajectory.joint_names
        trajectory_points = giskard_result.trajectory.points

        end_config = dict(zip(joints, trajectory_points[-1].positions) )
        orientation = list(tf.transformations.quaternion_from_euler(0, 0, end_config["odom_yaw"], axes="sxyz"))
        pose = [[end_config["odom_x"], end_config["odom_y"], 0], orientation]

        used_joints = {}
        for joint, joint_pose in end_config.items():
            if "odom" in joint:
                continue
            if end_config[joint] > 0:
                used_joints[joint] = joint_pose
        test_robot = BulletWorld.current_bullet_world.get_shadow_object(BulletWorld.robot)
        with Use_shadow_world():
            BulletWorld.robot.set_position_and_orientation(pose[0], pose[1])
            _apply_ik(BulletWorld.robot, list(used_joints.values()), list(used_joints.keys()))
            end_effector_pose = np.array(BulletWorld.robot.get_link_position("r_wrist_roll_link"))
            print(end_effector_pose)
            print(pose)
            dist = np.linalg.norm(np.array(desig.target[0]) - end_effector_pose)
            print(dist)

