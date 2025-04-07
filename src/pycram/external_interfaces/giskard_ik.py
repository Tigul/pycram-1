from giskardpy.data_types.data_types import PrefixName
from giskardpy.model.collision_avoidance_config import DisableCollisionAvoidanceConfig
from giskardpy.model.trajectory import Trajectory
from giskardpy.model.world_config import WorldWithOmniDriveRobot
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.user_interface import GiskardWrapper
import giskardpy.casadi_wrapper as cas
from typing_extensions import Dict
from giskardpy.qp.qp_solver_ids import SupportedQPSolver


from ..datastructures.pose import Pose
from ..tf_transformations import euler_from_quaternion
from ..world_concepts.world_object import Object


def compute_giskardlib_ik(target_pose: Pose, robot: Object, tip_link: str) -> Dict[str, float]:

    urdf = open('/home/jonas/workspace/ros/src/pycram-1/resources/cached/pr2.urdf', 'r').read()
    giskard = GiskardWrapper(world_config=WorldWithOmniDriveRobot(urdf=urdf),
                             collision_avoidance_config=DisableCollisionAvoidanceConfig(),
                             qp_controller_config=QPControllerConfig(qp_solver=SupportedQPSolver.daqp))

    traj = execute_cart_goal(giskard, target_pose, tip_link)
    end_state = traj.get_last()
    end_config = {joint.short_name: joint_pose.position for joint, joint_pose in end_state.items() if joint.short_name in robot.joints.keys()}
    return end_config



def execute_cart_goal(giskard: GiskardWrapper, pose: Pose, tip_link) -> Trajectory:
    init = 'init'
    g1 = 'g1'
    init_goal1 = cas.TransMatrix(reference_frame=PrefixName('map'))
    init_goal1.x = -0.5

    rotation = euler_from_quaternion(pose.orientation_as_list())

    base_goal2 = cas.TransMatrix.from_xyz_rpy(*pose.position_as_list(), *rotation,
                                              reference_frame=PrefixName("base_footprint", prefix="pr2"),
                                              child_frame=PrefixName(tip_link, prefix='pr2'))


    giskard.monitors.add_set_seed_odometry(base_pose=init_goal1, name=init)
    giskard.motion_goals.add_cartesian_pose(goal_pose=base_goal2, name=g1,
                                            root_link='base_footprint',
                                            tip_link=tip_link,
                                            start_condition=init,
                                            end_condition=g1)
    giskard.monitors.add_end_motion(start_condition=g1)
    return giskard.execute(sim_time=20)