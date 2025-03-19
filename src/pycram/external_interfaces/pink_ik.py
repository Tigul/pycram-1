from pink import solve_ik
from pink.barriers import PositionBarrier
from pink.tasks import FrameTask, PostureTask
from pink import Configuration
from loop_rate_limiters import RateLimiter
import numpy as np
from typing_extensions import Dict

from .pinocchio_ik import create_joint_configuration, parse_configuration_vector_to_joint_positions

import pinocchio
import qpsolvers

from ..datastructures.pose import Pose
from ..failures import IKError
from ..world_concepts.world_object import Object

def compute_ik(target_link: str, target_pose: Pose, robot: Object) -> Dict[str, float]:
    model = pinocchio.buildModelFromUrdf(robot.path)
    data = model.createData()
    q = create_joint_configuration(robot, model)

    configuration = Configuration(model, data, q)

    end_effector_task = FrameTask(
        target_link,
        position_cost=10.0,  # [cost] / [m]
        orientation_cost=1.0,  # [cost] / [rad]
    )

    #end_effector_task.set_target_from_configuration(configuration)
    #end_effector_task.transform_target_to_world.translation[1] -= 0.1

    end_effector_task.transform_target_to_world = pinocchio.XYZQUATToSe3(np.array(target_pose.position_as_list() + target_pose.orientation_as_list()))

    for i in range(1000):
        solver = qpsolvers.available_solvers[0]

        rate = RateLimiter(frequency=200.0)
        dt = rate.period
        velocity = solve_ik(
            configuration,
            [end_effector_task],
            dt,
            solver=solver,
        )

        configuration.integrate_inplace(velocity, dt)

        err = end_effector_task.compute_error(configuration)
        print(f"error: {np.linalg.norm(err)} in iteration: {i}")
        if np.linalg.norm(err) < 1e-4:
            return parse_configuration_vector_to_joint_positions(configuration.q, model)
    print(f"Failed with error {np.linalg.norm(end_effector_task.compute_error(configuration))}")
    raise IKError(pinocchio.SE3ToXYZQUAT(end_effector_task.transform_target_to_world), robot.tf_frame, target_link)



