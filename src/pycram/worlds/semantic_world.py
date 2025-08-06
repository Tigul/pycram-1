import random
from dataclasses import dataclass, field
from typing import Union

from semantic_world.connections import Connection6DoF, Has1DOFState
from semantic_world.geometry import Box, Color, Scale
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import World
from semantic_world.world_entity import Body, Connection
from typing_extensions import List

from ..datastructures.pose import PoseStamped

@dataclass
class SemanticWorld:

    sem_world: World = field(default_factory=World)
    """
    Instance of the World class that represents the semantic world.
    """
    _vis_axis: List[Body] = field(default_factory=list)
    """
    Keeps track of visualization axes in the semantic world.
    """
    def get_body_pose(self, body: Body, relative_to: Body = None) -> PoseStamped:
        """
        Get the pose of a body in the semantic world relative to another body if specified otherwise in the world frame.
        :param body:
        :param relative_to:
        :return:
        """
        pose_matrix = self.sem_world.compute_forward_kinematics_np(body, relative_to or self.sem_world.root)
        return PoseStamped.from_matrix(pose_matrix, frame_id=body.name.name)

    def set_body_pose(self, body: Body, pose: PoseStamped) -> None:
        """
        Set the pose of a body in the semantic world relative to another body.

        :param body: The body whose pose is to be set.
        :param pose: The new pose for the body.
        """
        connection = self.sem_world.get_connection(self.sem_world.root, body)
        if not isinstance(connection, Connection6DoF):
            raise TypeError(f"Connection for body {body.name} is not a 6DoF connection.")
        connection.origin = pose.to_matrix()

    def get_joint_position(self, joint_name: str) -> float:
        """
        Get the position of a joint in the semantic world.

        :param joint_name: The name of the joint to retrieve the position for.
        :return: The position of the joint.
        """
        connection = self.sem_world.get_connection_by_name(joint_name)
        if not isinstance(connection, Has1DOFState):
            raise TypeError(f"Connection for joint {joint_name} is not a 1DoF connection.")
        return connection.position

    def set_joint_position(self, joint_name: str, position: float) -> None:
        """
        Set the position of a joint in the semantic world.

        :param joint_name: The name of the joint to set the position for.
        :param position: The new position for the joint.
        """
        connection = self.sem_world.get_connection_by_name(joint_name)
        if not isinstance(connection, Has1DOFState):
            raise TypeError(f"Connection for joint {joint_name} is not a 1DoF connection.")
        connection.position = position

    def get_joint_positions(self, joint_names: List[str]) -> dict[str, float]:
        """
        Get the positions of all joints in a body.

        :param joint_names: A list of joint names to retrieve positions for.
        :return: A dictionary mapping joint names to their positions.
        """
        connections = [self.sem_world.get_connection_by_name(name) for name in joint_names]
        for connection in connections:
            if not isinstance(connection, Has1DOFState):
                raise TypeError(f"Connection for joint {connection.name} is not a 1DoF connection.")
        return {connection.name.name: connection.position for connection in connections}

    def set_joint_positions(self, joint_positions: dict[str, float]) -> None:
        """
        Set the positions of multiple joints in the semantic world.

        :param joint_positions: A dictionary mapping joint names to their new positions.
        """
        for joint_name, position in joint_positions.items():
            connection = self.sem_world.get_connection_by_name(joint_name)
            if not isinstance(connection, Has1DOFState):
                raise TypeError(f"Connection for joint {joint_name} is not a 1DoF connection.")
            connection.position = position


    def get_inverse_kinematics(self, root_body: Body, tip_body: Body, target_pose: PoseStamped) -> dict[str, float]:
        """
        Compute the inverse kinematics for a given root body and tip body to achieve a target pose.
        :param root_body: The body representing the root of the kinematic chain.
        :param tip_body: The body representing the tip of the kinematic chain.
        :param target_pose: The desired pose for the tip body.
        :return: A dictionary mapping joint names to their computed positions.
        """
        target_matrix = target_pose.to_matrix()
        ik =  self.sem_world.compute_inverse_kinematics(root_body, tip_body, target_matrix)
        return {joint_name.name: position for joint_name, position in ik}

    def  transform_pose(self, pose: PoseStamped, target_frame: Body) -> PoseStamped:
        """
        Transform a pose from the world frame to the target frame.

        :param pose: The pose to transform.
        :param target_frame: The body representing the target frame.
        :return: The transformed pose in the target frame.
        """
        target_matrix = pose.to_matrix()
        transformed_matrix = self.sem_world.compute_relative_pose(target_matrix, target_frame, self.sem_world.get_body_by_name(pose.frame_id))
        return PoseStamped.from_matrix(transformed_matrix, frame_id=target_frame.name.name)

    def add_body(self, body: Body) -> None:
        """
        Add a body to the semantic world.

        :param body: The body to add.
        """
        self.sem_world.add_body(body)

    def remove_body(self, body: Body) -> None:
        """
        Remove a body from the semantic world.
        :param body:
        :return:
        """
        self.sem_world.remove_body(body)

    def add_connection(self, connection: Connection) -> None:
        """
        Add a connection to the semantic world.
        :param connection: The connection to add.
        """
        self.sem_world.add_connection(connection)

    def get_body_by_name(self, name: str) -> Body:
        """
        Get a body by its name from the semantic world.
        :param name: The name of the body to retrieve.
        :return: The body with the specified name.
        """
        return self.sem_world.get_body_by_name(name)

    def get_connection_by_name(self, name: str) -> Connection:
        """
        Get a connection by its name from the semantic world.
        :param name: The name of the connection to retrieve.
        :return: The connection with the specified name.
        """
        return self.sem_world.get_connection_by_name(name)

    @property
    def bodies(self):
        """
        Get all bodies in the semantic world.
        :return: A list of all bodies in the semantic world.
        """
        return self.sem_world.bodies

    @property
    def bodies_with_collisions(self):
        """
        Get all bodies in the semantic world that have collision enabled.
        :return: A list of all bodies with collision enabled.
        """
        return self.sem_world.bodies_with_collisions

    @property
    def connections(self):
        """
        Get all connections in the semantic world.
        :return: A list of all connections in the semantic world.
        """
        return self.sem_world.connections

    def save_state(self):
        """
        Save the current state of the semantic world.
        :return: The current state of the semantic world.
        """
        pass

    def restore_state(self, state):
        """
        Restore the semantic world to a previously saved state.
        :param state: The state to restore to.
        """
        pass

    def reset_world(self):
        """
        Reset the semantic world to its initial state.
        """
        pass

    def set_initial_state(self) -> None:
        """
        Set the initial state of the semantic world to the current state.
        """
        pass

    def merge_world(self, other_world: Union[World, SemanticWorld], merge_pose: PoseStamped) -> None:
        """
        Merge another semantic world into this one.

        :param other_world: The semantic world to merge into this one.
        :param merge_pose: The pose to merge into the other one.
        """
        self.sem_world.merge_world(other_world)

    def merge_world_with_pose(self, other_world: World, pose: PoseStamped) -> None:
        """
        Merge another semantic world into this one at a specific pose.

        :param other_world: The semantic world to merge into this one.
        :param pose: The pose at which to merge the other world.
        """
        self.sem_world.merge_world_with_pose(other_world, pose.to_matrix())

    def add_vis_axis(self, pose: PoseStamped) -> None:
        body = Body(name=PrefixedName("viz_axis" + str(random.randint(0, 100))))
        x_axis = Box(color=Color(1, 0, 0, 1), origin=TransformationMatrix.from_xyz_rpy(x=0.5), scale=Scale(1, 0.05, 0.05))
        y_axis = Box(color=Color(0, 1, 0, 1), origin=TransformationMatrix.from_xyz_rpy(y=0.5), scale=Scale(0.05, 1, 0.05))
        z_axis = Box(color=Color(0, 0, 1, 1), origin=TransformationMatrix.from_xyz_rpy(z=0.5), scale=Scale(0.05, 0.05, 1))
        body.visual = [x_axis, y_axis, z_axis]
        self.add_body(body)
        connection = Connection6DoF(parent=self.sem_world.root, child=body, _world=self.sem_world)
        connection.origin = pose.to_matrix()
        self.add_connection(connection)

    def remove_all_vis_axes(self) -> None:
        """
        Remove all visualization axes from the semantic world.
        """
        for body in self._vis_axis:
            self.sem_world.remove_body(body)
        self._vis_axis.clear()


