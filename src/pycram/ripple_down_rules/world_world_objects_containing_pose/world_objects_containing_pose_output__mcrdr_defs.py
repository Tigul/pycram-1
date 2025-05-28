from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.world_entity import PhysicalBody
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World
from typing import Dict
from typing import List
from typing_extensions import Union


def conditions_337033937356385007460072279108940985389(case):
    def conditions_for_world_objects_containing_pose(self_: BulletWorld, pose: PoseStamped, output_: PhysicalBody) -> bool:
        """Get conditions on whether it's possible to conclude a value for World_objects_containing_pose.output_  of type PhysicalBody."""
        return len(World.current_world.objects) >= 2
    return conditions_for_world_objects_containing_pose(**case)


def conclusion_337033937356385007460072279108940985389(case):
    def world_objects_containing_pose(self_: BulletWorld, pose: PoseStamped, output_: PhysicalBody) -> List[PhysicalBody]:
        """Get possible value(s) for World_objects_containing_pose.output_  of type PhysicalBody."""
        # Write your code here
        result = []
        for obj in self_.objects:
            for link in obj.links.values():
                if link.get_axis_aligned_bounding_box().contains(*pose.position.to_list()):
                    result.append(link)
        return result
    return world_objects_containing_pose(**case)


