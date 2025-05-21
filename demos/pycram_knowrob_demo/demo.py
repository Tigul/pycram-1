from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.designators.action_designator import TransportActionDescription, SearchActionDescription
from pycram.designators.location_designator import KnowledgeLocation
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object

from pycrap.ontologies import Robot, Milk

world = BulletWorld(WorldMode.GUI)

pr2 = Object("pr2", Robot, "pr2.urdf")

plan = SequentialPlan(
    TransportActionDescription(
        SearchActionDescription(KnowledgeLocation(Milk)),
    PoseStamped.from_list([4, 5, 1])),
)

with simulated_robot:
    plan.perform()