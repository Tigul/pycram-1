import unittest

from pycram.bullet_world import BulletWorld, Object, fix_missing_inertial
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.enums import ObjectType


class BulletWorldTestCase(unittest.TestCase):

    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "apartment.urdf")
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.4, 3, 1.03]))
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([2.4, 2.8, 1.05]))
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_bullet_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()



