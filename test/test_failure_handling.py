import unittest

import roslaunch

from pycram.bullet_world import BulletWorld, Object
from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ParkArmsAction
from pycram.enums import ObjectType, Arms
from pycram.failure_handling import Retry
from pycram.plan_failures import PlanFailure
from pycram.process_module import ProcessModule, simulated_robot
from pycram.robot_descriptions import robot_description
from bullet_world_testcase import BulletWorldTestCase


# start ik_and_description.launch
class DummyActionDesignator(ActionDesignatorDescription):
    class Action(ActionDesignatorDescription.Action):
        def perform(self):
            raise PlanFailure("Dummy action failed")

    def __iter__(self):
        for _ in range(100):
            yield self.Action()


class FailureHandlingTest(BulletWorldTestCase):
    process: roslaunch.scriptapi.ROSLaunch

    def test_retry_with_success(self):
        with simulated_robot:
            Retry(ParkArmsAction([Arms.LEFT]), max_tries=5).perform()

    def test_retry_with_failure(self):
        with simulated_robot:
            with self.assertRaises(PlanFailure):
                Retry(DummyActionDesignator(), max_tries=5).perform()


if __name__ == '__main__':
    unittest.main()
