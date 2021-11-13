from pycram.action_designator import NavigateDescription
from pycram.motion_designator import *
from pycram.designator import DesignatorError
from pycram.motion_designator import MotionDesignator, MoveMotionDescription
#from pycram.knowrob import find_shelf_pose, get_all_shelves


def ground_navigating(self):
    if type(self.target_location) == str and "shelf" in self.target_location:
        self.function = lambda: nav_plan(self)
    else:
        DesignatorError()
    return super(NavigateDescription, self).ground()

def nav_plan(desig):
    shelf = desig.target_location
    all_shelves = get_all_shelves()
    shelf_knowrob = all_shelves[shelf.replace("shelf", "")]
    shelf_pose = find_shelf_pose(shelf_knowrob)

    MotionDesignator(MoveMotionDescription(target=shelf_pose[0], orientation=shelf_pose[1])).perfrom()

NavigateDescription.ground = ground_navigating
