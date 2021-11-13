import rospy
import tf
import actionlib
from pycram.process_module import ProcessModule
from pycram.helper import list2pose

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID, GoalStatus

class HSRRealNavigation(ProcessModule):
    def _execute(self, desig):
        def active_callback():
            rospy.loginfo("Start Navigating")
        def feedback_callback(msg):
            pass
        def done_callback(state, result):
            rospy.loginfo("Finished Navigation")
            for k in GoalStatus.__dict__.keys():
                if state == GoalStatus.__dict__[k]:
                    rospy.loginfo(f"Navigation has Finished with the result: {k}")
        solution = desig.reference()
        if solution['cmd'] == 'navigate':
            goal = MoveBaseGoal()
            pose = list2pose([solution['target'], solution['orientation']])
            goal.target_pose.pose = pose
            #goal.target_pose.pose.orientation.w = 1
            #goal.target_pose.pose.position.x = 1
            #goal.target_pose.pose.position.y = 1
            goal.target_pose.header.frame_id = "map"
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for action server")
            client.wait_for_server()
            client.send_goal(goal,active_cb=active_callback, done_cb=done_callback, feedback_cb=feedback_callback)
            wait = client.wait_for_result()

HSRProcessModuleSimulated = {}
HSRProcessModuleReal = {'navigate': HSRRealNavigation()}

def available_process_modules(desig):
    robot_name = robot_description.i.name
    robot_type = ProcessModule.robot_type
    type = desig.prop_value('cmd')

    if robot_name == 'hsr':
        if robot_type == 'simulated':
            return HSRProcessModulesSimulated[type]
        elif robot_type == 'real':
            return HSRProcessModulesReal[type]
        elif robot_type == "":
            rospy.logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            rospy.logerr(f"No Process Module could be found for robot {robot_name}")

ProcessModule.resolvers.append(available_process_modules)
