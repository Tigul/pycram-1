import actionlib

from actionlib import SimpleActionClient

def create_action_client(name_space: str, action_message) -> SimpleActionClient:
    return actionlib.SimpleActionClient(name_space, action_message)

