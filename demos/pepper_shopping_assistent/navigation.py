import numpy as np

# The 15 waypoints scattered over the refills lab
waypoints = {'WP1' : np.array([1.8, 0.1, 0]),
             'WP2' : np.array([1.5, -0.2, 0]),
             'WP3' : np.array([0.2, -0.6, 0]),
             'WP4' : np.array([-1, -0.6, 0]),
             'WP5' : np.array([-1.5, -1, 0]),
             'WP6' : np.array([-1.5, -2., 0]),
             'WP7' : np.array([-1.5, -3., 0]),
             'WP8' : np.array([-0.1, -3.2, 0]),
             'WP9' : np.array([0.2, -3.1, 0]),
             'WP10' : np.array([1.1, -3.1, 0]),
             'WP11' : np.array([1.8, -3.2, 0]),
             'WP12' : np.array([2.5, -3.1, 0]),
             'WP13' : np.array([3., -2.5, 0]),
             'WP14' : np.array([3, -1.5, 0]),
             'WP15' : np.array([3., -0.6, 0])}

def nearest_wp(shelf_pose):
    """
    Calculates the euclicidan distance between the given shelf pose and all waypoints
    contained the in the waypoints dictionary and returns the position of the waypoint
    that has the shortest distance to the shelf.
    :param shelf_pose: The position of the shelf for which a waypoint should be found
    :return: The waypoint with the shortest euclidian distance
    """
    wps = list(waypoints.values())
    shelf_pose = np.array(shelf_pose)
    distances = [np.linalg.norm(shelf_pose - w) for w in wps]
    return wps[np.argmin(distances)]


def navigation(shelf_pose, init_pose):
    """
    Currently Not Used.
    This method calculates a path, consisting of waypoints, which leads from the
    current robot position to a shelf position.
    :param shelf_pose: The shelf position to which a robot should navigate
    :param init_pose: The current position of the robot.
    :return: A list of positions, corresponding to waypoints, which lead from the
    robot position to the shelf position.
    """
    shelf_pose[2] = 0
    shelf_pose = np.array(shelf_pose)
    init_pose = np.array(init_pose)
    init_wp = "WP1"
    # Find nearest Waypoint
    for key, value in waypoints.items():
        dist = np.linalg.norm(init_pose-value)
        if dist < np.linalg.norm(init_pose-waypoints[init_wp]):
            init_wp = key
    curr_wp = init_wp
    route = [init_wp]
    while True:
        l_neighbour, r_neighbour = get_neighbour_wps(curr_wp)
        if np.linalg.norm(waypoints[curr_wp]-shelf_pose) +0.1 < np.linalg.norm(waypoints[l_neighbour]-shelf_pose) and \
            np.linalg.norm(waypoints[curr_wp]-shelf_pose) + 0.1 < np.linalg.norm(waypoints[r_neighbour]-shelf_pose):
            break

        if np.linalg.norm(waypoints[l_neighbour]-shelf_pose) < np.linalg.norm(waypoints[r_neighbour]-shelf_pose):
            curr_wp = l_neighbour
            route.append(l_neighbour)
        else:
            curr_wp = r_neighbour
            route.append(r_neighbour)
    route = [waypoints[x] for x in route]
    return route

def get_neighbour_wps(waypoint):
    """
    Currently not Used.
    Returns the name for waypoints which are located left and right the the given
    waypoint.
    :param waypoint: The waypoint name for which the neighbours should be found.
    :return: the names of naighbouring waypoints
    """
    num = int(waypoint.replace("W", "").replace("P", ""))
    left_neighbour = num % 15 + 1
    right_neighbour = (num - 1) % 15 if (num - 1) % 15 != 0 else 15
    return "WP" + str(left_neighbour), "WP" + str(right_neighbour)
