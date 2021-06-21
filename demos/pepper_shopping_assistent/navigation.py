# import pepper_process_modules
# import available_process_modules
# import motion_designators
import numpy as np

# {Shelf8 position: x:-0,9499998,y:0,45,z:-4,185,rotation: x:0,y:-0,7071068,z:0,w:0,7071068}


# waypoints = {'WP1' : np.array([[2.8, -0.97, 0], [0, 0, -0.70, 0.70]]),
#              'WP2' : np.array([[1.58, -0.84, 0], [0, 0, -0.70, 0.70]]),
#              'WP3' : np.array([[0.5, -0.84, 0], [0, 0, -0.70, 0.70]]),
#              'WP4' : np.array([[-0.85, -0.91, 0], [0, 0, -0.70, 0.70]]),
#              'WP5' : np.array([[-1.2, -1.35, 0], [0, 0, -0.70, 0.70]]),
#              'WP6' : np.array([[-1.2, -2.19, 0], [0, 0, -0.70, 0.70]]),
#              'WP7' : np.array([[-1.2, -3.06, 0], [0, 0, -0.70, 0.70]]),
#              'WP8' : np.array([[-0.44, -3.4, 0], [0, 0, -0.70, 0.70]]),
#              'WP9' : np.array([[0.54, -3.37, 0], [0, 0, -0.70, 0.70]]),
#              'WP10' : np.array([[1.58, -3.43, 0], [0, 0, -0.70, 0.70]]),
#              'WP11' : np.array([[2.396, -3.44, 0], [0, 0, -0.70, 0.70]]),
#              'WP12' : np.array([[3.31, -3.36, 0], [0, 0, -0.70, 0.70]]),
#              'WP13' : np.array([[3.643, -2.757, 0], [0, 0, -0.70, 0.70]]),
#              'WP14' : np.array([[3.644, -1.91, 0], [0, 0, -0.70, 0.70]]),
#              'WP15' : np.array([[3.405, -1.118, 0], [0, 0, -0.70, 0.70]])}

waypoints = {'WP1' : np.array([[1.8, -0.2, 0], [0, 0, 1, 0]]),
             'WP2' : np.array([[1.5, -0.3, 0], [0, 0, 1, 0]]),
             'WP3' : np.array([[0.2, -0.5, 0], [0, 0, -0.70, 0.70]]),
             'WP4' : np.array([[-1, -0.5, 0], [0, 0, -0.70, 0.70]]),
             'WP5' : np.array([[-1.3, -1, 0], [0, 0, -0.70, 0.70]]),
             'WP6' : np.array([[-1.5, -2., 0], [0, 0, -0.70, 0.70]]),
             'WP7' : np.array([[-1.5, -3.1, 0], [0, 0, -0.70, 0.70]]),
             'WP8' : np.array([[-0.1, -3.2, 0], [0, 0, -0.70, 0.70]]),
             'WP9' : np.array([[0.2, -3.1, 0], [0, 0, -0.70, 0.70]]),
             'WP10' : np.array([[1.1, -3.1, 0], [0, 0, -0.70, 0.70]]),
             'WP11' : np.array([[1.8, -3.2, 0], [0, 0, -0.70, 0.70]]),
             'WP12' : np.array([[2.5, -3.1, 0], [0, 0, -0.70, 0.70]]),
             'WP13' : np.array([[3., -2.5, 0], [0, 0, -0.70, 0.70]]),
             'WP14' : np.array([[3, -1.5, 0], [0, 0, -0.70, 0.70]]),
             'WP15' : np.array([[3., -0.8, 0], [0, 0, -0.70, 0.70]])}

def navigation(shelf_pose, init_pose):
    shelf_pose = np.array(shelf_pose)
    init_pose = np.array(init_pose)
    init_wp = "WP1"
    # Find nearest Waypoint
    for key, value in waypoints.items():
        dist = np.linalg.norm(init_pose-value[0])
        if dist < np.linalg.norm(init_pose-waypoints[init_wp][0]):
            init_wp = key
    curr_wp = init_wp
    route = [init_wp]
    while True:
        l_neighbour, r_neighbour = get_neighbour_wps(curr_wp)
        if np.linalg.norm(waypoints[curr_wp][0]-shelf_pose) < np.linalg.norm(waypoints[l_neighbour][0]-shelf_pose) and \
            np.linalg.norm(waypoints[curr_wp][0]-shelf_pose) < np.linalg.norm(waypoints[r_neighbour][0]-shelf_pose):
            break

        if np.linalg.norm(waypoints[l_neighbour][0]-shelf_pose) < np.linalg.norm(waypoints[r_neighbour][0]-shelf_pose):
            curr_wp = l_neighbour
            route.append(l_neighbour)
        else:
            curr_wp = r_neighbour
            route.append(r_neighbour)
    print(route)
    route = [waypoints[x] for x in route]
    return route

def get_neighbour_wps(waypoint):
    num = int(waypoint.replace("W", "").replace("P", ""))
    left_neighbour = (num + 1) % 15
    right_neighbour = (num - 1) % 15 if (num - 1) % 15 != 0 else 15
    return "WP" + str(left_neighbour), "WP" + str(right_neighbour)

navigation([-0.94, -4.185, 0.45], [2, 0, 0])
