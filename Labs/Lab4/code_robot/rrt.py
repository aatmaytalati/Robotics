# Aatmay S. Talati, Kristian Kabbabe, Min Ho Lee

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
import asyncio
from cmap import *
from gui import *
from utils import *
import numpy as np
from transitions import Machine, State
from time import sleep

cubeID = 1
MAX_NODES = 20000
states = [  State(name = 'FindTarget', on_enter=[], on_exit=[]),
            State(name = 'FindPath', on_enter=[], on_exit=[]),
            State(name = 'MoveToGoal', on_enter=[], on_exit=[]),
            State(name = 'DetectObstacle', on_enter=[], on_exit=[]),
            State(name = 'Stop', on_enter=[], on_exit=[]),]

transitions = [ {'trigger':'move','source':'FindPath','dest':'MoveToGoal'},
                {'trigger':'foundObstacle','source':'MoveToGoal','dest':'DetectObstacle'},
                {'trigger':'reachedGoal','source':'MoveToGoal','dest':'Stop'},
                {'trigger':'findPath','source':['DetectObstacle', 'FindTarget'],'dest':'FindPath'},]

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def findTarget(robot: cozmo.robot.Robot, cmap, stack, cube):
    # ADD CODE HERE
    # Cozmo should move towards the middle and scout the area until the cube is found!
    print("Find Target State")
    print(robot.pose_angle.degrees)
    robot.turn_in_place(degrees(45)).wait_for_completed()
    robot.drive_straight(distance_mm(250), speed_mmps(50)).wait_for_completed()
    lookAround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cube = None
    try:
        cube = robot.world.wait_for_observed_light_cube()
        lookAround.stop()
        
    except asyncio.TimeoutError:
        print("did not find cube")
    
    # cube.object_id how you detect which CUBE you are using
    if cube is not None:
        print(cube)
        if cube.object_id == cubeID:
            cmap.set_start(Node((robot.pose.position.x, robot.pose.position.y)))
            cmap.add_goal(Node((cube.pose.position.x,cube.pose.position.y)))
            robot.findPath() # trigger to go to FindPath state
    return robot, stack, cube

"""
findPath updates the RRT and returns a stack with the path!
KKnotes: in theory it should work! only one way to find out!
Function is done. In theory
"""
def findPath(robot: cozmo.robot.Robot, cmap, stack, cube):
    print("Find Path State")
    cmap.reset()
    RRT(cmap, cmap.get_start())
    goals = cmap.get_goals()
    print(goals[0])
    stack = []
    for goal in goals:
        cur = goal
        while cur is not None:
            stack.append(cur.coord)
            cur = cur.parent
    stack.pop()
    robot.move() # trigger to go to MoveToGoal state
    return robot, stack, cube # returns cozmo

def moveToGoal(robot: cozmo.robot.Robot, cmap, stack, cube):
    print("Move To Goal State")
    cube = None
    goals = cmap.get_goals();
    goal = goals[0]
    if stack:
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout = 4)
            print(cube)
        except asyncio.TimeoutError:
            print("no obstacles found")
            pass
        if cube is not None and cube.object_id != cubeID:
            print("Tadah")
            robot.foundObstacle()
        else:
            # Keep moving
            curr = stack.pop()
            ro = (robot.pose.position.x, robot.pose.position.y)
            if ((goal[0] - 90 < ro[0] < goal[0] + 90) and (goal[1] - 90 < ro[1] < goal[1] + 90)):
                print("Yay");
                robot.reachedGoal()
            dist = get_dist(Node(ro), Node(curr)) #get distance
            angle = np.arctan2(curr[1]-ro[1],curr[0]-ro[0])*180/np.pi - robot.pose_angle.degrees
            print(angle)
            robot.turn_in_place(degrees(angle)).wait_for_completed()
            robot.drive_straight(distance_mm(dist), speed_mmps(50)).wait_for_completed()
    else:
        print("Yay")
        robot.reachedGoal()

    return robot, stack, cube # returns cozmo

def detectObstacle(robot: cozmo.robot.Robot, cmap, stack, cube):
    print("Detect Obstacle State")
    # cmap.set_start(Node((robot.pose.position.x, robot.pose.position.y)))
	# nodes = [Node((cube.pose.position.x-5, cube.pose.position.y-5)),Node((cube.pose.position.x+5, cube.pose.position.y+5)),Node	((cube.pose.position.x-5, cube.pose.position.y+5)),Node((cube.pose.position.x+5, cube.pose.position.y-5))]
    # cmap.add_obstacle(nodes)

    robot.findPath() #trigger to go to FindPath state
    return robot, stack, cube # returns cozmo

def stop(robot: cozmo.robot.Robot, cmap, stack, cube):
    print("Stop State YAYYY")
    return robot, stack, cube # returns cozmo


def step_from_to(node0, node1, limit=75):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    ############################################################################
    dist_robo = get_dist(node0, node1)
    if (dist_robo < limit):
        return node1
    else:
        dy = node1.y-node0.y
        dx = node1.x-node0.x
        angle = np.arctan2(dy,dx)
        return Node((limit*np.cos(angle), limit*np.sin(angle)))
        

def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    ############################################################################
    xCoor = np.random.rand()*cmap.width
    yCoor = np.random.rand()*cmap.height
    rand_node = Node((xCoor,yCoor))  # xCoor and yCor needs to make a cordinate, and plus parent_node.

    while not cmap.is_inbound(rand_node) or cmap.is_inside_obstacles(rand_node):
        xCoor = np.random.rand()*cmap.width
        yCoor = np.random.rand()*cmap.height
        rand_node = Node((xCoor, yCoor))
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        ########################################################################
        rand_node = cmap.get_random_valid_node() #get random node
        nodes = cmap.get_nodes() # get lists of all nodes
        closest = float('inf'); #sets initial value to infinity

        for node in nodes: #iterate among all nodes
            dist = get_dist(node, rand_node) #loop finds the closest node to the rand_node
            if  dist < closest:
                closest = dist
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)
        sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")


switch = {  "FindTarget": findTarget,
            "FindPath" : findPath,
            "MoveToGoal" : moveToGoal,
            "DetectObstacle" : detectObstacle,
            "Stop" : stop}

def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent, stack
    stack = []
    cube = None
    cmap.clear_goals()
    initState = 'FindTarget'
    machine = Machine(model=robot, states=states, transitions=transitions, initial=initState, ignore_invalid_triggers=True)
    robot.set_head_angle(degrees(-10)).wait_for_completed()
    robot.set_lift_height(1.0).wait_for_completed()
    print((robot.pose.position.x,robot.pose.position.y))
    while(1):
        robot, stack, cube = switch[robot.state](robot, cmap, stack, cube)
    
    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    ########################################################################
    

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=True, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent, stack
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator) # Program depends upon provided upon cmap.py, gui.py and utill.py.
    robot_thread = RRTThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
