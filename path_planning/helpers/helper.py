import math

import numpy as np
from scipy import spatial


class Node:
    """ Definition of a node explored during path planning"""

    def __init__(self, x: float, y: float, index=None, parent_index=None):
        """
        x: x position of node (m)
        y: y position of node (m)
        index: Location of node in a tree
        parent_index: Points to the index of the parent node
        """
        self.index = index
        self.point = [x, y]
        self.parent_index = parent_index
        self.path_to_parent = []


class Tree:
    """ Definition of a tree -> list of nodes and points collected during path planning"""

    def __init__(self) -> None:
        self.tree_of_nodes = []
        self.tree_of_points = []

    def add_new_node(self, node) -> None:
        # Add a new node to a tree
        self.tree_of_nodes.append(node)
        self.tree_of_points.append(node.point)


class Helper_Functions:
    def nearest_node_in_tree(tree: list, node: list)-> float:
        """
        Finds the nearest point in the tree, from a random point
        tree: [[x1,y1],[x2,y2]....] current points in the tree
        node: [x,y] random point (m,m)
        """

        return spatial.KDTree(tree).query(node)[1]

    def distance_angle_between_nodes(from_node: list, to_node: list)-> [float,float]:
        """
        Measure Distance and angle between two points
        from_node: [x,y] of start point (m,m)
        to_node:  [x,y] of to point (m,m)
        """

        dx = to_node[0] - from_node[0]
        dy = to_node[1] - from_node[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        return distance, angle

    def steer_to_node(
        from_node: list,
        to_node: list,
        goal: list,
        path_resolution: float,
        obstacle_list: float,
        goal_threhsold: float,
        robot_radius: float,
        padding: float,
    ):
        """ 
        Finds the path and waypoints between two points, and checks for collisions in the points.
        Also checks if the waypoints found, are close to the goal.
        from_node: [x,y] of start point (m,m)
        to_node:  [x,y] of to point (m,m)
        goal: [x,y] final goal of planning (m,m)
        path_resolution: Distance at which to find the waypoints (m)
        obstacle_list: [ox,os,oy] center and shape of obstacles present inside the map (m,m,m)
        goal_threshold: Points within this distance to the goal will be considered goal (m^2)
        robot_radius: Size of the robot (m)
        padding: Extra padding around the robot, for safe planning along obstacles (m)
        """

        distance, angle = Helper_Functions.distance_angle_between_nodes(
            from_node, to_node
        )
        waypoints = []
        itreations = 0
        total_iterations = distance / path_resolution
        goal_reached = False
        path_valid = True
        while itreations < total_iterations:
            node = [float(0), float(0)]
            node[0] = from_node[0] + itreations * path_resolution * math.cos(
                angle
            )
            node[1] = from_node[1] + itreations * path_resolution * math.sin(
                angle
            )
            if Helper_Functions.check_collision(
                node, obstacle_list, robot_radius, padding
            ):
                path_valid = False
                break

            waypoints.append(node)
            if Helper_Functions.is_goal_reached(node, goal, goal_threhsold):
                goal_reached = True
                break
            itreations += 1
        return waypoints, goal_reached, path_valid

    def is_goal_reached(node: list, goal: list, goal_threhsold: float) -> bool:
        """
        Checks if the node is within a certain threshold to the goal.
        node: [x,y] node to be checked (m,m)
        goal: [x,y] Goal (m,m)
        goal_threshold: Max distance to goal (m^2)
        """
        
        if np.linalg.norm(np.array(goal) - np.array(node)) < goal_threhsold:
            return True
        else:
            return False

    def check_collision(node: list, obstacle_list: list, robot_radius: float, padding: float) -> bool:
        """
        Checks if a node is in the collision region.
        node: [x,y] node to be checked (m,m)
        obstacle_list: [ox,os,oy] center and shape of obstacles present inside the map (m,m,m)    
        robot_radius: Size of the robot (m)
        padding: Extra padding around the robot, for safe planning along obstacles (m)
        """

        dx_list = [abs(ox - node[0]) for [ox, _, _] in obstacle_list]
        dy_list = [abs(oy - node[1]) for [_, oy, _] in obstacle_list]
        R_list = [os/2 + robot_radius +
                  padding for [_, _, os] in obstacle_list]
        for i in range(len(R_list)):
            if dx_list[i] > R_list[i] or dy_list[i] > R_list[i]:
                continue
            else:
                return True
        return False
