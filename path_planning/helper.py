import math

import numpy as np
from scipy import spatial


class Node:
    def __init__(self, x, y, index=None, parent_index=None):
        self.index = index
        self.point = [x, y]
        self.parent_index = parent_index
        self.path_to_parent = []


class Tree:
    def __init__(self) -> None:
        self.tree_of_nodes = []
        self.tree_of_points = []

    def add_new_node(self, node):
        self.tree_of_nodes.append(node)
        self.tree_of_points.append(node.point)

class RRT_Helper_Functions:
    def nearest_node_in_tree(tree_points, node_point):
        return spatial.KDTree(tree_points).query(node_point)[1]

    def distance_angle_between_nodes(from_node_point, to_node_point):
        dx = to_node_point[0] - from_node_point[0]
        dy = to_node_point[1] - from_node_point[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        return distance, angle

    def steer_to_node(
        from_node_point,
        to_node_point,
        goal_point,
        path_resolution,
        obstacle_list,
        goal_threhsold,
        robot_radius,
        padding,
    ):
        distance, angle = RRT_Helper_Functions.distance_angle_between_nodes(
            from_node_point, to_node_point
        )
        points_in_path = []
        itreations = 0
        total_iterations = distance / path_resolution
        goal_reached = False
        path_valid = True
        while itreations < total_iterations:
            point = [0.0, 0.0]
            point[0] = from_node_point[0] + itreations * path_resolution * math.cos(
                angle
            )
            point[1] = from_node_point[1] + itreations * path_resolution * math.sin(
                angle
            )
            if RRT_Helper_Functions.check_collision(
                point, obstacle_list, robot_radius, padding
            ):
                path_valid = False
                break

            points_in_path.append(point)
            if RRT_Helper_Functions.is_goal_reached(point, goal_point, goal_threhsold):
                goal_reached = True
                break
            itreations += 1
        return points_in_path, goal_reached, path_valid

    def is_goal_reached(node_point, goal_point, goal_threhsold):
        if np.linalg.norm(np.array(goal_point) - np.array(node_point)) < goal_threhsold:
            return True
        else:
            return False

    def check_collision(point, obstacle_list, robot_radius, padding):
        dx_list = [abs(ox - point[0]) for [ox, _, _] in obstacle_list]
        dy_list = [abs(oy - point[1]) for [_, oy, _] in obstacle_list]
        R_list = [os/2 + robot_radius +
                  padding for [_, _, os] in obstacle_list]
        for i in range(len(R_list)):
            if dx_list[i] > R_list[i] or dy_list[i] > R_list[i]:
                continue
            else:
                return True
        return False
