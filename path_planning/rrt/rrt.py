import random
import sys
from pathlib import Path


import yaml
sys.path.append("..")
from helpers.graph_plot import Plotting
from helpers.helper import Node, Helper_Functions, Tree
import time


class RRT:  
    """ Class to compute path between two points using RRT"""

    def __init__(
        self,
        start: list,
        goal: list,
        area_x: list,
        area_y: list,
        obstacle_list: list,
        robot_radius: float,
        padding: float,
        goal_check_distance=1.0,
        path_resolution=0.05,
        max_iteration=100000,
        show_live_plot=False
    ):
        """
        start: [x,y] of starting point (m,m)
        goal: [x,y] of goal (m,m)
        area_x: Min and max x limits of the map (m,m)
        area_y: Min and max y limits of the map (m,m)
        obstacle_list: [ox, oy, os] center point and size of the obstacle (m,m,m)
        robot_radius: radius of the robot (m)
        padding: Extra padding around the robot, for safe planning along obstacles (m)
        goal_check_distance: DIstance below which robot is assumed to have reached the goal (m^2)
        path_resolution: Resolution of the path to check for obstacles, should be less than obstacle width. (m)
        max_iterations: Maximum itreations of RRT Node that needs to be run
        show_live_plot: Plot rrt tree as it is expanding 
        """
        self.index = 0
        self.start = Node(start[0], start[1], self.index)
        self.goal = Node(goal[0], goal[1])
        self.area_x = area_x
        self.area_y = area_y
        self.obstacle_list = obstacle_list
        self.goal_check_distance = goal_check_distance
        self.path_resolution = path_resolution
        self.max_iteration = max_iteration
        self.enable_realtime_plot = show_live_plot
        self.plotting = Plotting()
        self.plotting.plot_obstacles(self.obstacle_list)
        self.robot_radius = robot_radius
        self.padding = padding
        self.path_found = False

    def find_path(self):
        """ Comutes Path between the two points"""
        iteration = 0
        tree = Tree()
        tree.add_new_node(self.start)

        while iteration < self.max_iteration:
            plot_list_x = []
            plot_list_y = []
            random_node_point = [
                round(random.uniform(self.area_x[0], self.area_x[1]), 2),
                round(random.uniform(self.area_y[0], self.area_y[1]), 2),
            ]
            if Helper_Functions.check_collision(
                random_node_point, self.obstacle_list, self.robot_radius, self.padding
            ):
                continue
            random_node = Node(random_node_point[0], random_node_point[1])
            nearest_node_index = Helper_Functions.nearest_node_in_tree(
                tree.tree_of_points, random_node_point
            )
            (
                path_to_nearest_node,
                goal_reached,
                path_valid,
            ) = Helper_Functions.steer_to_node(
                tree.tree_of_points[nearest_node_index],
                random_node_point,
                self.goal.point,
                self.path_resolution,
                self.obstacle_list,
                self.goal_check_distance,
                self.robot_radius,
                self.padding
            )
            if path_valid:
                self.index += 1
                if goal_reached:
                    self.goal.parent_index = nearest_node_index
                    self.goal.index = self.index
                    self.goal.path_to_parent = path_to_nearest_node
                    tree.add_new_node(path_to_nearest_node[-1])
                    plot_list_x.append(
                        tree.tree_of_points[nearest_node_index][0])
                    plot_list_y.append(
                        tree.tree_of_points[nearest_node_index][1])
                    plot_list_x.append(tree.tree_of_points[self.index][0])
                    plot_list_y.append(tree.tree_of_points[self.index][1])
                    if self.enable_realtime_plot:
                        self.plotting.plot_line(plot_list_x, plot_list_y)
                    goal_index = self.index
                    self.path_found = True
                    break
                else:
                    random_node.parent_index = nearest_node_index
                    random_node.index = self.index
                    random_node.path_to_parent = path_to_nearest_node
                    tree.add_new_node(random_node)
                    plot_list_x.append(
                        tree.tree_of_points[nearest_node_index][0])
                    plot_list_y.append(
                        tree.tree_of_points[nearest_node_index][1])
                    plot_list_x.append(tree.tree_of_points[self.index][0])
                    plot_list_y.append(tree.tree_of_points[self.index][1])
                    if self.enable_realtime_plot:
                        self.plotting.plot_line(plot_list_x, plot_list_y)
                    (
                        path_to_nearest_node,
                        goal_reached,
                        path_valid,
                    ) = Helper_Functions.steer_to_node(
                        random_node_point,
                        self.goal.point,
                        self.goal.point,
                        self.path_resolution,
                        self.obstacle_list,
                        self.goal_check_distance,
                        self.robot_radius,
                        self.padding
                    )
                    if goal_reached and path_valid:
                        self.goal.parent_index = self.index
                        self.index += 1
                        self.goal.index = self.index
                        self.goal.path_to_parent = path_to_nearest_node
                        tree.add_new_node(self.goal)
                        plot_list_x.append(tree.tree_of_points[self.index][0])
                        plot_list_y.append(tree.tree_of_points[self.index][1])
                        plot_list_x.append(tree.tree_of_points[self.index][0])
                        plot_list_y.append(tree.tree_of_points[self.index][1])
                        if self.enable_realtime_plot:
                            self.plotting.plot_line(plot_list_x, plot_list_y)
                        goal_index = self.index
                        self.path_found = True
                        break

            iteration += 1
        if(self.path_found):
            print("Path Found")
            self.show_final_path(tree, goal_index)
        else:
            print("Path Not Found")

    def show_final_path(self, tree, goal_index):
        """ Display the final path that is calculated"""
        index_to_print = goal_index
        plot_list_x = []
        plot_list_y = []

        plot_list_x.append(tree.tree_of_points[index_to_print][0])
        plot_list_y.append(tree.tree_of_points[index_to_print][1])

        while not (index_to_print == 0):
            index_to_print = tree.tree_of_nodes[index_to_print].parent_index
            plot_list_x.append(tree.tree_of_points[index_to_print][0])
            plot_list_y.append(tree.tree_of_points[index_to_print][1])
        self.plotting.plot_line(plot_list_x, plot_list_y, "black")
        self.plotting.stop_plot()


def main():
    # Get resources from yaml file
    with open("../resources.yaml") as stream:
        try:
            resources_loaded = yaml.safe_load(stream)
            print(resources_loaded)
        except yaml.YAMLError as exc:
            print("File Not read")
    obstacle_zone = resources_loaded["Map"]["obstacle_list"]
    map_x_limits = resources_loaded["Map"]["limits_x"]
    map_y_limits = resources_loaded["Map"]["limits_y"]
    start = resources_loaded["PathPlanning"]["start"]
    goal = resources_loaded["PathPlanning"]["goal"]
    robot_radius = resources_loaded["Robot"]["size"]
    robot_padding = resources_loaded["Robot"]["padding"]
    goal_check_distance = resources_loaded["PathPlanning"]["goal_check_distance"]
    show_live_plot = resources_loaded["PathPlanning"]["show_live_plot"]
    max_iterations = resources_loaded["PathPlanning"]["RRT"]["max_iteration"]
    path_resolution = resources_loaded["PathPlanning"]["RRT"]["path_resolution"]
    
    # Call the RRT Function
    find_path = RRT(start, goal, map_x_limits,
                    map_y_limits, obstacle_zone, robot_radius, robot_padding, goal_check_distance, path_resolution, max_iterations, show_live_plot)
    find_path.find_path()


if __name__ == "__main__":
    main()
