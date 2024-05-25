import random
import yaml

from graph_plot import Plotting
from helper import Node, RRT_Helper_Functions, Tree


class RRT:
    def __init__(
        self,
        start,
        goal,
        area_x,
        area_y,
        obstacle_list,
        robot_radius,
        padding,
        goal_check_distance=1.0,
        path_resolution=0.05,
        max_iteration=100000,
    ):
        """
        start: [x,y] of starting point
        goal: [x,y] of goal
        area_x: Min and max x limits of the map
        area_y: Min and max y limits of the map
        goal_check_distance: DIstance below which robot is assumed to have reached the goal
        path_resolution = Resolution of the path to check for obstacles (Should be less than obstacle width),
        max_iterations = Maximum itreations of RRT Node that needs to be run
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
        self.path_found = False
        self.enable_realtime_plot = True
        self.plotting = Plotting()
        self.plotting.plot_obstacles(self.obstacle_list)
        self.robot_radius = robot_radius
        self.padding = padding

    def find_path(self):
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
            if RRT_Helper_Functions.check_collision(
                random_node_point, self.obstacle_list, self.robot_radius, self.padding
            ):
                continue
            random_node = Node(random_node_point[0], random_node_point[1])
            nearest_node_index = RRT_Helper_Functions.nearest_node_in_tree(
                tree.tree_of_points, random_node_point
            )
            (
                path_to_nearest_node,
                goal_reached,
                path_valid,
            ) = RRT_Helper_Functions.steer_to_node(
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
                    plot_list_x.append(tree.tree_of_points[nearest_node_index][0])
                    plot_list_y.append(tree.tree_of_points[nearest_node_index][1])
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
                    plot_list_x.append(tree.tree_of_points[nearest_node_index][0])
                    plot_list_y.append(tree.tree_of_points[nearest_node_index][1])
                    plot_list_x.append(tree.tree_of_points[self.index][0])
                    plot_list_y.append(tree.tree_of_points[self.index][1])
                    if self.enable_realtime_plot:
                        self.plotting.plot_line(plot_list_x, plot_list_y)
                    (
                        path_to_nearest_node,
                        goal_reached,
                        path_valid,
                    ) = RRT_Helper_Functions.steer_to_node(
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
    with open("resources.yaml") as stream:
        try:
            resources_loaded = yaml.safe_load(stream)
            print(resources_loaded)
        except yaml.YAMLError as exc:
            print(exc)
    obstacle_zone = resources_loaded["map"]["obstacle_list"]

    find_path = RRT([0.0, 40.0], [30.0, 30.0], [0.0, 40.0],
                    [0.0, 40.0], obstacle_zone, 1.8, 0.2)
    find_path.find_path()


if __name__ == "__main__":
    main()
