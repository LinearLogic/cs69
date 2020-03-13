#!/usr/bin/env python
import math

class PathPlanner: # a path planner currently offering A* functionality; more search strategies will be implemented in the future

    def __init__(self, occupancy_grid, map_width, map_height, map_resolution, map_origin, occupancy_threshold=65):
        # init map info
        self.occupancy_grid = occupancy_grid
        self.map_width = map_width
        self.map_height = map_height
        self.map_resolution = map_resolution
        self.map_origin = map_origin
        self.occupancy_threshold = occupancy_threshold

        self.start = (0,0)
        self.goal = (0,0)

        # init path list
        self.best_path = []


    def set_start(self, start):
        self.start = self.world_to_grid(start)


    def set_goal(self, goal):
        self.goal = self.world_to_grid(goal)


    def is_occupied(self, point):
        return self.occupancy_grid[int(point[0] + point[1] * self.map_width)] >= self.occupancy_threshold


    # converts world x/y coords to grid coords
    def world_to_grid(self, world_coords):
        world_x = world_coords[0]
        world_y = world_coords[1]
        origin_x = self.map_origin.position.x
        origin_y = self.map_origin.position.y

        grid_x = max(0, math.floor((world_x - origin_x) / self.map_resolution)) # ensure values always greater than zero
        grid_y = max(0, math.floor((world_y - origin_y) / self.map_resolution))

        return (grid_x, grid_y)


    # converts grid x/y coords to world coords
    def grid_to_world(self, grid_coords):
        grid_x = grid_coords[0]
        grid_y = grid_coords[1]
        origin_x = self.map_origin.position.x
        origin_y = self.map_origin.position.y

        world_x = (grid_x * self.map_resolution) + origin_x
        world_y = (grid_y * self.map_resolution) + origin_y

        return (world_x, world_y)


    # function for a* search algorithm
    def a_star(self, max_iterations=500):
        # init start and end nodes with f, g, h as 0
        start_node = Node(None, self.start)
        start_node.g = 0
        start_node.h = 0
        start_node.f = 0
        goal_node = Node(None, self.goal)
        goal_node.g = 0
        goal_node.h = 0
        goal_node.f = 0

        iterations = 0 # limit iterations

        # create open and closed lists
        open_list = []
        closed_list = []

        # add the start node to open list
        open_list.append(start_node)

        # loop until open list is empty
        while len(open_list) > 0:
            if iterations >= max_iterations:
                return []
            iterations += 1
            # get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # pop current off open list and add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # found the goal and create the path
            if current_node == goal_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # generate children
            children = []
            adjacency_array = [(0, -1), (0, 1), (-1, 0), (1, 0)]
            for new_position in adjacency_array: # adjacent squares

                # get node position using 4 adjacent squares
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # ensure within range of map
                if node_position[0] > self.map_width or node_position[0] < 0 or node_position[1] > self.map_height or node_position[1] < 0:
                    continue

                # ensure there are no obstacles
                if self.is_occupied(node_position):
                    continue

                # create new node
                new_node = Node(current_node, node_position)

                # append new node to children list
                children.append(new_node)

            # loop through children
            for child in children:

                # child is on the closed list then skip calculations
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # create the f, g, and h values for the child
                child.g = current_node.g + 1
                child.h = ((child.position[0] - goal_node.position[0]) ** 2) + ((child.position[1] - goal_node.position[1]) ** 2)
                child.f = child.g + child.h

                # child is already in the open list then do not add again
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # add the child to the open list
                open_list.append(child)
        # else
        return []


# Node class
class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        # dist between current node and start
        self.g = 0
        # estimated distance between current node and end node
        self.h = 0
        # total cost
        self.f = 0

    # compares position when trying to compare node
    def __eq__(self, other):
        return self.position == other.position
