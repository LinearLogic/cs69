#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import numpy as np
import g2o

DEBUG=True

class Graph_SLAM_ROS:
    """A ROS node that performs graph-SLAM based on /pose and /scan messages, using a g2opy backend

    Args:
        measured_start_distance (float): the distance the robot is measured to be starting from the wall, in meters
        measured_end_distance (float): the robot's final distance from the wall (meters) as measured after the actuation routine is complete
        front_range_index (int): the index in the array of laser scanner ranges corresponding to the measurement directly ahead of the robot (along the x-axis)
        rate (int): the robot tick rate, in Hz. Defaults to 10.
    """

    def __init__(self, measured_start_distance, measured_end_distance, front_range_index=0, rate=10):
        self.measured_start_distance = measured_start_distance
        self.measured_end_distance = measured_end_distance
        self.front_range_index = front_range_index
        self.rate = rospy.Rate(rate)

        # subscribe to relevant topics
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1) # scan used for landmark vertex + edges
        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback, queue_size=1) # pose used for pose vertices and edges

        # init optimizer
        self.optimizer = PoseGraphOptimization()

        # init transient variables used to calculate pose-pose edge measurements
        self.last_pose_x = -1
        self.last_pose_y = -1
        self.last_pose_theta = -1

    def scan_callback(self, msg): # parses /scan messages into a landmark vertex and edges between it and pose vertices
        measurement = msg.ranges[self.front_range_index] # for this assignment we only consider the range directly ahead of the robot
        if measurement != np.inf and self.optimizer.get_last_pose_vertex_id() != -1: # only parse /scan after a /pose message has been parsed, since landmark is relative to pose
            xy = [measurement, 0] # x is the range directly ahead of the robot, thus y is zero
            if self.optimizer.get_landmark_vertex_id() == -1: # if landmark not yet set
                self.optimizer.add_landmark_vertex(xy) # set it now
            self.optimizer.add_landmark_edge([self.optimizer.get_last_pose_vertex_id(), self.optimizer.get_landmark_vertex_id()], xy) # add pose-landmark edge
        return

    def pose_callback(self, msg): # parses /pose messages into pose vertices and edges between them
        # store x,y,theta of new pose; use them to construct g2o SE2 for the pose measurement
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = msg.pose.orientation.z
        se2 = g2o.SE2(x, y, theta) # pose measurement
        # se2 = g2o.SE2(0, 0, 0) # pose measurement with zeros; uncomment to test optimization functionality

        prev_vertex_id = self.optimizer.get_last_pose_vertex_id() # store most recent prior pose vertex id, if any, before adding new vertex (as this will override last vertex id)
        v_se2, new_vertex_id = self.optimizer.add_pose_vertex(se2) # register new pose vertex with optimizer

        if prev_vertex_id != -1: # this is not the first pose vertex, so a pose-pose edge can be calculated and added
            # calculate delta x,y,theta between prev and new poses; use them to construct g2o SE2 for the edge measurement
            delta_x = x - self.last_pose_x
            delta_y = y - self.last_pose_y
            delta_theta = msg.pose.orientation.z - self.last_pose_theta
            delta_se2 = g2o.SE2(delta_x, delta_y, delta_theta) # pose-pose edge measurement
            self.optimizer.add_pose_edge([prev_vertex_id, new_vertex_id], delta_se2) # register the new edge with the optimizer

        # update the x,y,theta of the most recent pose vertex, to be used to calculate the next pose-pose edge measurement
        self.last_pose_x = x
        self.last_pose_y = y
        self.last_pose_theta = theta
        return

    def receive_input(self): # runs the graph optimization and plots error upon receiving user input
        input = raw_input() # await input (content of input doesn't matter; user just needs to hit 'Enter' to trigger the optimization)
        if self.optimizer.get_last_pose_vertex_id() != -1 and self.optimizer.get_landmark_vertex_id() != -1: # only attempt to optimize if graph is populated
            self.optimizer.save('graph_raw.g2o') # save the raw, unoptimized graph and store the raw first/last pose vertices for error plotting
            raw_first_pose_vertex = self.optimizer.vertex(self.optimizer.get_first_pose_vertex_id()).estimate().to_vector()
            raw_last_pose_vertex = self.optimizer.vertex(self.optimizer.get_last_pose_vertex_id()).estimate().to_vector()

            self.optimizer.optimize() # run the optimization

            self.optimizer.save('graph_optimized.g2o') # save the optimized graph and store the optimized first/last pose vertices for error plotting
            optimized_first_pose_vertex = self.optimizer.vertex(self.optimizer.get_first_pose_vertex_id()).estimate().to_vector()
            optimized_last_pose_vertex = self.optimizer.vertex(self.optimizer.get_last_pose_vertex_id()).estimate().to_vector()

            if DEBUG: # print raw and optimized first and last pose vertex [x,y,theta]
                print 'Optimization complete\n\nRaw (saved to graph_raw.g2o)\n- start:'
                print raw_first_pose_vertex
                print '- end:'
                print raw_last_pose_vertex
                print '\nOptimized (saved to graph_optimized.g2o)\n- start:'
                print optimized_first_pose_vertex
                print '- end:'
                print optimized_last_pose_vertex

            # set bar heights to be starting/ending x displacement values (as opposed to distance from the wall, which conveys equivalent information, but less visually intuitively)
            measured_x_displacement = self.measured_start_distance - self.measured_end_distance
            measured_poses = [0, measured_x_displacement]
            odom_poses = [raw_first_pose_vertex[0], raw_last_pose_vertex[0]] # [0] because we are only concerned with the x coordinate in this case
            estimated_poses = [optimized_first_pose_vertex[0], optimized_last_pose_vertex[0]] # [0] because we are only concerned with the x coordinate in this case

            # init plot
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
            bar_width = 0.25

            # starting pose subplot
            ax1.bar([0], [measured_poses[0]], color='#7f6d5f', width=bar_width, edgecolor='white', label='Measurement')
            ax1.bar([bar_width], [odom_poses[0]], color='#557f2d', width=bar_width, edgecolor='white', label='Odometry')
            ax1.bar([2*bar_width], [estimated_poses[0]], color='#2d7f5e', width=bar_width, edgecolor='white', label='Graph-SLAM estimate')
            ax1.set_ylabel('x displacement (m)', fontweight='bold')
            ax1.set_xlabel('Start pose', fontweight='bold')

            # ending pose subplot
            ax2.bar([0], [measured_poses[1]], color='#7f6d5f', width=bar_width, edgecolor='white', label='Measurement')
            ax2.bar([bar_width], [odom_poses[1]], color='#557f2d', width=bar_width, edgecolor='white', label='Odometry')
            ax2.bar([2*bar_width], [estimated_poses[1]], color='#2d7f5e', width=bar_width, edgecolor='white', label='Graph-SLAM estimate')
            ax2.set_xlabel('End pose', fontweight='bold')
            values = [measured_poses[1], odom_poses[1], estimated_poses[1]]
            ax2.set_ylim(min(values) * 0.99, max(values) * 1.01) # scale y-axis range to fit data
            ax2.legend(loc='upper center')

            # ending pose error wrt measured ground truth subplot
            ax3.bar([0], [measured_x_displacement - odom_poses[1]], color='#557f2d', width=bar_width, edgecolor='white', label='Odometry')
            ax3.bar([bar_width], [measured_x_displacement - estimated_poses[1]], color='#2d7f5e', width=bar_width, edgecolor='white', label='Graph-SLAM estimate')
            ax3.set_xlabel('End pose error', fontweight='bold')

            plt.show() # draw plot
        return

    def spin(self):
        while not rospy.is_shutdown():
            self.receive_input()
            self.rate.sleep()


class PoseGraphOptimization(g2o.SparseOptimizer):
    """A g2opy optimizer for graphs with one landmark vertex, many pose vertices, and many edges pose-pose and pose-landmark edges."""

    def __init__(self):
        super(PoseGraphOptimization, self).__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(PoseGraphOptimization, self).set_algorithm(solver)

        self.first_pose_vertex_id = -1 # used to create pose-pose and pose-landmark edges
        self.last_pose_vertex_id = -1 # used to create pose-pose and pose-landmark edges
        self.landmark_vertex_id = -1 # single landmark vertex for this assignment

        self.next_vertex_id = 0 # generic vertex id incrementer, used to generate ids for both pose and landmark vertices

    def get_next_vertex_id(self): # utility function to get next vertex and increment the counter
        current_id = self.next_vertex_id
        self.next_vertex_id += 1
        return current_id

    def optimize(self, max_iterations=20):
        super(PoseGraphOptimization, self).initialize_optimization()
        super(PoseGraphOptimization, self).optimize(max_iterations)

    def get_first_pose_vertex_id(self):
        return self.first_pose_vertex_id

    def get_last_pose_vertex_id(self):
        return self.last_pose_vertex_id

    def get_landmark_vertex_id(self):
        return self.landmark_vertex_id

    def add_pose_vertex(self, se2, fixed=False):
        id = self.get_next_vertex_id() # increment global pose id counter
        v_se2 = g2o.VertexSE2()
        v_se2.set_id(id) # assign vertex id
        v_se2.set_estimate(se2) # pass the se2 param (x, y, theta) as the estimate
        # v_se2.set_fixed(True if self.last_pose_vertex_id == -1 else fixed) # fix the first node
        v_se2.set_fixed(fixed)

        if self.last_pose_vertex_id == -1: # store the first pose vertex id for later use during error plotting
            self.first_pose_vertex_id = id
        self.last_pose_vertex_id = id # update last pose vertex id

        super(PoseGraphOptimization, self).add_vertex(v_se2) # register the new pose vertex
        return v_se2, id # return the new pose vertex and its id

    def add_landmark_vertex(self, xy, fixed=False):
        id = self.get_next_vertex_id() # increment global pose id counter
        v_xy = g2o.VertexPointXY()
        v_xy.set_id(id) # assign vertex id
        v_xy.set_estimate(xy) # pass the x- and y-coordinate array as the estimate
        v_xy.set_fixed(fixed)
        self.landmark_vertex_id = id # store the landmark vertex id for later use adding pose-landmark edges

        super(PoseGraphOptimization, self).add_vertex(v_xy) # register the new landmark vertex
        return v_xy, id # return the new landmark vertex and its id

    def add_pose_edge(self, vertices, se2, information=np.identity(3), robust_kernel=None):
        edge = g2o.EdgeSE2()
        for i, v in enumerate(vertices):
            if isinstance(v, int): # vertex id provided; retrieve vertex
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(se2)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)

        super(PoseGraphOptimization, self).add_edge(edge)

    def add_landmark_edge(self, vertices, xy, information=np.identity(2), robust_kernel=None):
        edge = g2o.EdgeSE2PointXY()
        for i, v in enumerate(vertices):
            if isinstance(v, int): # vertex id provided; retrieve vertex
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(xy)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)

        super(PoseGraphOptimization, self).add_edge(edge)


if __name__ == "__main__":
    rospy.init_node("graph_slam_ros")
    r = Graph_SLAM_ROS(2.04, 1.06) # instantiate Graph_SLAM_ROS with the measured distances from the wall
    r.spin()
