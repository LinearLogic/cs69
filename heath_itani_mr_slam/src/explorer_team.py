#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, Odometry
from explorer import Explorer
from path_planner import PathPlanner

class ExplorerTeam:
    def __init__(self, max_distance=5, search_area_increment=750, rate=50): # max distance in meters, search area increment in sq. meters
        self.max_distance = max_distance
        self.search_area_increment = search_area_increment
        self.rate = rospy.Rate(rate)
        self.rate_raw = rate

        # init pub and subscriber
        # self.cmd_pub = rospy.Publisher('/posesequence', PoseStamped, queue_size=0)
        self.map_sub = rospy.Subscriber('/robot_0/map', OccupancyGrid, self.map_callback)

        # init map info
        self.occupancy_grid = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = False
        self.start = (0,0)
        self.goal = (0,0)
        self.map_initialized = False

        # init rendezvous transients
        self.executing_rendezvous = False
        self.rendezvous_target = (0,0)

        self.master_node = Explorer('/robot_0', is_master=True, rate=rate)
        self.drone_node = Explorer('/robot_1', rate=rate)

        self.tf_listener = tf.TransformListener()



    # callback function to obtain map info
    def map_callback(self, msg):
        self.occupancy_grid = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        if not self.map_initialized:
            self.map_initialized = True


    def execute_rendezvous(self, master_coords, drone_coords): # calculates rendezvous path and sets nav goal for robots at the provided poses
        if self.executing_rendezvous or not self.map_initialized:
            return False

        # init and run the path planner
        path_planner = PathPlanner(self.occupancy_grid, self.map_width, self.map_height, self.map_resolution, self.map_origin)
        path_planner.set_start(master_coords)
        path_planner.set_goal(drone_coords)
        rendezvous_path = path_planner.a_star()
        # abort if no path found
        if len(rendezvous_path) == 0:
            return False

        # parse the result and start nav to goal for both robots
        grid_goal = rendezvous_path[int(math.floor(len(rendezvous_path) / 2))]
        world_goal  = path_planner.grid_to_world(grid_goal)
        print 'executing rendezvous at ' + str(world_goal)
        self.master_node.start_navigation(world_goal[0], world_goal[1], 0, (master_coords[0], master_coords[1], 0))
        self.drone_node.start_navigation(world_goal[0], world_goal[1], 0, (drone_coords[0], drone_coords[1], 0))
        self.executing_rendezvous = True
        return True


    def odometry_to_pose_stamped(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.pose = msg.pose.pose
        return pose


    def get_pose_distance(self, pose_1, pose_2 ): # calculates distance between two PoseStamped messages
        x_dist = abs(pose_1.pose.position.x - pose_2.pose.position.x)
        y_dist = abs(pose_1.pose.position.y - pose_2.pose.position.y)
        return math.sqrt(x_dist ** 2 + y_dist ** 2)


    def tick(self):
        r0 = self.master_node
        r1 = self.drone_node
        if r0.is_localized and r1.is_localized: # all cooperative exploration routines require initial co-localization
            if not r0.is_navigating() and not r1.is_navigating(): # only start a new routine if none is currently in progress
                if self.executing_rendezvous: # register that rendezvous is complete if not already done
                    self.executing_rendezvous = False
                master_tf_pose = self.tf_listener.transformPose('/robot_0/map', self.odometry_to_pose_stamped(self.master_node.latest_pose))
                master_coords = (master_tf_pose.pose.position.x, master_tf_pose.pose.position.y)
                drone_tf_pose = self.tf_listener.transformPose('/robot_1/map', self.odometry_to_pose_stamped(self.drone_node.latest_pose))
                drone_coords = (drone_tf_pose.pose.position.x, drone_tf_pose.pose.position.y)
                distance = self.get_pose_distance(master_tf_pose, drone_tf_pose)
                if distance > self.max_distance:
                    self.execute_rendezvous(master_coords, drone_coords)
                    self.max_distance = math.sqrt(self.max_distance ** 2 + self.search_area_increment / math.pi)
                    print 'max distance increased to ' + str(self.max_distance) + ' meters'


    def spin(self): # tick()s individual nodes and then the team
        while not rospy.is_shutdown():
            self.master_node.tick() # tick individual nodes
            self.drone_node.tick()
            self.tick() # tick the team controller
            self.rate.sleep()


if __name__ == '__main__':
	# init ros node
    rospy.init_node('explorer_team')
    node = ExplorerTeam()
    node.spin()
