#!/usr/bin/env python
import math
import numpy
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav2d_navigator.msg import MoveToPosition2DActionGoal, MoveToPosition2DActionResult
from actionlib_msgs.msg import GoalID

class Explorer:

    def __init__(self, base_topic, is_master=False, rate=10):
        self.base_topic = base_topic
        self.is_master = is_master
        self.rate_raw = rate
        self.rate = rospy.Rate(rate)

        # init localization, pose, and nav goal id transients
        self.has_particles = False
        self.is_localized = is_master
        self.latest_pose = Odometry()
        self.navigation_goal_incrementer = -1
        self.navigation_goal_id = '-1'
        self.anchor_pose = (0,0,0)
        self.anchor_pose_set = False

        # initialize wall-follower transients
        self.wf_state = 0 # init FSM (0 = wait, 1 = rotate, 2 = follow wall)

         # init wall following transients and default settings
        self.init_wall_following()
        self.configure_wall_following()

        # init subscribers (/base_scan, /operator_cmd_vel, /MoveTo/result; if not master node, /localization_result, /Mapper/particles)
        self.scan_sub = rospy.Subscriber(base_topic + '/base_scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(base_topic + '/odom', Odometry, self.odom_callback, queue_size=1)
        self.operator_cmd_vel_sub = rospy.Subscriber(base_topic + '/operator_cmd_vel', Twist, self.operator_cmd_vel_callback, queue_size=1)
        self.move_result_sub = rospy.Subscriber(base_topic + '/MoveTo/result', MoveToPosition2DActionResult, self.move_result_callback, queue_size=1)
        if not is_master:
            self.localization_sub = rospy.Subscriber(base_topic + '/localization_result', PoseStamped, self.localization_callback)
            self.particles_sub = rospy.Subscriber(base_topic + '/Mapper/particles', PoseArray, self.particles_callback)

        # init publishers (/cmd_vel, /MoveTo/goal, /MoveTo/cancel)
        self.cmd_vel_pub = rospy.Publisher(base_topic + '/cmd_vel', Twist, queue_size=0)
        self.move_goal_pub = rospy.Publisher(base_topic + '/MoveTo/goal', MoveToPosition2DActionGoal, queue_size=0)
        self.move_cancel_pub = rospy.Publisher(base_topic + '/MoveTo/cancel', GoalID, queue_size=0)


    def init_wall_following(self): # initializes transients used for wall-following
        self.wf_error = 0 # difference between current and target distance from the wall
        self.wf_prev_error = 0 # error from the last tick
        self.wf_angle_of_min_range = 0 # angle (relative to robot) of min_range (this is used to modulate the PD output to reduce oscillation. full disclosure: i couldn't figure out how to track a satisfactorily straight path without factoring this into the PD equation)
        self.wf_min_range = 0 # current shortest scan range, less than or equal to max range threshold
        self.wf_min_range_raw = 0 # true min range, possibly greater than max range threshold
        self.wf_prev_min_range = 0 # shortest scan range persisted from the last tick
        self.wf_min_ranges_by_zone = { # portions of the scanner's field of view
                'side_right': 0,
                'front_right': 0,
                'front': 0,
                'front_left': 0,
                'side_left': 0,
        }
        self.wf_prev_angular_vel = 0 # previous angular vel, used to modulate robot rotation when not near a wall
        self.wf_paused_at = 0 # time at which the robot last paused to wait for an obstacle to move


    # sets parameters controlling wall-following
    def configure_wall_following(self, max_range=2, target_wall_distance=0.7, linear_vel=1.0, angular_vel=2.0, p_gain=15.0, d_gain=5.0, obstacle_wait_time=0.2):
        self.wf_max_range = max_range
        self.wf_target_wall_distance = target_wall_distance
        self.wf_linear_vel = linear_vel
        self.wf_angular_vel = angular_vel
        self.wf_p_gain = p_gain
        self.wf_d_gain = d_gain
        self.wf_obstacle_wait_time = obstacle_wait_time


    def scan_callback(self, msg):
        if self.navigation_goal_id != '-1': # ignore scan msgs (used for wall following) if nav to goal is underway
            return

        range_count = len(msg.ranges)
        self.wf_prev_min_range = self.wf_min_range # store the previous min range before finding the new min range
        self.wf_prev_error = self.wf_error # store the previous error before calculating the new error

        # arbitrary initial values, updated below
        self.wf_min_range = self.wf_max_range
        self.wf_min_range_raw = min(msg.ranges)
        min_range_index = 0

        # find the smallest scan range and its index within msg.ranges
        for i in range(0, range_count):
            if msg.ranges[i] < self.wf_min_range:
                self.wf_min_range = msg.ranges[i]
                min_range_index = i

        # calculate pertinent variables now that we have the min_range and its index in msg.ranges
        if self.wf_min_range < self.wf_max_range: # only update angle of min range if min range is below max range cutoff
            self.wf_angle_of_min_range = (min_range_index - range_count/2)*msg.angle_increment
        self.wf_error = self.wf_min_range - self.wf_target_wall_distance

        # find the min range in each portion of the robot's FOV. these will be used to handle edge cases (encountering an obstacle or getting too close to the wall and pausing translation while rotating toward an unobstructed path)
        region_range_count = range_count/7 # the number of ranges in each region. note: i divide by an odd number so that the 'front' region is directly in front, and 7 is chosen as it provides a reasonable region_range_count (angle of FOV).
        self.wf_min_ranges_by_zone = {
            'back_right': min(min(msg.ranges[0 : region_range_count - 1]), self.wf_max_range),
            'side_right': min(min(msg.ranges[region_range_count : 2*region_range_count - 1]), self.wf_max_range),
            'front_right':  min(min(msg.ranges[2*region_range_count : 3*region_range_count - 1]), self.wf_max_range),
            'front':  min(min(msg.ranges[3*region_range_count : 4*region_range_count - 1]), self.wf_max_range),
            'front_left':   min(min(msg.ranges[4*region_range_count : 5*region_range_count - 1]), self.wf_max_range),
            'side_left':   min(min(msg.ranges[5*region_range_count : 6*region_range_count - 1]), self.wf_max_range),
            'back_left':   min(min(msg.ranges[6*region_range_count : 7*region_range_count - 1]), self.wf_max_range),
        }

        if min(self.wf_min_ranges_by_zone['front'], self.wf_min_ranges_by_zone['front_left'], self.wf_min_ranges_by_zone['front_right']) < self.wf_target_wall_distance*0.9: # check 90% of wall distance instead of full wall distance to prevent pauses for rotation during normal wall following
            if self.wf_state == 2: # currently following wall, need to stop and wait before rotating
                self.wf_paused_at = rospy.Time.now().to_sec() # store the time at which we started waiting
                self.set_state(0) # pause
            elif self.wf_state == 0 and rospy.Time.now().to_sec() - self.wf_paused_at >= self.wf_obstacle_wait_time: # we have waiting long enough, proceed to rotate to an unobstructed path
                self.set_state(1) # rotate
        else:
            self.set_state(2) # follow wall


    def odom_callback(self, msg): # store latest Odometry message
        self.latest_pose = msg


    # intercepts remapped cmd_vel messages sent by Operator, publishing them to the true /cmd_vel topic
    # only if goal-based nav is underway (otherwise they'll conflict with cmd_vel msgs for wall-following)
    def operator_cmd_vel_callback(self, msg):
        if self.navigation_goal_id != '-1':
            self.cmd_vel_pub.publish(msg)


    def move_result_callback(self, msg): # once nav goal is reached, cancel nav and resume to wall-following, or select a new frontier goal
        self.stop_navigation()
        if self.anchor_pose_set: # if an anchor pose has been specified, nav to it
            self.start_navigation(self.anchor_pose[0], self.anchor_pose[1], self.anchor_pose[2])
            self.anchor_pose_set = False
            print 'returning to anchor pose at ' + str(self.anchor_pose)
            return
        else:
            print 'arrived at goal, resuming exploration'
        # todo: frontier selection if no anchor was specified


    def localization_callback(self, msg):
        if self.is_master or self.is_localized: # master explorer node is pre-localized; avoid redundant localization handling
            return
        print 'localized ' + self.base_topic
        self.is_localized = True
        # todo: commence cooperative frontier selection


    def particles_callback(self, msg):
        if self.is_master or self.has_particles: # master explorer does not use particle filter to localize; only handle first msg (to trigger movement to refine the particle filter)
            return
        print 'initialized particle filter for ' + self.base_topic
        self.has_particles = True
        # todo: commence movement to refine filter


    def set_state(self, new_state):
        if new_state != self.wf_state:
            self.wf_state = new_state


    def follow_wall(self): # apply constant linear velocity and set angular velocity via PD controller
        msg = Twist()
        msg.linear.x = self.wf_linear_vel
        if self.wf_min_range_raw <= self.wf_max_range * 5: # if no wall nearby, go straight instead of turning based on error
            d_error = abs(self.wf_error - self.wf_prev_error) / self.rate_raw # using the formula from the class slides
            angular_vel = -(self.wf_p_gain*self.wf_error + self.wf_d_gain*d_error) # the clause with angle_of_min_range was the only way I was able to reduce oscillation enough to satisfy my OCD :)
            if abs(angular_vel) > self.wf_linear_vel:
                angular_vel = numpy.sign(angular_vel) * self.wf_linear_vel
            if self.wf_angle_of_min_range > 0: # rotate away from the side containing the smallest range
                msg.angular.z = -angular_vel
            else:
                msg.angular.z = angular_vel
            msg.angular.z = msg.angular.z * self.wf_max_range / self.wf_min_range_raw # modulate speed by closeness to wall (closer -> move faster; further -> move slower to avoid going in circles)
        else:
            msg.angular.z = self.wf_prev_angular_vel * 0.99 # gradually tapir angular vel to zero
        self.wf_prev_angular_vel = msg.angular.z
        return msg


    def rotate(self): # rotate out of a sticky situation. linear vel is set to 0 and angular vel is applied in the direction that is more open (i.e. min scanner range is greater in that direction than in the other)
        msg = Twist()
        msg.linear.x = 0
        if self.wf_min_ranges_by_zone['front_left'] + self.wf_min_ranges_by_zone['side_left'] + self.wf_min_ranges_by_zone['back_left'] > self.wf_min_ranges_by_zone['front_right'] + self.wf_min_ranges_by_zone['side_right'] + self.wf_min_ranges_by_zone['back_right']: # if there's more space on the left, rotate left, otherwise rotate right
            msg.angular.z = self.wf_angular_vel
        else:
            msg.angular.z = -self.wf_angular_vel
        return msg


    # commences nav to the provided coords, with optional anchor coords to return to after goal is reached
    def start_navigation(self, x, y, theta, anchor_pose=False):
        if self.navigation_goal_id != '-1': # only commence nav if not already underway
            return

        # init goal msg
        msg = MoveToPosition2DActionGoal()

        # increase nav goal id incrementer and use its value as this nav message's goal id
        self.navigation_goal_incrementer += 1
        goal_id = str(self.navigation_goal_incrementer)
        msg.goal_id.id = goal_id

        # set frame_id as the robot's localized copy of the global map
        frame_id = self.base_topic + '/map'
        msg.header.frame_id = frame_id
        msg.goal.header.frame_id = frame_id

        # set goal pose
        msg.goal.target_pose.x = x
        msg.goal.target_pose.y = y
        msg.goal.target_pose.theta = theta

        # set anchor pose, if provided
        if anchor_pose != False:
            self.anchor_pose = anchor_pose
            self.anchor_pose_set = True

        # publish message and update current nav goal id
        self.move_goal_pub.publish(msg)
        self.navigation_goal_id = goal_id


    def stop_navigation(self):
        if self.navigation_goal_id == '-1': # only cancel nav if currently underway
            return
        msg = GoalID() # init cancel message
        msg.id = self.navigation_goal_id # specify current goal id
        self.move_cancel_pub.publish(msg) # publish message
        self.navigation_goal_id = '-1' # update current nav goal id


    def is_navigating(self):
        return self.navigation_goal_id != '-1'


    def tick(self): # essential logic from the spin() loop as a separate function suitable to be called by ExplorerTeam's spin() method (rather than spin()-ing each Explorer)
        if self.navigation_goal_id == '-1' and (self.is_master or self.has_particles or self.is_localized): # don't override nav to goal if underway; don't move non-masters until their particle filters have initialized or localization has occurred
            msg = Twist() # default message, published if robot is in 'waiting' state
            if self.wf_state == 1:
               msg = self.rotate()
            elif self.wf_state == 2:
               msg = self.follow_wall()
            self.cmd_vel_pub.publish(msg)


    def spin(self): # spin() command for standalone operation (versus controlled by ExplorerTeam)
        while not rospy.is_shutdown():
            self.tick()
            self.rate.sleep()


if __name__ == '__main__': # init and run standalone Explorer
    rospy.init_node('master_explorer_node')
    master = Explorer('/robot_0', is_master=True)
    master.spin()
