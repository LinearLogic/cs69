#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import numpy as np

class Kalman_Filter_ROS:
    """A ROS node that estimates linear x displacement from /cmd_vel, /pose, and /scan messages using Kalman Filters

    Args:
        measured_start_distance (float): the distance the robot is measured to be starting from the wall, in meters
        measured_end_distance (float): the robot's final distance from the wall (meters) as measured after the actuation routine is complete
        plot_rate_secs (int): the rate at which to update the plot of Kalman-estimated and measured states (every <plot_rate_secs> seconds).  Defaults to 5.
        rate (int): the robot tick rate, in Hz. Defaults to 10.
    """

    def __init__(self, measured_start_distance, measured_end_distance, plot_rate_secs=5, rate=10):
        self.measured_start_distance = measured_start_distance
        self.measured_end_distance = measured_end_distance
        self.plot_rate_secs = plot_rate_secs
        self.rate = rospy.Rate(rate)

        self.starting_distance = False # initialize the measured starting distance (from /scan) as False, flagging it to be updated with the first appropriate /scan value

        # subscribe to relevant topics
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback, queue_size=1)

        # initialize cmd_vel Kalman Filter and its publisher
        self.cmd_vel_kalman = Kalman_Filter_1D() # Kalman Filter with controls provided by /cmd_vel messages, measurements provided by /scan messages
        self.cmd_vel_kalman_pub = rospy.Publisher("cmd_vel_kalman_pose", PoseWithCovarianceStamped, queue_size=1)
        self.cmd_vel_kalman_pub_seq = -1 # the sequence used in headers of messages sent by the /cmd_vel-based Kalman Filter's publisher

        # initialize pose Kalman Filter and its publisher
        self.pose_kalman = Kalman_Filter_1D() # Kalman Filter with controls interpolated from /pose messages, measurements provided by /scan messages
        self.pose_kalman_pub = rospy.Publisher("pose_kalman_pose", PoseWithCovarianceStamped, queue_size=1)
        self.pose_kalman_pub_seq = -1 # the sequence used in headers of messages sent by the /pose-based Kalman Filter's publisher

        # initialize transients for calculating x-velocity from two /pose message x-coordinates and timestamps
        self.last_pose_x = False
        self.last_pose_timestamp = False

        # initialize arrays for plotting Kalman-estimated and measured states over time
        self.cmd_vel_kalman_x = [] # seconds elapsed
        self.cmd_vel_kalman_y = [] # linear x location estimiated by Kalman Filter parsing /cmd_vel and /scan updates

        self.pose_kalman_x = [] # seconds elapsed
        self.pose_kalman_y = [] # linear x location estimiated by Kalman Filter parsing /pose and /scan updates

        self.pose_x = [] # seconds elapsed
        self.pose_y = [] # linear x location provided by /pose updates

        self.plot_shown = False


    def cmd_vel_callback(self, msg): # used to update controls of /cmd_vel-based Kalman Filter
        self.cmd_vel_kalman.set_controls(msg.linear.x) # pass the x-velocity as the controls for the /cmd_vel-based Kalman Filter


    def scan_callback(self, msg):
        delta_t = msg.scan_time
        measurement = msg.ranges[0]

        # execute predict() step for both Kalman Filters
        self.cmd_vel_kalman.predict(delta_t)
        self.pose_kalman.predict(delta_t)

        if measurement != np.inf:
            # use the first scan to set the starting distance, and subtract future measurements from this value.
            # this is done to achieve a measurement which grows from 0, affording easier integration with /pose values
            if not self.starting_distance:
                self.starting_distance = measurement

            delta_t_secs = self.get_current_timestamp() - self.starting_timestamp

            # call update() step on /cmd_vel-based Kalman Filter, and publish the resulting state estimate and covariance
            [cmd_vel_kalman_state, cmd_vel_kalman_covariance] = self.cmd_vel_kalman.update(self.starting_distance - measurement)

            # update arrays for plotting
            self.cmd_vel_kalman_x.append(delta_t_secs)
            self.cmd_vel_kalman_y.append(cmd_vel_kalman_state)

            # prepare and publish PoseWithCovarianceStamped message containing latest state estimate and covariance for /cmd_vel-based Kalman Filter
            cmd_vel_kalman_pub_msg = self.get_covariance_pose(cmd_vel_kalman_state, cmd_vel_kalman_covariance, '/cmd_vel_kalman_map') # custom frame is /cmd_vel_kalman_map
            self.cmd_vel_kalman_pub_seq += 1
            cmd_vel_kalman_pub_msg.header.seq = self.cmd_vel_kalman_pub_seq
            self.cmd_vel_kalman_pub.publish(cmd_vel_kalman_pub_msg)

            # call update() step on /pose-based Kalman Filter, and publish the resulting state estimate and covariance
            [pose_kalman_state, pose_kalman_covariance] = self.pose_kalman.update(self.starting_distance - measurement)

            # update arrays for plotting
            self.pose_kalman_x.append(delta_t_secs)
            self.pose_kalman_y.append(pose_kalman_state)

            # prepare and publish PoseWithCovarianceStamped message containing latest state estimate and covariance for /pose-based Kalman Filter
            pose_kalman_pub_msg = self.get_covariance_pose(pose_kalman_state, pose_kalman_covariance, '/pose_kalman_map') # custom frame is /pose_kalman_map
            self.pose_kalman_pub_seq += 1
            pose_kalman_pub_msg.header.seq = self.pose_kalman_pub_seq
            self.pose_kalman_pub.publish(pose_kalman_pub_msg)
        return


    def pose_callback(self, msg): # used to update controls of /pose-based Kalman Filter
        timestamp = self.get_current_timestamp()

        delta_t_secs = timestamp - self.starting_timestamp
        self.pose_x.append(delta_t_secs)
        self.pose_y.append(msg.pose.position.x)

        if not self.last_pose_x or not self.last_pose_timestamp: # iniitial call, skip delta calculations since this is the first x and timestamp
            self.last_pose_timestamp = timestamp
            self.last_pose_x = msg.pose.position.x
            return

        # calculate delta_t and delta_x between current and previous pose, then calculate x-velocity (delta_x / delta_t)
        delta_t = timestamp - self.last_pose_timestamp
        delta_x = msg.pose.position.x - self.last_pose_x
        x_velocity = delta_x / delta_t

        self.pose_kalman.set_controls(x_velocity) # pass the computed x-velocity as the controls for the /pose-based Kalman Filter

        # store timestamp and x-coordinate of latest pose
        self.last_pose_timestamp = timestamp
        self.last_pose_x = msg.pose.position.x
        return


    def get_covariance_pose(self, state, covariance, frame_id):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()

        msg.pose.pose.position.x = state

        # note: using 1D Kalman Filter, covariance is a single value, which is used here to populate primary diagonal of 6x6 covariance matrix
        covariance_matrix = np.zeros((6,6))
        covariance_matrix[0,0] = covariance
        # convert covariance_matrix into 36x1 array (required by PoseWithCovarianceStamped spec) and add it to message
        covariance_array = np.asarray(covariance_matrix).reshape(-1)
        msg.pose.covariance = covariance_array

        return msg


    def get_current_timestamp(self): # returns timestamp incorporating seconds and nanoseconds
        now = rospy.Time.now()
        timestamp = now.secs + now.nsecs * math.pow(10, -9)
        return timestamp


    def setup_plot(self): # sets up matplotlib plot of measured and estimated states
        self.figure, self.ax = plt.subplots()
        self.cmd_vel_kalman_plot, = self.ax.plot(self.cmd_vel_kalman_x, self.cmd_vel_kalman_y, label='/cmd_vel Kalman Filter est.', lw=2, color='b')
        self.pose_kalman_plot, = self.ax.plot(self.pose_kalman_x, self.pose_kalman_y, label='/pose Kalman Filter est.', lw=1, color='r')
        self.pose_plot, = self.ax.plot(self.pose_x, self.pose_y, label='Raw /pose values', lw=1, color='g')

        self.ax.axhline(y=0, label='Measured start/end distance', color='k')
        self.ax.axhline(y=self.measured_start_distance - self.measured_end_distance, color='k')

        plt.xlabel('Elapsed time (seconds)')
        plt.ylabel('Linear x displacement (meters)')
        plt.ylim(-0.05, 1.05)
        self.ax.grid()
        self.ax.legend(loc='upper left')

        plt.pause(0.05)
        self.figure.canvas.draw()


    def update_plot(self): # updates/re-draws matplotlib plot of measured and estimated states
        self.cmd_vel_kalman_plot.set_xdata(self.cmd_vel_kalman_x)
        self.cmd_vel_kalman_plot.set_ydata(self.cmd_vel_kalman_y)
        self.pose_kalman_plot.set_xdata(self.pose_kalman_x)
        self.pose_kalman_plot.set_ydata(self.pose_kalman_y)
        self.pose_plot.set_xdata(self.pose_x)
        self.pose_plot.set_ydata(self.pose_y)

        self.ax.relim()
        self.ax.autoscale_view()

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


    def spin(self): # main loop, responsible for creating and updating the estimates/measurements plot
        self.starting_timestamp = self.get_current_timestamp()
        self.last_plotted_at_secs = rospy.Time.now().secs
        self.setup_plot()

        while not rospy.is_shutdown():
            now_secs = rospy.Time.now().secs
            if (now_secs - self.last_plotted_at_secs >= self.plot_rate_secs):
                self.update_plot()
                self.last_plotted_at_secs = now_secs
            self.rate.sleep()


class Kalman_Filter_1D:
    """ A Kalman Filter for estimation of a one-dimensional state

    Args:
        initial_state (float): the initial state estimate, default 0.
        state_transformation (float): the scalar applied to the state during the predict() step. Defaults to 1.

        initial_controls (float): the controls value, incorporated into the state estimate in the predict() step. Defaults to 0.
        controls_transformation (float): the scalar applied to the controls value during the predict() step. Defaults to 0.

        state_to_measurement_transformation (float): the scalar mapping the estimated state to a measurement estimate during the update() step. Defaults to 1.
        initial_state_covariance (float): the covariance of the initial state estimate, default 1000.

        process_noise (float): the process noise, default 1
        measurement_noise (float): the measurement noise, default 0.04 (based on observation of fluctuations in /scan values from a static robot)
    """

    def __init__(self, initial_state=0, state_transformation=1, initial_controls=0, controls_transformation=0, state_to_measurement_transformation=1, initial_state_covariance=1000, process_noise=1, measurement_noise=0.04):
        self.last_state = initial_state
        self.state_transformation = state_transformation

        self.controls = initial_controls
        self.controls_transformation = controls_transformation

        self.state_to_measurement_transformation = state_to_measurement_transformation

        self.last_state_covariance = initial_state_covariance

        self.process_noise = process_noise
        self.measurement_noise = measurement_noise


    def set_controls(self, controls): # updates the controls scalar
        self.controls = controls


    def predict(self, controls_transformation_override):
        # the prediction/propagation step of the Kalman Filter
        controls_transformation = self.controls_transformation
        if (controls_transformation_override): # override default with value in param, eg. to handle non-uniform delta_t values
            controls_transformation = controls_transformation_override

        # compute state estimate priors for use in the update() step
        self.state_a_priori = self.state_transformation * self.last_state + controls_transformation * self.controls # x'(t+1/t) = F * x'(t/t) + B * u
        self.state_covariance_a_priori = self.state_transformation * self.last_state_covariance * self.state_transformation + self.process_noise # P(t+1/t) = F * P(t/t) * F^T + Q

        # store state estimate and covariance to support multiple calls to predict() in between calls to update()
        self.last_state = self.state_a_priori
        self.last_state_covariance = self.state_covariance_a_priori


    def update(self, measurement): # the update step of the Kalman Filter
        # compute state estimate, residual, and covariance
        self.measurement_estimate = self.state_to_measurement_transformation * self.state_a_priori # z'(t+1) = H(t+1) * x'(t+1/t)
        measurement_residual = measurement - self.measurement_estimate # r(t+1) = z(t+1) - z'(t+1)
        measurement_covariance = self.state_to_measurement_transformation * self.state_covariance_a_priori * self.state_to_measurement_transformation + self.measurement_noise # S(t+1) = H * P(t+1/t) * H^T + R

        # compute the kalman gain
        kalman_gain = self.state_covariance_a_priori * self.state_to_measurement_transformation * math.pow(measurement_covariance, -1) # K(t+1) = P(t+1/t) * H^T * S(t+1)^-1

        # calculate state estimate posteriors
        state_a_posteriori = self.state_a_priori + kalman_gain * measurement_residual # x'(t+1/t+1) = x'(t+1/t) + K(t+1) * r(t+1)
        state_covariance_a_posteriori = (1 - kalman_gain * self.state_to_measurement_transformation) * self.state_covariance_a_priori # 1D reformulation of P(t+1/t+1) = P(t+1/t) - P(t+1/t) * H^T * S(t+1)^-1 * H * P(t+1/t)

        # store posterior state estimate and covariance for use in the predict() step
        self.last_state = state_a_posteriori
        self.last_state_covariance = state_covariance_a_posteriori

        # return state estimate posteriors for use elsewhere (in this case, for publication as PoseStamped message)
        return [state_a_posteriori, state_covariance_a_posteriori]


if __name__ == "__main__":
    rospy.init_node("kalman_filter_ros")
    r = Kalman_Filter_ROS(2.04, 1.06, 1) # instantiate Kalman_Filter_ROS with the measured distances from the wall, configured to update plot every 1 second
    r.spin()
