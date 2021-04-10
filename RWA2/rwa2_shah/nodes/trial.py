#!/usr/bin/env python

# Importing required libraries
import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

rospy.init_node("move_robot")                                   # Initiate ROS node
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)           # Set up a publisher to the /cmd_vel topic
velocity_msg = Twist()                                          # Declare a message of type Twist
rate = rospy.Rate(4)                                            # publish the velocity at 4 Hz (4 times per second)
tf_listener = tf.TransformListener()                            # set up a tf listener to retrieve transform
parent_frame = 'odom'                                           # parent frame for the listener
child_frame = 'base_footprint'                                  # child frame for the listener
k_h_gain = 1                                                    # gains for the proportional controllers
k_v_gain = 1                                                    # These values can be tuned.
last_rotation = [0]                                             # Initializing list to keep track of last rotation

try:
    tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1))
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo("Cannot find transform between {p} and {c}".format(p=parent_frame, c=child_frame))
    rospy.signal_shutdown("tf Exception")


def compute_distance(x1, y1, x2, y2):
    """Compute the distance between 2 points.

    Parameters
    ----------
    x1 : float
        x coordinate of the first point.
    y1 : float
        y coordinate of the first point.
    x2 : float
        x coordinate of the second point.
    y2 : float
        y coordinate of the second point.

    Return
    ----------
    The distance between between a point (x1,y1) and another point (x2,y2).
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def go_straight(distance_to_drive, linear_velocity):
    """Move the robot in a straight line until it has driven a certain distance.
    The linear velocity is modified for a Twist message and then published on /cmd_vel.
    """

    global velocity_msg
    velocity_msg.linear.x = linear_velocity
    t_0 = rospy.Time.now().to_sec()                                 # get the current time (s)
    distance_moved = 0.0                                            # keep track of the distance

    # while the amount of distance has not been reached
    if distance_moved <= distance_to_drive:
        rospy.loginfo("TurtleBot is moving")
        pub.publish(velocity_msg)
        rate.sleep()
        t_1 = rospy.Time.now().to_sec()                             # time in sec in the loop
        distance_moved = (t_1 - t_0) * abs(linear_velocity)
        rospy.loginfo("distance moved: {0}".format(distance_moved))

    rospy.loginfo("Distance reached")

    # finally, stop the robot when the distance is moved
    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)


def rotate(relative_angle_degree, angular_velocity):
    """Make the robot rotate in place
    The angular velocity is modified before publishing the message on the topic /cmd_vel.
    """
    velocity_msg.angular.z = angular_velocity

    t0 = rospy.Time.now().to_sec()
    while True:
        rospy.loginfo("TurtleBot is rotating")
        pub.publish(velocity_msg)
        rate.sleep()
        t1 = rospy.Time.now().to_sec()
        rospy.loginfo("t0: {t}".format(t=t0))
        rospy.loginfo("t1: {t}".format(t=t1))
        current_angle_degree = (t1 - t0) * angular_velocity

        rospy.loginfo("current angle: {a}".format(a=current_angle_degree))
        rospy.loginfo("angle to reach: {a}".format(a=relative_angle_degree))
        if abs(current_angle_degree) >= math.radians(abs(relative_angle_degree)):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)


def get_odom_data():
    """Get the current pose of the robot from the /odom topic
    Return
    ----------
    The position (x, y, z) and the yaw of the robot.
    """

    try:
        (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)                               # rotation is a list [r, p, y]
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    return Point(*trans), rotation[2]                                       # return the position (x, y, z) and the yaw


def get_goal():
    """Get goal arguments from the command line
    The first argument is the x coordinate.
    The second argument is the y coordinate.
    Return
    ----------
    The goal (x, y) to reach.
    """
    x = int
    y = int
    if node == 0:
        x = float(-2)
        y = float(0)
    elif node == 1:
        x = float(-1)
        y = float(2)
    elif node == 2:
        x = float(1)
        y = float(2)
    elif node == 3:
        x = float(2)
        y = float(0)
    elif node == 4:
        x = float(1)
        y = float(-2)
    elif node == 5:
        x = float(-1)
        y = float(-2)
    return x, y


def get_rotation():
    """Retrieve rotation arguments from the command line
    The first argument to retrieve is the angle (deg) to rotate.
    The second argument is the velocity at which to rotate.
    Return
    ----------
    The angle (deg) and the z angular velocity.
    """
    if len(sys.argv) == 4:
        angle = float(sys.argv[1])
        vel = float(sys.argv[2])
        return angle, vel
    else:
        sys.exit('Not enough arguments')


def get_distance_velocity():
    """Get distance and velocity arguments from the command line.
    The first argument to retrieve is the distance (m) to drive.
    The second argument is the velocity of the robot.
    Return
    ----------
    The distance (m)  and the velocity.
    """
    if len(sys.argv) == 4:
        distance = float(sys.argv[1])
        vel = float(sys.argv[2])
        return distance, vel
    else:
        sys.exit('Not enough arguments')


def go_to_goal():
    """Task the robot to reach a goal (x,y) using a proportional controller.
    The current pose of the robot is retrieved from /odom topic.
    Publish the message to /cmd_vel topic.
    """

    (position, rotation) = get_odom_data()
    goal_x, goal_y = get_goal()
    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)

    if distance_to_goal > 0.05:
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)
        if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal

        # From list of last rotation accessing last rotation
        if last_rotation[-1] > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation[-1] < -math.pi + 0.1 and rotation > 0:
            rotation = -2 * math.pi + rotation
        velocity_msg.angular.z = k_v_gain * angle_to_goal-rotation
        distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
        if -.07 <= velocity_msg.angular.z <= .07:
            velocity_msg.linear.x = min(k_h_gain * distance_to_goal, 0.15)

        # set the z angular velocity for positive and negative rotations
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, .2)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -.2)

        last_rotation.append(rotation)
        pub.publish(velocity_msg)
        rate.sleep()

    else:
        rospy.logwarn("We have reached the Goal !!")
        rospy.logwarn("Hope you had an awesome journey!")

    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)


def sensor_callback(msg):
    """Callback function to deal with messages on the /scan topic
    """

    # Initializing the data set lists
    left_data_set = []
    right_data_set = []

    for i in range(0, 91):
        distance = msg.ranges[i]
        if math.isinf(distance):
            left_data_set.append(10)
        else:
            left_data_set.append(msg.ranges[i])

    for j in range(270, 360):
        distance = msg.ranges[j]
        if math.isinf(distance):
            right_data_set.append(10)
        else:
            right_data_set.append(msg.ranges[j])

    complete_spectrum = left_data_set + right_data_set
    threshold = 0.4
    (position, rotation) = get_odom_data()
    if compare_node_goal():
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        rospy.signal_shutdown('We are at the goal !!')

    if min(complete_spectrum) > threshold:
        rospy.loginfo('Approaching the goal, please wait !!')
        go_to_goal()
    else:
        rospy.loginfo('Object has been detected')

        if min(complete_spectrum[0:26]) < threshold and min(complete_spectrum[-26:]) < threshold:
            velocity_msg.linear.x = 0.0
            if sum(complete_spectrum[0:21]) < sum(complete_spectrum[-21:]):
                rospy.loginfo('Object has been detected on the LEFT')
                velocity_msg.linear.x = -0.2
                velocity_msg.angular.z = -0.7
                last_rotation.append(rotation)
            else:
                rospy.loginfo('Object has been detected on the RIGHT')
                velocity_msg.linear.x = -0.2
                velocity_msg.angular.z = 0.7
                last_rotation.append(rotation)
        else:
            velocity_msg.linear.x = 0.3
            velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)


def read_scan():
    """Set up a subscriber for the scan topic
    """
    rospy.Subscriber("scan", LaserScan, sensor_callback)
    rospy.spin()


def compare_node_goal():
    (pose, orientation) = get_odom_data()
    present_x = pose.x
    present_y = pose.y
    goal_x, goal_y = get_goal()
    if (goal_x - .1) < present_x < (goal_x + .1) and (goal_y - .1) < present_y < (goal_y + .1):
        return True
    else:
        return False


if __name__ == "__main__":
    if len(sys.argv) == 2:
        node = int(sys.argv[1])
    else:
        sys.exit('Not enough arguments passed to the command line')
    if 0 <= node <= 5:
        if compare_node_goal():
            rospy.logwarn("Robot is at the Goal node only !!")
            sys.exit("Thank you !!")

        else:
            rospy.logwarn("Off to the Node number {0}".format(node))
            while not rospy.is_shutdown():
                read_scan()
    else:
        sys.exit('Node given does NOT exist !!')
