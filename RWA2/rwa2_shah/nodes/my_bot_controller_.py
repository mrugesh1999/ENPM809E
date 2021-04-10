#!/usr/bin/env python

import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


# Initialize your ROS node
rospy.init_node("move_robot")
# Set up a publisher to the /cmd_vel topic
pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
# Declare a message of type Twist
velocity_msg = Twist()
# publish the velocity at 4 Hz (4 times per second)
rate = rospy.Rate(4)
# set up a tf listener to retrieve transform between the robot and the world
tf_listener = tf.TransformListener()
# parent frame for the listener
parent_frame = 'odom'
# child frame for the listener
child_frame = 'base_footprint'
# gains for the proportional controllers. These values can be tuned.
k_h_gain = 0.1
k_v_gain = 0.1

try:
    tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(0.1))
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    rospy.loginfo("Cannot find transform between {p} and {c}".format(p=parent_frame, c=child_frame))
    rospy.signal_shutdown("tf Exception")

#!/usr/bin/env python

# generate random floating point values
import math
from random import seed
from random import random
# seed random number generator
seed(1)



def go_straight(distance_to_drive, linear_velocity):
    """Move the robot in a straight line until it has driven a certain distance.

    The linear velocity is modified for a Twist message and then published on /cmd_vel.

    """

    global velocity_msg
    # update linear.x from the command line
    velocity_msg.linear.x = linear_velocity
    # get the current time (s)
    t_0 = rospy.Time.now().to_sec()
    # keep track of the distance
    distance_moved = 0.0

    # while the amount of distance has not been reached
    while distance_moved <= distance_to_drive:
        rospy.loginfo("TurtleBot is moving")
        pub.publish(velocity_msg)
        rate.sleep()
        # time in sec in the loop
        t_1 = rospy.Time.now().to_sec()
        distance_moved = (t_1 - t_0) * abs(linear_velocity)
        rospy.loginfo("distance moved: {d}".format(d=distance_moved))

    rospy.logwarn("Distance reached")
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
        # rotation is a list [r, p, y]
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    # return the position (x, y, z) and the yaw
    return Point(*trans), rotation[2]


def generate_random():
    """Generate a random value between 1 and 2

    Return
    ----------
    The random value.
    """
    # generate random numbers between 0-1
    value = random()
    scaled_value = 1 + (value * (2 - 1))
    return scaled_value


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
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


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
        x = -2
        y = 0
    if node == 1:
        x = -1
        y = 2
    if node == 2:
        x = 1
        y = 2
    if node == 3:
        x = 2
        y = 0
    if node == 4:
        x = 1
        y = -2
    if node == 5:
        x = -1
        y = -2
    return x, y



def get_rotation():
    """Retrieve rotation arguments from the command line

    The first argument to retrieve is the angle (deg) to rotate.
    The second argument is the velocity at which to rotate.

    Return
    ----------
    The angle (deg) and the z angular velocity.
    """
    angle = 0
    vel = 0
    if len(sys.argv) == 4:
        angle = float(sys.argv[1])
        vel = float(sys.argv[2])
    else:
        sys.exit('Not enough arguments')
    return angle, vel


def get_distance_velocity():
    """Get distance and velocity arguments from the command line.

    The first argument to retrieve is the distance (m) to drive.
    The second argument is the velocity of the robot.

    Return
    ----------
    The distance (m)  and the velocity.
    """
    distance = 0
    vel = 0
    if len(sys.argv) == 4:
        distance = float(sys.argv[1])
        vel = float(sys.argv[2])
    else:
        sys.exit('Not enough arguments')
    return distance, vel

def go_to_goal():
    """Task the robot to reach a goal (x,y) using a proportional controller.

    The current pose of the robot is retrieved from /odom topic.
    Publish the message to /cmd_vel topic.


    """

    # get current pose of the robot from the /odom topic
    (position, rotation) = get_odom_data()
    # get the goal to reach from arguments passed to the command line
    goal_x, goal_y = get_goal()
    # compute the distance from the current position to the goal
    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
    if distance_to_goal > 0.05:
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        rospy.loginfo("x = {0}, y = {1}".format(x_start, y_start))
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)

        # the domain of arctan(x) is (-inf, inf)
        # we would like to restrict the domain to (0, 2pi)
        if angle_to_goal < -math.pi/4 or angle_to_goal > math.pi/4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        if last_rotation[-1] > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation[-1] < -math.pi + 0.1 and rotation > 0:
            rotation = -2 * math.pi + rotation

        # proportional control for rotating the robot
        while -0.1 <= angle_to_goal-rotation <= 0.1:
            velocity_msg.angular.z = k_v_gain * (angle_to_goal-rotation)
            velocity_msg.angular.z = 0.1
            pub.publish(velocity_msg.angular.z)
        velocity_msg.angular.z = 0
        distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
        # proportional control to move the robot forward
        # We will drive the robot at a maximum speed of 0.5
        velocity_msg.linear.x = min(k_h_gain * distance_to_goal, 0.5)



        # update the new rotation for the next loop
        print(velocity_msg)
        last_rotation.append(rotation)
        pub.publish(velocity_msg)
        rate.sleep()



def sensor_callback(msg):
    """Callback function to deal with messages on the /scan topic

    """
    pos = []
    neg = []
    all_ranges = []
    prev_vel = [0.5]
    zero = msg.ranges[0]
    for i in reversed(range(0, 91)):
        num = msg.ranges[i]
        if num == float('inf'):
            pos.append(10)
        else:
            pos.append(msg.ranges[i])

    for j in reversed(range(270, 360)):
        num = msg.ranges[j]
        if num == float('inf'):
            neg.append(10)
        else:
            neg.append(msg.ranges[j])

    all_ranges = pos + neg

    # todo for the assignment.
    # Call other functions to do obstacle avoidance here
    (position, rotation) = get_odom_data()
    current_x = position.x
    current_y = position.y
    x, y = get_goal()
    f = 0.4
    # s = 0.25
    if (x - .1) < current_x < (x + .1) and (y - .1) < current_y < (y + .1):
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        rospy.loginfo('Goal Reached.')
        rospy.signal_shutdown('Goal Reached.')

    if min(all_ranges) > f:
        rospy.loginfo('Go to Goal')
        go_to_goal()
    else:
        rospy.loginfo('Object Detected')

        if min(all_ranges[65:116]) < f:
            velocity_msg.linear.x = 0.0
            if sum(all_ranges[70:91]) < sum(all_ranges[90:111]):
                rospy.loginfo('Object left')
                velocity_msg.linear.x = -0.1
                velocity_msg.angular.z = -0.5
                last_rotation.append(rotation)
            else:
                rospy.loginfo('Object right')
                velocity_msg.linear.x = -0.1
                velocity_msg.angular.z = 0.5
                last_rotation.append(rotation)
        else:
            rospy.loginfo('Go to Goal')
            go_to_goal()
    pub.publish(velocity_msg)


def read_scan():
    """Set up a subscriber for the scan topic

    """
    rospy.Subscriber("scan", LaserScan, sensor_callback)
    rospy.spin()


# Run the following is this file is called from the package bot_controller
# rosrun bot_controller my_bot_controller <arguments>
if __name__ == "__main__":

    if len(sys.argv) == 2:
        node = int(sys.argv[1])
    else:
        sys.exit('Not enough arguments passed to the command line')
    last_rotation = [0]
    read_scan()
    (position, rotation) = get_odom_data()
    current_x = position.x
    current_y = position.y
    x, y = get_goal()
    if (x - .1) < current_x < (x + .1) and (y - .1) < current_y < (y + .1):
        sys.exit('Robot already at goal position.')

    while not rospy.is_shutdown():
        read_scan()
