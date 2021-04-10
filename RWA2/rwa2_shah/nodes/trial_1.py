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
pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
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
k_h_gain = 1
k_v_gain = 1

try:
    tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))
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


def go_straight(dist, vel):
    """Move the robot in a straight line until it has driven a certain distance.
    The linear velocity is modified for a Twist message and then published on /cmd_vel.
    """

    (position, rotation) = get_odom_data()
    x_start = position.x
    y_start = position.y
    # get distance and linear velocity from command line
    distance_to_drive, linear_velocity = dist, vel
    global velocity_msg
    # update linear.x from the command line
    velocity_msg.linear.x = linear_velocity
    # get the current time (s)
    t_0 = rospy.Time.now().to_sec()
    # keep track of the distance
    distance_moved = 0.0

    # while the amount of distance has not been reached
    if distance_moved <= distance_to_drive:
        rospy.loginfo("TurtleBot is moving")
        pub.publish(velocity_msg)
        rate.sleep()
        # time in sec in the loop
        t_1 = rospy.Time.now().to_sec()
        distance_moved = (t_1 - t_0) * abs(linear_velocity)
        rospy.loginfo("distance moved: {0}".format(distance_moved))

        last_distance = distance_moved

    rospy.loginfo("Distance reached")
    # finally, stop the robot when the distance is moved
    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)


def rotate(angle, vel):
    """Make the robot rotate in place
    The angular velocity is modified before publishing the message on the topic /cmd_vel.
    """

    # angular_velocity = math.radians(angular_velocity)
    relative_angle_degree = angle
    angular_velocity = vel
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


def get_goal():
    """Get goal arguments from the command line
    The first argument is the x coordinate.
    The second argument is the y coordinate.
    Return
    ----------
    The goal (x, y) to reach.
    """

    x = 0
    y = 0
    if len(sys.argv) == 2:
        coords = int(sys.argv[1])
        if coords == 0:
            x = float(-2)
            y = float(0)
        elif coords == 1:
            x = float(-1)
            y = float(2)
        elif coords == 2:
            x = float(1)
            y = float(2)
        elif coords == 3:
            x = float(2)
            y = float(0)
        elif coords == 4:
            x = float(1)
            y = float(-2)
        elif coords == 5:
            x = float(-1)
            y = float(-2)
        else:
            sys.exit('Location submitted is out of range')
    else:
        sys.exit('Not enough arguments')
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
    vel = .2
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


last_rotation = [0]


def go_to_goal():
    """Task the robot to reach a goal (x,y) using a proportional controller.
    The current pose of the robot is retrieved from /odom topic.
    Publish the message to /cmd_vel topic.
    """

    # get current pose of the robot from the /odom topic
    (position, rotation) = get_odom_data()
    # get the goal to reach from arguments passed to the command line
    goal_x, goal_y = get_goal()
    rospy.loginfo("x Goal = {0}, y Goal = {1}".format(goal_x, goal_y))
    # compute the distance from the current position to the goal
    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)

    if distance_to_goal > 0.03:
        (position, rotation) = get_odom_data()
        x_start = position.x
        y_start = position.y
        rospy.loginfo("x = {0}, y = {1}".format(x_start, y_start))
        angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)

        # the domain of arctan(x) is (-inf, inf)
        # we would like to restrict the domain to (0, 2pi)
        if angle_to_goal < -math.pi / 4 or angle_to_goal > math.pi / 4:
            if 0 > goal_y > y_start:
                angle_to_goal = -2 * math.pi + angle_to_goal
            elif 0 <= goal_y < y_start:
                angle_to_goal = 2 * math.pi + angle_to_goal
        if last_rotation[-1] > math.pi - 0.1 and rotation <= 0:
            rotation = 2 * math.pi + rotation
        elif last_rotation[-1] < -math.pi + 0.1 and rotation > 0:
            rotation = -2 * math.pi + rotation

        # proportional control for rotating the robot
        velocity_msg.angular.z = k_v_gain * angle_to_goal - rotation

        distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
        # proportional control to move the robot forward
        # We will drive the robot at a maximum speed of 0.5
        rospy.loginfo("Angular velocity: {a}".format(a=velocity_msg.angular.z))
        if -.07 <= velocity_msg.angular.z <= .07:
            velocity_msg.linear.x = min(k_h_gain * distance_to_goal, 0.15)

        # set the z angular velocity for positive and negative rotations
        if velocity_msg.angular.z > 0:
            velocity_msg.angular.z = min(velocity_msg.angular.z, .2)
        else:
            velocity_msg.angular.z = max(velocity_msg.angular.z, -.2)

        # update the new rotation for the next loop
        last_rotation.append(rotation)
        pub.publish(velocity_msg)
        rate.sleep()

    # force the robot to stop by setting linear and angular velocities to 0
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    # publish the new message on /cmd_vel topic
    pub.publish(velocity_msg)


def object_block(f, fl, fr, l, r):
    velocity_msg.linear.x = 0.0
    velocity_msg.angular.z = 0.0
    go_straight(.2, -.2)
    pub.publish(velocity_msg)

    # dir_list = [f, fl, fr]
    # dir_ind = ['f', 'fl', 'fr']
    # dir_index = dir_list.index(min(dir_list))
    # object_dir = dir_ind[dir_index]
    # (position, rotation) = get_odom_data()
    # rospy.loginfo("Position before: {p}".format(p=position))
    # rospy.loginfo("Rotation before: {r}".format(r=rotation))
    # if object_dir == 'f':
    #    rotate(70, 0.3)
    # elif object_dir == 'fr':
    #    rotate(60, 0.3)
    # else:
    # rotate(60, -0.3)


def distance(angle, front, side):
    if 0 < angle < 45 or 315 < angle < 360:
        distance = front * math.tan(math.radians(angle))
    elif 90 > angle > 45 or 270 > angle > 315:
        distance = side * math.tan(math.radians(angle))
    else:
        distance = math.sqrt(front ** 2 + side ** 2)

    return distance


def sensor_callback(msg):
    """Callback function to deal with messages on the /scan topic
    """

    zero = msg.ranges[0]
    # fiftteen = msg.ranges[15]
    # neg_fifteen = msg.ranges[345]
    # thirty = msg.ranges[30]
    # neg_thirty = msg.ranges[330]
    # forty_five = msg.ranges[45]
    # neg_forty_five = msg.ranges[315]
    pos = []
    neg = []
    all_ranges = []
    prev_vel = [0.5]
    zero = msg.ranges[0]
    for i in range(0, 91, 10):
        num = msg.ranges[i]
        if math.isinf(num):
            pos.append(6)
        else:
            pos.append(msg.ranges[i])

    for j in range(270, 360, 10):
        num = msg.ranges[j]
        if math.isinf(num):
            neg.append(6)
        else:
            neg.append(msg.ranges[j])

    all_ranges = pos + neg

    # todo for the assignment.
    # Call other functions to do obstacle avoidance here
    (position, rotation) = get_odom_data()
    current_x = position.x
    current_y = position.y
    x, y = get_goal()
    f = 0.5
    # s = 0.25
    if (x - .1) < current_x < (x + .1) and (y - .1) < current_y < (y + .1):
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        rospy.loginfo('Goal Reached.')
        rospy.signal_shutdown('Goal Reached.')

    # if zero > f and pos[0] > distance(15, f, s) and neg[-1] > distance(345, f, s) and pos[1] > distance(30, f, s) and \
    #    neg[-2] > distance(330, f, s) and pos[2] > distance(45, f, s) and neg[-3] > distance(315, f, s) and \
    #    pos[3] > distance(60, f, s) and neg[-4] > distance(300, f, s) and pos[4] > distance(75, f, s) and \
    #    neg[-5] > distance(285, f, s) and pos[5] > s and neg[-6] > s:
    # go_to_goal()

    # if zero > f and pos[0] > f and neg[-1] > f and pos[1] > f and neg[-2] > f and pos[2] > f-.1 and \
    #        neg[-3] > f-.1 and pos[3] > .15 and neg[-4] > f-.15 and pos[4] > f-.2 and neg[-5] > f-.2 and \
    #        pos[5] > f-.2 and neg[-6] > f-.2 and msg.ranges[35] > f-.05 and msg.ranges[325] > f-.05:
    #    velocity_msg.linear.x = 0.0
    #    velocity_msg.angular.z = 0.0
    #    rospy.loginfo('Go to Goal')
    #    go_to_goal()
    if min(all_ranges) > f:
        rospy.loginfo('Go to Goal')
        go_to_goal()
    else:
        rospy.loginfo('Object Detected')

        if min(all_ranges[0:4]) < f or min(all_ranges[-4:]):
            velocity_msg.linear.x = 0.0
            if sum(all_ranges[0:3]) < sum(all_ranges[-3:]):
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
            velocity_msg.linear.x = 0.4
            velocity_msg.angular.z = 0.0

        # if zero > f and pos[0] > f and neg[-1] > f and pos[1] > f and neg[-2] > f and pos[2] > f-.05 and neg[-3] > f-.05:
        # velocity_msg.linear.x = 0.5
        # velocity_msg.angular.z = 0.0

    pub.publish(velocity_msg)

    # if front > 0.6 and front_left > 0.6 and front_right > 0.6 and left > 0.3 and right > 0.3:
    #    go_to_goal()
    # else:
    #    velocity_msg.linear.x = 0.0
    #    velocity_msg.angular.z = 0.5
    #    if left < 0.3 or right < 0.3:
    #        velocity_msg.linear.x = 0.5
    #        velocity_msg.angular.z = 0.0
    # pub.publish(velocity_msg)


def read_scan():
    """Set up a subscriber for the scan topic
    """
    rospy.Subscriber("scan", LaserScan, sensor_callback)
    rospy.spin()


# Run the following is this file is called from the package bot_controller
# rosrun bot_controller my_bot_controller <arguments>
if __name__ == "__main__":
    action = ""
    if len(sys.argv) == 2:
        action = int(sys.argv[1])
    else:
        sys.exit('Not enough arguments passed to the command line')
    if 0 <= action <= 5:
        (position, rotation) = get_odom_data()
        current_x = position.x
        current_y = position.y
        x, y = get_goal()
        if (x - .1) < current_x < (x + .1) and (y - .1) < current_y < (y + .1):
            sys.exit('Robot already at goal position.')

        while not rospy.is_shutdown():
            read_scan()
    else:
        sys.exit('Unknown argument')
