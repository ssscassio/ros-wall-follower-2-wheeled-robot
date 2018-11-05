#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sensor_msgs.msg
import math

DEFAULT_WALL_DISTANCE = 0.13    # Distance from the wall
DEFAULT_MAX_SPEED = 0.1    # Maximum speed of robot
DEFAULT_P = 10    # Proportional constant for controller
DEFAULT_D = 5    # Derivative constant for controller
DEFAULT_ANGLE = 1    # Proportional constant for angle controller (just simple P controller)
DEFAULT_DIRECTION = 1    # 1 for wall on the left side of the robot (-1 for the right side)


class NodeWallFollowing(object):
    """Demonstration task: "Wall Following", robot control.

    Robot finds nearest wall (on defined side) and goes along it.

    Attributes:
        wall_dist: Desired distance from the wall.
        max_speed: Maximum speed of the robot.
        direction: Definition of side on which robot should follow the wall (1 for wall on the right side -1 for the left one).
        p: Proportional constant for controller.
        d: Derivative constant for controller.
        angle: Proportional constant for angle controller (just simple P controller).

        e: Difference between desired distance from the wall and actual distance.
        angle_min: Angle, at which was measured the shortest distance.
        dist_front: Distance, measured by ranger in front of the robot.
        diff_e: Derivative element for PD controller.
    """

    def __init__(self, wall_dist=DEFAULT_WALL_DISTANCE, max_speed=DEFAULT_MAX_SPEED, direction=DEFAULT_DIRECTION, \
    p=DEFAULT_P, d=DEFAULT_D, angle=DEFAULT_ANGLE):

        # Creating publisher, publish (send) to topic cmdVelTopic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

        # Creating subscriber, subscribe (recieve) from topic scanTopic
        self.sub = rospy.Subscriber('/m2wr/laser/scan', sensor_msgs.msg.LaserScan, self.callback, queue_size=5)

        self.wall_dist = wall_dist
        self.max_speed = max_speed
        self.direction = direction
        self.p = p
        self.d = d
        self.angle = angle

        self.e = 0
        self.angle_min = 0    # Angle, at which was measured the shortest distance
        self.dist_front = 0
        self.diff_e = 0


    # Publisher method
    def publish_message(self):
        """Publish commands through Twist messages.

        Creates Twist message, fills in the details and generates commands for robot movement.
        """

        msg = Twist()

        # PD controller
        msg.angular.z = self.direction*(self.p*self.e+self.d*self.diff_e) + self.angle*(self.angle_min-math.pi*self.direction/2)

        if self.dist_front < self.wall_dist:
            msg.linear.x = 0
        elif self.dist_front < self.wall_dist*2:
            msg.linear.x = 0.5*self.max_speed
        elif abs(self.angle_min) > 1.75:
            msg.linear.x = 0.4*self.max_speed
        else:
            msg.linear.x = self.max_speed
        print 'Linear -- Angular %s - %s' %(msg.linear.x, msg.angular.z)
        # Publishing message
        self.pub.publish(msg)


    # Subscriber method
    def callback(self, laser_data):
        """Read data from the sensor and processes them to variables.

        Processes data from sensor.

        Args:
            laser_data: Message, which came from robot and contains data from laser scan.
        """

        # Update value of parameters
        self.update_parameters()

        # Variables with index of highest and lowest value in array
        size = len(laser_data.ranges)
        min_index = size*(self.direction+1)/4
        max_index = size*(self.direction+3)/4

        # Go through array and find minimum
        for i in range(min_index, max_index):
            if laser_data.ranges[i] < laser_data.ranges[min_index] and laser_data.ranges[i] > 0.01:
                min_index = i

        # Calculation of angles from indexes and storing data to class variables
        self.angle_min = (min_index-size/2)*laser_data.angle_increment
        dist_min = laser_data.ranges[min_index]
        self.dist_front = laser_data.ranges[size/2]
        self.diff_e = (dist_min - self.wall_dist) - self.e
        self.e = dist_min - self.wall_dist

        # Invoking method for publishing message
        self.publish_message()


    def update_parameters(self):
        """Parameters modification.

        Updates (modifies) default parameters with values from ROS server (in case there are some).
        """

        self.wall_dist = DEFAULT_WALL_DISTANCE
        self.wall_dist = rospy.get_param("/wall_following/wall_dist", self.wall_dist)
        self.max_speed = DEFAULT_MAX_SPEED
        self.max_speed = rospy.get_param("/wall_following/max_speed", self.max_speed)
        self.p = DEFAULT_P
        self.p = rospy.get_param("/wall_following/p", self.p)
        self.d = DEFAULT_D
        self.d = rospy.get_param("/wall_following/d", self.d)
        self.angle = DEFAULT_ANGLE
        self.angle = rospy.get_param("/wall_following/angle", self.angle)
        self.direction = DEFAULT_DIRECTION
        self.direction = rospy.get_param("/wall_following/direction", self.direction)


if __name__ == '__main__':
    try:
        # Node initialization
        rospy.init_node('reading_laser')

        # Creating object, which subscribes (receives) data from sensors and publish (send) them to the robot control
        node_wall_following = NodeWallFollowing()

        # spin() keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
