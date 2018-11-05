#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

size = 1008 #nr of samples
DEFAULT_WALL_DISTANCE = 0.13    # Distance from the wall
DEFAULT_MAX_SPEED = 0.1    # Maximum speed of robot
DEFAULT_P = 10    # Proportional constant for controller
DEFAULT_D = 5    # Derivative constant for controller
DEFAULT_ANGLE = 1    # Proportional constant for angle controller (just simple P controller)
DEFAULT_DIRECTION = 1    # 1 for wall on the left side of the robot (-1 for the right side)

pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def clbk_laser(msg):
    global e, angleMin, diffE, distFront, angleCoef, wallDistance, maxSpeed, direction, P, D, PI

    minIndex = size * (direction+1)/4
    maxIndex = size*(direction+3)/4

    distMin, idx = min((distMin, idx) for (idx, distMin) in enumerate(msg.ranges[0:1007]))
    angleMin = (minIndex-size/2)*msg.angle_increment
    distFront = msg.ranges[size/2];
    diffE = (distMin - wallDistance) - e;
    e = distMin - wallDistance;

    publishMessage()
    #print 'msg.angle_increment- %s' % (msg.angle_increment)
    #print 'minIndex maxIndex- [%s] - %s' % (minIndex, maxIndex)
    #print 'Angle Distance- [%s] - %s' % (angleMin, distMin)

    #global regions_
    #regions_ = {
    #    'bright':  min(min(msg.ranges[0:143]), 10),
    #    'right': min(min(msg.ranges[144:287]), 10),
    #    'fright':  min(min(msg.ranges[288:431]), 10),
    #    'front':  min(min(msg.ranges[432:575]), 10),
    #    'fleft':  min(min(msg.ranges[576:719]), 10),
    #    'left':   min(min(msg.ranges[720:863]), 10),
    #    'bleft':   min(min(msg.ranges[864:1007]), 10),
    #}

    #take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.2
    return msg

def follow_the_wall():
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg



def publishMessage():
    global e, angleMin, diffE, distFront, angleCoef, wallDistance, maxSpeed, direction, P, D, PI
    msg = Twist()
    msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2) # PD controller
    if distFront < wallDistance*2:
        msg.linear.x =0
    elif distFront < wallDistance*2:
        msg.linear.x=0.5*maxSpeed
    elif abs(angleMin)>1.75:
        msg.linear.x= 0.4*maxSpeed;
    else :
        msg.linear.x = maxSpeed;

    pub_.publish(msg)
    print 'error Difference, fdist, angleCoef, angleMin, [%s] - %s - %s - %s' % (diffE, distFront, angleCoef, angleMin)
    print 'published z, x- [%s] - %s' % (msg.angular.z, msg.linear.x)


def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        #if state_ == 0:
        #    msg = find_wall()
        #elif state_ == 1:
        #    msg = turn_left()
        #elif state_ == 2:
        #    msg = follow_the_wall()
        #    pass
        #else:
        #    rospy.logerr('Unknown state!')
        
        #pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
