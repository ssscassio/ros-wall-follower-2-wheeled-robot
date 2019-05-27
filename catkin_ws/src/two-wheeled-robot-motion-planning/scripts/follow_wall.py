#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from tf import transformations
#from datetime import datetime

# Util imports
import random
import math
import time

hz = 20                     # Cycle Frequency
loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 5                     # Limit to Laser sensor range in meters, all distances above this value are 
                            #      considered out of sensor range
wall_dist = 0.5             # Distance desired from the wall
max_speed = 0.3             # Maximum speed of the robot on meters/seconds
p = 15                      # Proportional constant for controller  
d = 0                       # Derivative constant for controller 
angle = 1                   # Proportional constant for angle controller (just simple P controller)
direction = -1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

# Time when the last outer corner; direction and inner corner were detected or changed.
last_outer_corner_detection_time = time.time()
last_change_direction_time = time.time()
last_inner_corner_detection_time = time.time()
rotating = 0 
pub_ = None
# Sensor regions
regions_ = {
        'bright': 0,
        'right': 0,
        'fright': 0,
        'front': 0,
        'left': 0,
}
last_kinds_of_wall=[0, 0, 0, 0, 0]
index = 0

state_outer_inner=[0, 0, 0, 0]
index_state_outer_inner = 0

bool_outer_corner = 0
bool_inner_corner =0

last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]
wall_found =0

#Robot state machines
state_ = 0
state_dict_ = {
    0: 'random wandering',
    1: 'following wall',
    2: 'rotating'
}

def clbk_laser(msg):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall
    size = len(msg.ranges)
    min_index = size*(direction+1)/4
    max_index = size*(direction+3)/4
    
    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
            min_index = i
    angle_min = (min_index-size/2)*msg.angle_increment
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[size/2]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)

    # Determination of minimum distances in each region
    regions_ = {
        'bright':  min(min(msg.ranges[0:143]), inf),
        'right': min(min(msg.ranges[144:287]), inf),
        'fright':  min(min(msg.ranges[288:431]), inf),
        'front':  min(min(msg.ranges[432:575]), inf),
        'fleft':   min(min(msg.ranges[576:719]), inf),
        'left':   min(min(msg.ranges[720:863]), inf),
        'bleft':   min(min(msg.ranges[864:1007]), inf),
    }
    #rospy.loginfo(regions_)

    # Detection of Outer and Inner corner
    bool_outer_corner = is_outer_corner()
    bool_inner_corner = is_inner_corner()
    if bool_outer_corner == 0 and bool_inner_corner == 0:
        last_kinds_of_wall[index]=0
    
    # Indexing for last five pattern detection
    # This is latter used for low pass filtering of the patterns
    index = index + 1 #5 samples recorded to asses if we are at the corner or not
    if index == len(last_kinds_of_wall):
        index = 0
        
    take_action()

def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        #print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Wall found - Following Wall
            State 2 Pattern sequence reached - Rotating
    """
    global regions_, index, last_kinds_of_wall, index_state_outer_inner, state_outer_inner, loop_index, loop_index_outer_corner
    
    global wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner

    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # Patterns for rotating
    rotate_sequence_V1 = ['I', 'C', 'C', 'C']
    rotate_sequence_V2 = [0, 'C', 'C', 'C']
    rotate_sequence_W = ['I', 'C', 'I', 'C']

    if rotating == 1:
        state_description = 'case 2 - rotating'
        change_state(2)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fleft'] == inf and regions['left'] == inf and regions['bleft'] == inf:
        state_description = 'case 0 - random wandering'
        change_state(0)
    elif (loop_index == loop_index_outer_corner) and (rotate_sequence_V1 == state_outer_inner or rotate_sequence_V2 == state_outer_inner or rotate_sequence_W == state_outer_inner):
        state_description = 'case 2 - rotating'
        change_direction()
        state_outer_inner = [ 0, 0,  0, 'C']
        change_state(2)
    else:
        state_description = 'case 1 - following wall'
        change_state(1)

def random_wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> [0.1, 0.3]
                    msg.angular.z -> [-1, 1]
    """
    global direction, last_vel
    msg = Twist()
    msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
    msg.angular.z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
    if msg.angular.z == 1 or msg.angular.z == -1:
        msg.angular.z = 0
    last_vel[0] = msg.linear.x
    last_vel[1] = msg.angular.z
    return msg

def following_wall():
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    msg = Twist()
    if dist_front < wall_dist:
        msg.linear.x = 0
    elif dist_front < wall_dist*2:
        msg.linear.x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        msg.linear.x = 0.4*max_speed
    else:
        msg.linear.x = max_speed
    msg.angular.z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    return msg

def change_direction():
    """
    Toggle direction in which the robot will follow the wall
        1 for wall on the left side of the robot and -1 for the right side
    """
    global direction, last_change_direction, rotating
    print 'Change direction!'
    elapsed_time = time.time() - last_change_direction_time # Elapsed time since last change direction
    if elapsed_time >= 20:
        last_change_direction = time.time()
        direction = -direction # Wall in the other side now
        rotating = 1

def rotating():
    """
    Rotation movement of the robot. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction*2
    return msg


def is_outer_corner():
    """
    Assessment of outer corner in the wall. 
    If all the regions except for one of the back regions are infinite then we are in the presence of a possible corner.
    If all the elements in last_kinds_of_wall are 'C' and the last time a real corner was detected is superior or equal to 30 seconds:
        To state_outer_inner a 'C' is appended and 
        The time is restart.
    Returns:
            bool_outer_corner: 0 if it is not a outer corner; 1 if it is a outer corner
    """
    global regions_, last_kinds_of_wall, last_outer_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index, loop_index_outer_corner
    regions = regions_
    bool_outer_corner = 0
    if (regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] < inf  and regions['left'] == inf and regions['bleft'] == inf and regions['fleft'] == inf) or (regions['bleft'] < inf and regions['fleft'] == inf and regions['front'] == inf and regions['left'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fright'] == inf):
        bool_outer_corner = 1 # It is a corner
        last_kinds_of_wall[index]='C'
        elapsed_time = time.time() - last_outer_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('C') == len(last_kinds_of_wall) and elapsed_time >= 30:
            last_outer_corner_detection_time = time.time()
            loop_index_outer_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('C')
            print 'It is a outer corner'
    return bool_outer_corner

def is_inner_corner():
    """
    Assessment of inner corner in the wall. 
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and 
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """
    global regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    regions = regions_
    bool_inner_corner = 0
    if regions['fright'] < wall_dist and regions['front'] < wall_dist and regions['fleft'] < wall_dist:
        bool_inner_corner = 1
        last_kinds_of_wall[index]='I'
        elapsed_time = time.time() - last_inner_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('I') == len(last_kinds_of_wall) and elapsed_time >= 20:
            last_inner_corner_detection_time = time.time()
            loop_index_inner_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('I')
            print 'It is a inner corner'
    return bool_inner_corner

def main():
    global pub_, active_, hz, loop_index
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    
    print 'Code is running'
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        loop_index = loop_index + 1
        msg = Twist()

        # State Dispatcher
        if state_ == 0:
            msg = random_wandering()
        elif state_ == 1:
            msg = following_wall()
        elif state_ == 2:
            msg = rotating()
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
