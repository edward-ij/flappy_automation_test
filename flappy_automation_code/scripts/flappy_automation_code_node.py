#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
range_limit = 0.5
current_vel = [0, 0]
gap_angle = [0]
range_limit_warning = [0, 0, 0, 0, 0, 0, 0, 0, 0]
timeout = 0

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)
    

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3

   # save velocity for use in laserScanCallback
    global current_vel
    current_vel = [msg.x, msg.y]
    

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser ranges

    global current_vel
    global gap_angle
    global range_limit
    global range_limit_warning
    global timeout

    msg_ranges = [range_limit, range_limit, range_limit, range_limit, range_limit, range_limit, range_limit, range_limit, range_limit]
    delta_vel = [0, 0]
    position = 0.0
    
    # discard out of range values & create vector for manipulation
    for n in range(0, 9):
    	if msg.ranges[n] < msg.range_min:
	    msg_ranges[n] = msg.range_min
    	elif msg.ranges[n] > msg.range_max:
	    msg_ranges[n] = msg.range_min
	else:
	    msg_ranges[n] = msg.ranges[n]

    # PATHFINDING ------------------

    # find max values
    m = max(msg_ranges)
    pos_max_range = [i for i,j in enumerate(msg_ranges) if j==m ]
    print "pos_max_rangle: {}".format(pos_max_range)
    #pos_max_range = np.argmax(msg_ranges)

    # find angles associated with the max values
    # if there is only one angle which is larger than the rest
    if len(pos_max_range) == 1:
        new_gap_angle = msg.angle_min + (pos_max_range[0]*msg.angle_increment)
        print "new_gap_angle: {0:6.3f}".format(gap_angle[0])
        gap_angle.insert(0, new_gap_angle)
        
        if abs(gap_angle[0] - gap_angle[1]) < abs(msg.angle_increment/2) and abs(gap_angle[0] - gap_angle[2]) < abs(msg.angle_increment):
            desired_y = (np.sin(gap_angle[0]))
            desired_x = 0.5
        else:
            if gap_angle[0] < 0:
                desired_y = -0.5
            else:
                desired_y = 0.5
            desired_x = 0.1
            print "no constant gap, using default vel: {0:6.3f}  {1:6.3f}".format(desired_x,desired_y)
    # if there are multiple max angles approximate the path and exaggerate the acceleration
    else:
        print "multiple maximum ranges, using average and adding disturbance"
        for i in range(0, len(pos_max_range)):
            position += pos_max_range[i]
        position = position / len(pos_max_range)
        #print "approx_gap_position: {0:6.3f}".format(position)
        
        new_gap_angle = msg.angle_min + (position*msg.angle_increment)
        #new_gap_angle = (new_gap_angle + gap_angle[0])/2
        print "new_gap_angle: {0:6.3f}".format(gap_angle[0])
        gap_angle.insert(0, new_gap_angle)
        
        desired_y = (np.sin(gap_angle[0]))        
        
        if gap_angle[0] < 0:
            desired_y = desired_y - 0.3
        else:
            desired_y = desired_y + 0.3
        desired_x = 0.3
    
    # OBJECT AVOIDANCE ------------------------    
    
    # check for and log the location of any close objects
    # set values to 'remember' the object even if the scan misses it (as the scanner is at the front only)
    for i in range(0, 9) :
        if msg_ranges[i] < range_limit :
            if i == 0 or i == 8 :
                range_limit_warning[i] = 18
            elif i == 1 or i == 7 :
                range_limit_warning[i] = 15
            else :
                range_limit_warning[i] = 10
    
    # set desired velocity based on object locations
    if all(v == 0 for v in range_limit_warning):
        timeout = 0
        print "no close objects timeout reset"
    elif range_limit_warning[3] == 0 and range_limit_warning[4] == 0 and range_limit_warning[5] == 0 :
        timeout = 0
        print "timeout reset"
        if range_limit_warning[2] == 0 and range_limit_warning[6] == 0 :
            print "big gap straight ahead"
            desired_x = 0.4
            desired_y = 0
        elif range_limit_warning[2] > 0 and range_limit_warning[6] > 0 :
            print "small gap straight ahead, death may be iminent"
            if range_limit_warning[7] == 0 and range_limit_warning[8] > 0 :
                print "avoid via space above"
                desired_x = 0
                desired_y = 0.4
            if range_limit_warning[0] == 0 and range_limit_warning[1] > 0 :
                print "avoid via space below"
                desired_x = 0
                desired_y = -0.4
            else :
                print "take the risk"
                desired_x = 0.2
                desired_y = 0
        elif range_limit_warning[6] == 0 :
            if range_limit_warning[7] == 0 :
                print "big gap slightly above"
                desired_x = 0.2
                desired_y = 0.1
            else :
                print "small gap slightly above"
                desired_x = 0.1
                desired_y = 0.1
        elif range_limit_warning[2] == 0 :
            if range_limit_warning[1] == 0 :
                print "big gap slightly below"
                desired_x = 0.2
                desired_y = -0.1
            else :
                print "small gap slightly below"
                desired_x = 0.1
                desired_y = -0.1
    elif range_limit_warning[8] == 0 and range_limit_warning[7] == 0 :
        timeout = 0
        print "timeout reset"
        if range_limit_warning[6] == 0 :
            if range_limit_warning[5] == 0 :
                print "big gap above"
                desired_x = 0.1
                desired_y = 0.4
            else :
                print "small gap above"
                desired_x = 0.1
                desired_y = 0.5
        else :
            print "avoid via space above"
            desired_x = 0
            desired_y = 0.5
    elif range_limit_warning[0] == 0 and range_limit_warning[1] == 0 :
        timeout = 0
        print "timeout reset"
        if range_limit_warning[2] == 0 :
            if range_limit_warning[3] == 0 :
                print "big gap above"
                desired_x = 0.1
                desired_y = -0.4
            else :
                print "small gap above"
                desired_x = 0.1
                desired_y = -0.5
        else :
            print "avoid via space above"
            desired_x = 0
            desired_y = -0.5
    # wait for history to expire incase of errors then take extreme action
    else:
        if timeout > 20 :
            print "no real gaps, jump to light speed"
            print "range_limit_warning = {}".format(range_limit_warning)
            desired_x = 3
            desired_y = 30
        else :
            timeout += 1
            print "timing out"
            print "range_limit_warning = {}".format(range_limit_warning)
            desired_x = 0
            desired_y = 0

    # PRINTING AND PUBLISHING COMMANDS ------------------------    

            
    print "desired_vel: {0:6.3f}  {1:6.3f}".format(desired_x,desired_y)
    print "current_vel : {0:6.3f} {1:6.3f}".format(current_vel[0], current_vel[1])

    delta_vel[0] = (desired_x - current_vel[0])
    delta_vel[1] = 1.7*(desired_y - current_vel[1])
    print "delta_vel : {0:6.3f} {1:6.3f}".format(delta_vel[0], delta_vel[1])
    pub_acc_cmd.publish(Vector3(delta_vel[0],delta_vel[1],0))
    
    # print info messages
    #print "Laser Ranges: {0:6.3f} {1:6.3f} {2:6.3f} {3:6.3f} {4:6.3f} {5:6.3f} {6:6.3f} {7:6.3f} {8:6.3f}".format(msg_ranges[0], msg_ranges[1], msg_ranges[2], msg_ranges[3], msg_ranges[4], msg_ranges[5], msg_ranges[6], msg_ranges[7], msg_ranges[8] )
    #print "Laser Ranges: {0:6.3f} {1:6.3f} {2:6.3f} {3:6.3f} {4:6.3f} {5:6.3f} {6:6.3f} {7:6.3f} {8:6.3f}".format(msg.ranges[0], msg.ranges[1], msg.ranges[2], msg.ranges[3], msg.ranges[4], msg.ranges[5], msg.ranges[6], msg.ranges[7], msg.ranges[8] )
    #print "Laser Min Angle: {0} Laser Angle Increment: {1}".format(msg.angle_min, msg.angle_increment)
    #print "Laser Range Min: {0} Laser Range Max: {1}".format(msg.range_min, msg.range_max)
    #print "Laser time between scans: {0}".format(msg.time_increment)
    #print "range_limit_warning = {}".format(range_limit_warning)
    print ""
    
    # CLEAN UP ITERATION ------------------------    
    
    # decrease the range warnings over time
    for i in range (0, 9) :
        if range_limit_warning[i] > 0 :
            range_limit_warning[i] -= 1
        elif range_limit_warning[i] < 0 :
            range_limit_warning[i] = 0
        if range_limit_warning[i] > 12 :
            range_limit_warning[i] = 12
    
    # ensure that gap_angle doesn't get excessively large
    if len(gap_angle) > 5 :
        for i in range(5, len(gap_angle)):
            del gap_angle[i]
    
if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
