#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)
current_vel = [0, 0]
gap_angle = [0]
range_history = [0, 0, 0, 0, 0, 0, 0, 0, 0]

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

   # calculate acceleration vector
   # average-based direct input implementation

#    y = np.sin(gap_angle)
#    x = np.cos(gap_angle)
#    print "{}  {}".format(x,y)
#    print "velCallback gap_angle: {}".format(gap_angle)
    global current_vel
    current_vel = [msg.x, msg.y]
    #if msg.x > aValue
    #x=0
    

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser ranges

    global current_vel
    global gap_angle
    global range_history
    msg_ranges = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    delta_vel = [0, 0]
    position = 0
    
    print "current_vel : {0:6.3f} {1:6.3f}".format(current_vel[0], current_vel[1])

    # discard out of range values & create vector for manipulation
    for n in range(0, 8):
    	if msg.ranges[n] < msg.range_min:
	    msg_ranges[n] = msg.range_min;
    	elif msg.ranges[n] > msg.range_max:
	    msg_ranges[n] = msg.range_max
	else:
	    msg_ranges[n] = msg.ranges[n]

    # find max values
    m = max(msg_ranges)
    pos_max_range = [i for i,j in enumerate(msg_ranges) if j==m ]
    print "pos_max_rangle: {}".format(pos_max_range)
    #pos_max_range = np.argmax(msg_ranges)

    # find angles associated with the max values
    if len(pos_max_range) == 1:
        new_gap_angle = msg.angle_min + (pos_max_range[0]*msg.angle_increment)
        print "new_gap_angle: {0:6.3f}".format(gap_angle[0])
        gap_angle.insert(0, new_gap_angle)

    else:
        print "multiple maximum ranges, using historical data"
        for i in range(0, len(pos_max_range)):
            position += pos_max_range[i]
        position = position / len(pos_max_range)
        new_gap_angle = msg.angle_min + (position*msg.angle_increment)
        #new_gap_angle = (new_gap_angle + gap_angle[0])/2
        gap_angle.insert(0, new_gap_angle)
    
    print "gap_angle: {0:6.3f}".format(gap_angle[0])
    
    if abs(gap_angle[0] - gap_angle[1]) < abs(msg.angle_increment):
        desired_y = (np.sin(gap_angle[0]))/2
        desired_x = 0.5
        print "desired_vel: {0:6.3f}  {1:6.3f}".format(desired_x,desired_y)
    else:
        desired_y = 0
        desired_x = 0.3
        print "no clear gap, using default vel: {0:6.3f}  {1:6.3f}".format(desired_x,desired_y)
    
    
    delta_vel[0] = desired_x - current_vel[0]
    delta_vel[1] = 2*(desired_y - current_vel[1])
    print "delta_vel : {0:6.3f} {1:6.3f}".format(delta_vel[0], delta_vel[1])
    pub_acc_cmd.publish(Vector3(delta_vel[0],delta_vel[1],0))
    
    if len(gap_angle) > 4 :
        for i in range(4, len(gap_angle)):
            del gap_angle[i]
    
    print "Laser Ranges: {0:6.3f} {1:6.3f} {2:6.3f} {3:6.3f} {4:6.3f} {5:6.3f} {6:6.3f} {7:6.3f} {8:6.3f}".format(msg_ranges[0], msg_ranges[1], msg_ranges[2], msg_ranges[3], msg_ranges[4], msg_ranges[5], msg_ranges[6], msg_ranges[7], msg_ranges[8] )
    #print "Laser Min Angle: {0} Laser Angle Increment: {1}".format(msg.angle_min, msg.angle_increment)
    #print "Laser Range Min: {0} Laser Range Max: {1}".format(msg.range_min, msg.range_max)
    print "Laser time between scans: {0}".format(msg.time_increment)
    range_history = msg_ranges + range_history

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
