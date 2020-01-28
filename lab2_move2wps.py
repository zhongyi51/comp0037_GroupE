#!/usr/bin/env python

import rospy
from geometry_msgs.msg  import Twist, Pose
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from PyKDL import Rotation
#my dev
import sys

class stdr_controller():

    def __init__(self, waypoints = None):
        rospy.init_node('stdr_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_pose_callback)

        self.current_pose = Odometry()
        self.pose = self.current_pose.pose.pose
        self.distance_tolerance = 1 # Tolerence
	orientation = self.pose.orientation
        self.theta = atan2(orientation.z, orientation.w)

        print(waypoints) #del, for debug later
        self.waypoints = waypoints #list of wps
        # self.pose = Pose()
        self.rate = rospy.Rate(10)

    def current_pose_callback(self, data): # The update upon receiving info from subscriptiong
        self.current_pose = data
        self.pose = self.current_pose.pose.pose
        orientation = self.pose.orientation
        self.theta = atan2(orientation.z, orientation.w)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant= 0.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):

        return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
	print('Steer A: %f |  Cur A: %f' %(self.steering_angle(goal_pose) * 180/pi, self.theta * 180/pi)) #degrees
        return constant * (self.steering_angle(goal_pose) - self.theta)

    def move2goal(self, waypoint):
        goal_pose = Pose()

	# read the next waypoint's position info
	goal_pose.position.x = waypoint[0] #x value
        goal_pose.position.y = waypoint[1] #y value
        distance_tolerance = self.distance_tolerance

	# manual inputs for testing
	'''
	goal_pose.position.x = input('Set X: ')
        goal_pose.position.y = input('Set Y: ')
        distance_tolerance = self.distance_tolerance
	'''

        vel_msg = Twist()
        # objectie 1: move to the location
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
	    self.report_info()
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

	    print('Publish speed Linear: {}, Angular: {}\n'.format(vel_msg.linear.x, vel_msg.angular.z))
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # objectie 2: correct the angel (ie. theta)
        #TODO: define a function to inspect and correct the robot's angle after reaching a waypoints

	print('Arrive Target Waypoint. The last log is:')
	self.report_info()
	print('\n')

    def report_info(self):
        pose = self.current_pose.pose.pose
        position = pose.position

	theta = self.theta
	info = 'Current position, x: {}, y:{}, theta:{}'.format(position.x, position.y, theta)
	#print(info)
        rospy.loginfo(info)

    def run(self):
        rospy.sleep(1.0)

        while self.waypoints:
            next_aim = self.waypoints.pop(0)  # queue
            self.move2goal(next_aim)

        rospy.spin() #press crltC to quit


def parse_waypoints_fromCLI():
    '''return objective waypoint list of coordinate tuples, or None if no argument is given'''
    if not len(sys.argv) > 1: #no argument
        print('No given waypoints, manual controll is on.')
        return None

    farg = sys.argv[1] #assume only one input waypoint file
    dir, fname = '/'.join(farg.split('/')[:-1]), farg.split('/')[-1]
    dir = (dir + '/') if dir else './' #support both abosulute or relative dir

    try:
        print('Opening File: {}{}'.format(dir, fname))
        fopen = open('{}{}'.format(dir, fname))
        fread = fopen.read()

    except IOError as e:
        print('{}\nThe file does not exist in the current dir, please double check!'.format(e))
        sys.exit(1)

    return [tuple(map(int, s.split())) for s in fread.split('\n') ] #return format such as [(15, 2, 0), (15, 2, 90)]

if __name__ == '__main__':
    try:
        #parse waypoints
        wps = parse_waypoints_fromCLI()

        x = stdr_controller(wps)
        x.run()

    except rospy.ROSInterruptException: pass
