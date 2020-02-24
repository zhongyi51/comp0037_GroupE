#!/usr/bin/env python
import rospy, time, math, os
from controller_record import ControllerRecord
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
from subprocess import Popen, call
from datetime import datetime

# for saving the travelling summary infomation for analysis
timestr = time.strftime("%Y%m%d-%H%M%S")
call("[ ! -d {0} ] &&  mkdir {0}".format('/home/ros_user/datasave_assignment1/'), shell=True)
DRIVE_TO_GOAL_STATUS_SUMMARY_SAVEFILE = '/home/ros_user/datasave_assignment1/AstarBySED_Factory_Summary_{}.txt'.format(timestr)
del timestr

NORMVAL = 4.55 # normalize the travel cost, which is diff from the distance

# This is the base class of the controller which moves the robot to its goal
class ControllerBase(ControllerRecord):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        ControllerRecord.__init__(self)
        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid

        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation

        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

        self.updateRecord(pose)
        self.printRecord() # report, important !!!

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        self.plannerDrawer = plannerDrawer

        goal = str(path.waypoints[-1].coords) # myMod: assume last waypoints is the goal. DEBUG: can consider to del
        rospy.loginfo('Driving path to goal cell %s with '%goal + str(len(path.waypoints)) + ' waypoint(s)')
        self.reset() # myMod: clean up the pre-setup status errors

        # Drive to each waypoint in turn
        pre_waypoint=[-1,-1]
        pre_delta=[0,0]
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            delta=[waypoint[0]-pre_waypoint[0],waypoint[1]-pre_waypoint[1]]
            # rospy.loginfo("Current waypoint is:"+str(waypoint)+"  Previous waypoint is:"+str(pre_waypoint))

            # work out farthest waypoint to skip
            if delta==pre_delta and waypointNumber != len(path.waypoints) - 1: #the 2nd and handles the skipping last waypoint on a straight line bug
                pre_waypoint=waypoint[:]
                # rospy.loginfo("Delta unchanged: (%f, %f)", delta[0], delta[1])
            else:
                rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
                self.driveToWaypoint(waypoint)
                pre_waypoint=waypoint[:]
                pre_delta=delta[:]

            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

        self.redirect_current_status_to_file(DRIVE_TO_GOAL_STATUS_SUMMARY_SAVEFILE, path, last_waypoint = str(waypoint)) #myMod: save data for summanry of this travelling

    def redirect_current_status_to_file(self, file, path, last_waypoint = None): # assume a path is always given, sorry for the nasty assumption
        with open(file, "a+") as fp:
            s = \
            "Datatime: {}, ".format(str(datetime.now())) + ("Reached Goal %s "%last_waypoint if last_waypoint else '')\
            + '\n' +\
            "Contoller Infomation: \n\
            Controller Distance: {}\n\
            Controller Total Angle Turned in Degree: {}\n\
            Controller Total Runtime: {}s\n\
            Controller Average Speed: {}\n".format(self._distanceTravelled, self._angleTurned * 180/math.pi, self._timePassed, str(self._distanceTravelled/self._timePassed))\
            + '\n' +\
            "Planned Path Infomation:\n\
            Path travel cost = {}\n\
            Path travel cost(normalized)= {}\n\
            Path Angle Turned in Degree= {}\n\
            Path Cardinality = {}\n\
            Path Number of Cells Visited = {}\n\
            Path Maximum Queue Size = {}\n".format(str(path.travelCost), str(path.travelCost/NORMVAL),str(path.totalAngleTurned), str(path.numberOfWaypoints), str(path.numberOfCellsVisited), str(path.maxLenOfQueue))\
            + '\n\n'

            fp.write(s)
        print s
