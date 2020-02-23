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

# for saving the travelling summary infomation for analysis
timestr = time.strftime("%Y%m%d-%H%M%S")
call("[ ! -d {0} ] &&  mkdir {0}".format('/home/ros_user/datasave_assignment1/'), shell=True)
DRIVE_TO_GOAL_STATUS_SUMMARY_SAVEFILE = '/home/ros_user/datasave_assignment1/AstarByMD_FactoryTerrain_Summary_{}.txt'.format(timestr)
del timestr

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
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords) # convert cell coords to a real world coords
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        self.redirect_current_status_to_file(DRIVE_TO_GOAL_STATUS_SUMMARY_SAVEFILE, last_waypoint = str(waypoint))

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)
