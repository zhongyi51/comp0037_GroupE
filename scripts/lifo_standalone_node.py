#! /usr/bin/env python

# Import the needed types.
from comp0037_planner_controller.lifo_planner import LIFOPlanner
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
import map_getter
import rospy

#Initialise node
rospy.init_node('lifo_standalone', anonymous=True)

#Mapgetter  helps load maps off the map server
mapGetter = map_getter.MapGetter()
occupancyGrid=mapGetter.getMapFromServer()

start = rospy.get_param("start_pose")
goal = rospy.get_param("goal_pose")

# Create the planner. The first field is the title which will appear in the
# graphics window, the second the occupancy grid used.
planner = LIFOPlanner('Deapth First Search', occupancyGrid)

# This causes the planner to slow down and pause for things like key entries
planner.setRunInteractively(True)

# This specifies the height of the window drawn showing the occupancy grid. Everything
# should scale automatically to properly preserve the aspect ratio
planner.setWindowHeightInPixels(400)

# Search and see if a path can be found. Returns True if a path from the start to the
# goal was found and False otherwise
goalReached = planner.search(start, goal)

# Extract the path. This is based on the last search carried out.
path = planner.extractPathToGoal()