#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.fifo_planner import FIFOPlanner

# Create the occupancy grid
occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

# Start and goal cells
start = (3, 18)
goal = (20, 0)

# Create the planner on the original map
planner = FIFOPlanner('Depth First Search Original Occupancy Grid', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

# Now try it on our Minkowski sum map with radius 0.5 (1 cell)
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0.5)
planner = FIFOPlanner('Depth First Search Robot Radius 0.5', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

# Now try it on our Minkowski sum map with radius 0. Should be the same as the original
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0)
planner = FIFOPlanner('Depth First Search Robot Radius 0 (Same As Original Map)', occupancyGrid);
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()


# Now try it on our Minkowski sum map with a radius of 2 (4 cells). In this case, the
# robot can't get there from here.
occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(2)
planner = FIFOPlanner('Depth First Search Robot Radius 4', occupancyGrid);
planner.setRunInteractively(True)
planner.setWindowHeightInPixels(400)
planner.search(start, goal)
path = planner.extractPathToGoal()

