#! /usr/bin/env python

# See run_fifo_standalone.py for documentation. The only difference is that
# a LIFO planner is created instead of a FIFO planner.

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.greedy_planner import GreedyPlanner

occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 19):
    occupancyGrid.setCell(11, y, 1)

start = (3, 18)
goal = (20, 0)

planner = GreedyPlanner('Greedy (priority queue)', occupancyGrid);
planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()

# for task 1.1
print('The planner is: ', type(planner).__name__)
print('Number of cells to reach goal: ', planner.getNumberOfCellsVisited())
print('Size of maximum queue: ', planner.getMaxLenOfQueue())
print('Total travel cost of the optimal path: ', planner.getTotalTravelCost())
print('Total angle turned for the optimal path: ', planner.getTotalAgle())
