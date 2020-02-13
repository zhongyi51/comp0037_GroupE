#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.dijkstra_planner import DijkstraPlanner
from comp0037_planner_controller.fifo_planner import FIFOPlanner
from comp0037_planner_controller.lifo_planner import LIFOPlanner
from comp0037_planner_controller.greedy_planner import GreedyPlanner
from comp0037_planner_controller.greedy_rewiring_planner import GreedyRewiringPlanner
from comp0037_planner_controller.astar_planner import AStarPlanner
from comp0037_planner_controller.astar_manhattan_planner import AStarPlanner_Manhattan
from comp0037_planner_controller.stupid_planner import RandomPlanner
from comp0037_planner_controller.astar_squared_euclidean_planner import AStarPlanner_Squared_Euclidean
from comp0037_planner_controller.astar_none_negative_planner import AStarPlanner_None_Negative
from comp0037_planner_controller.astar_octile_distance_planner import AStarPlanner_Octile_Distance



Planners = (FIFOPlanner, LIFOPlanner, GreedyPlanner, GreedyRewiringPlanner, DijkstraPlanner, AStarPlanner,RandomPlanner,AStarPlanner_Manhattan,AStarPlanner_Squared_Euclidean,AStarPlanner_None_Negative,AStarPlanner_Octile_Distance) # append planners here for testing

occupancyGrid = OccupancyGrid(21, 21, 0.5)
for y in xrange(0, 20): # set block cell positions here
    occupancyGrid.setCell(11, y, 1)

start = (2, 18)
goal = (20, 0)

occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0) # set cells size here for testing

for Planner in Planners:
    print Planner.__name__
    planner = Planner(Planner.__name__, occupancyGrid);
    if Planner == Planners[-1]:
        planner.setRunInteractively(True) # hold the drawing by the last case
    planner.setWindowHeightInPixels(400)
    goalReached = planner.search(start, goal)
    # print out infomation
    path = planner.extractPathToGoal()
    print 'Size of maximum queue: ', planner.getMaxLenOfQueue()
    print '\n'

# # for task 1.1, some may be not needed
# print 'The planner is: ', type(planner).__name__
# print 'Number of cells to reach goal: ', planner.getNumberOfCellsVisited()
# print 'Size of maximum queue: ', planner.getMaxLenOfQueue()
# print 'Total travel cost of the optimal path: ', planner.getTotalTravelCost()
# print 'Total angle turned for the optimal path: ', planner.getTotalAgle()
