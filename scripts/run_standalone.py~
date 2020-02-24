#! /usr/bin/env python

from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.dijkstra_planner import DijkstraPlanner
from comp0037_planner_controller.fifo_planner import FIFOPlanner
from comp0037_planner_controller.lifo_planner import LIFOPlanner
from comp0037_planner_controller.greedy_planner import GreedyPlanner
from comp0037_planner_controller.greedy_rewiring_planner import GreedyRewiringPlanner

from comp0037_planner_controller.astar_by_C_planner import AStarByCPlanner
from comp0037_planner_controller.astar_by_ED_planner import AStarByEDPlanner
from comp0037_planner_controller.astar_by_OD_planner import AStarByODPlanner
from comp0037_planner_controller.astar_by_MD_planner import AStarByMDPlanner
from comp0037_planner_controller.astar_by_ED_planner import AStarByEDPlanner



Planners = (FIFOPlanner, LIFOPlanner, GreedyPlanner, GreedyRewiringPlanner, DijkstraPlanner, AStarByCPlanner,
            AStarByEDPlanner, AStarByODPlanner, AStarByMDPlanner) # append planners here for testing

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
    print '\n'
