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
from comp0037_planner_controller.astar_by_SED_planner import AStarBySEDPlanner



# Planners = (FIFOPlanner, LIFOPlanner, GreedyPlanner, GreedyRewiringPlanner, DijkstraPlanner)
Planners = (FIFOPlanner, LIFOPlanner)

for mapindex in range(1,4):
	occupancyGrid = OccupancyGrid(21, 21, 0.5)
	if mapindex==0:#empty map
		start = (2, 18)
		goal = (20, 0)

	elif mapindex==1:#One Line Map
		for y in xrange(0, 20): # set block cell positions here
		    occupancyGrid.setCell(11, y, 1)

		start = (0, 0)
		goal = (20, 0)

	elif mapindex==2:
		for y in xrange(1, 20): # set block cell positions here
		    occupancyGrid.setCell(2, y, 1)
		for y in xrange(0, 18): # set block cell positions here
		    occupancyGrid.setCell(5, y, 1)
		for y in xrange(1, 20): # set block cell positions here
		    occupancyGrid.setCell(8, y, 1)
		for y in xrange(0, 18): # set block cell positions here
		    occupancyGrid.setCell(11, y, 1)
		for y in xrange(1, 20): # set block cell positions here
		    occupancyGrid.setCell(14, y, 1)
		for y in xrange(0, 20): # set block cell positions here
		    occupancyGrid.setCell(17, y, 1)
		for y in xrange(2, 17): # set block cell positions here
		    occupancyGrid.setCell(y, 19, 1)

		occupancyGrid.setCell(17,10,0)

		start = (0, 0)
		goal = (20, 0)

	elif mapindex==3:
		for y in xrange(1, 21): # set block cell positions here
		    occupancyGrid.setCell(2, y, 1)
		for y in xrange(1, 18): # set block cell positions here
		    occupancyGrid.setCell(5, y, 1)
		for y in xrange(0, 21): # set block cell positions here
		    occupancyGrid.setCell(8, y, 1)
		for y in xrange(1, 13): # set block cell positions here
		    occupancyGrid.setCell(11, y, 1)
		for y in xrange(1, 21): # set block cell positions here
		    occupancyGrid.setCell(14, y, 1)
		for y in xrange(0, 21): # set block cell positions here
		    occupancyGrid.setCell(17, y, 1)
		for y in xrange(4, 10): # set block cell positions here
		    occupancyGrid.setCell(y, 17, 1)
		for y in xrange(0, 14): # set block cell positions here
		    occupancyGrid.setCell(19, y, 1)

		occupancyGrid.setCell(17,10,0)
		occupancyGrid.setCell(2,17,0)
		occupancyGrid.setCell(8,18,0)
		occupancyGrid.setCell(8,6,0)
		occupancyGrid.setCell(14,18,0)
		occupancyGrid.setCell(17,19,0)
		occupancyGrid.setCell(18,15,1)

		start = (0, 0)
		goal = (20, 0)

	occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0) # set cells size here for testing

	for Planner in Planners:
	    print Planner.__name__
	    planner = Planner(Planner.__name__, occupancyGrid);
	    # if Planner == Planners[-1]:
	    #     planner.setRunInteractively(True) # hold the drawing by the last case
	    planner.setWindowHeightInPixels(400)
	    goalReached = planner.search(start, goal)

	    # print out infomation
	    path = planner.extractPathToGoal()
	    print '\n'
