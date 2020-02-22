# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from priority_queue_by_cost import PriorityQueueByCost
from greedy_rewiring_planner import GreedyRewiringPlanner
from math import *

class AStarPlanner_Manhattan(GreedyRewiringPlanner): # or think about inherite from dijkstra if you feel like that is more natural

    def __init__(self, title, occupancyGrid):
        GreedyRewiringPlanner.__init__(self, title, occupancyGrid)

    # @OVERRIDE: override GreedyPlanner.pushCellOntoQueue
    def pushCellOntoQueue(self, cell):
        self._queue.enqueue(cell, self.goal, self._costEvalFunc) # push based on the L = C + G

        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue
			
    # @OVERRIDE: override GeneralForwardSearchAlgorithm.computeLStageAdditiveCost
    def computeLStageAdditiveCost(self, parentCell, cell):
	# If the parent is empty, this is the start of the path and the
	# cost is 0.
	if (parentCell is None):
		return 0

	# Travel cost is Cartesian distance
	dX = cell.coords[0] - parentCell.coords[0]
	dY = cell.coords[1] - parentCell.coords[1]
	# Terrain cost
	#  Run this in matlab to visualise ro check the image
	# However, basically it builds up extremely quickly
	# x=[1:0.01:2];
	# c=min(1+(.2./((1.7-x).^2)).^2,1000);
	cost=min(1+(0.2/((1.75-cell.terrainCost)**2))**2, 1000)
	L = (sqrt(dX*dX)+sqrt(dY*dY))*cost# Multiplied by the terrain cost of the cell
	#L = sqrt(dX * dX + dY * dY)*cost# Multiplied by the terrain cost of the cell

	return L

    def _costEvalFunc(self, fromCell, toCell):
        """
        l is Estimated path length given that we are at cell i
        c is Cost from the start to cell i
        g is Estimated cost from cell i to the goal (see lecture note 4 for more info)
        """

        c = fromCell.pathCost
        g = self.computeLStageAdditiveCost(fromCell, toCell)
        l = c + g

        return l
