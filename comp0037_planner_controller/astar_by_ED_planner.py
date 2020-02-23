# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from priority_queue_by_cost import PriorityQueueByCost
from greedy_rewiring_planner import GreedyRewiringPlanner

class AStarByEDPlanner(GreedyRewiringPlanner): # or think about inherite from dijkstra if you feel like that is more natural

    def __init__(self, title, occupancyGrid,scale=1):
        GreedyRewiringPlanner.__init__(self, title, occupancyGrid)
	self.scale=scale

    # @OVERRIDE: override GreedyPlanner.pushCellOntoQueue
    def pushCellOntoQueue(self, cell):
        self._queue.enqueue(cell, self.goal, self._costEvalFunc) # push based on the L = C + G

        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue

    def _costEvalFunc(self, fromCell, toCell):
        """
        l is Estimated path length given that we are at cell i
        c is Cost from the start to cell i
        g is Estimated cost from cell i to the goal (see lecture note 4 for more info)
        """

        c = fromCell.pathCost
        g = self.computeLStageAdditiveCost(fromCell, toCell)
        l = c + g*self.scale

        return l
