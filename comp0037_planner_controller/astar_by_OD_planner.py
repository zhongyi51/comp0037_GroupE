# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from priority_queue_by_cost import PriorityQueueByCost
from greedy_rewiring_planner import GreedyRewiringPlanner
from cell import *

class AStarByODPlanner(GreedyRewiringPlanner): 

    def __init__(self, title, occupancyGrid):
        GreedyRewiringPlanner.__init__(self, title, occupancyGrid)

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

        connerCell = Cell((fromCell.coords[0], toCell.coords[1],), 0) # some more info can be fount in lecture note 4
        dx, dy = self.computeLStageAdditiveCost(fromCell, connerCell), self.computeLStageAdditiveCost(connerCell, toCell)
        g = max(dx, dy) + (2**0.5 - 1) * min(dx, dy)

        l = c + g

        return l
