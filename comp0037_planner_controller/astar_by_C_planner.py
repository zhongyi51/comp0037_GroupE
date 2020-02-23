# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from priority_queue_by_cost import PriorityQueueByCost
from greedy_rewiring_planner import GreedyRewiringPlanner

class AStarByCPlanner(GreedyRewiringPlanner): # or think about inherite from dijkstra if you feel like that is more natural

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
        g = 0 # g is a constant, try some others to see how it goes as well
        l = c + g

        return l
