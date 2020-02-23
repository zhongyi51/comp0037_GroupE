# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from priority_queue_by_cost import PriorityQueueByCost

class GreedyPlanner(CellBasedForwardSearch):

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self._queue = PriorityQueueByCost()
        self._curLenOfQueue = 0
        self._maxLenOfQueue = 0

    def pushCellOntoQueue(self, cell):
        self._queue.enqueue(cell, self.goal, self._costEvalFunc) # push based on the Euclidean dist from cell to goal

        # track for q size and max q size for task 1.1
        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue

    def isQueueEmpty(self):
        return not bool(self._queue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        self._curLenOfQueue -= 1 # for task 1.1
        return self._queue.pop()

    def resolveDuplicate(self, cell, parentCell):
        pass

    # for task 1.1
    def getMaxLenOfQueue(self):
        return self._maxLenOfQueue

    def _costEvalFunc(self, fromCell, toCell):
        return self.computeLStageAdditiveCost(fromCell, toCell)
