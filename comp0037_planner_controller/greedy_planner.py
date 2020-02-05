# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
import heapq

CELL_POS = 1

class GreedyPlanner(CellBasedForwardSearch):

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self._PriorityQueueByGoal = self.PriorityQueueByGoal()
        self._curLenOfQueue = 0
        self._maxLenOfQueue = 0

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self._PriorityQueueByGoal.pushByED(cell, self._goal) # push based on the Euclidean dist from cell to goal

        # track for q size and max q size for task 1.1
        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue

    def isQueueEmpty(self):
        return not bool(self._PriorityQueueByGoal)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        self._curLenOfQueue -= 1 # for task 1.1
        return self._PriorityQueueByGoal.pop()

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    # for task 1.1
    def getMaxLenOfQueue(self):
        return self._maxLenOfQueue


    class PriorityQueueByGoal(object):
        def __init__(self):
            self._queue = [] # list of (Euclidean distance, Cell) tuple pairs. eg. [(1, cellA),(3, cellB),(4, cellC), (12, cellD)]

        def pushByED(self, cell, goal_coords):
            def findDistance(cell, toCoords):
                # x1, y1, x2, y2 = *fromCell.coords, *toCell.coords # seems python 2.7 can't parse this
                x1, y1 = cell.coords
                x2, y2 = toCoords
                return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

            dist = findDistance(cell, goal_coords)
            heapq.heappush(self._queue, (dist, cell,))

        def pop(self):
            cell = heapq.heappop(self._queue)[CELL_POS]
            return cell

        def __bool__(self):
            return bool(self._queue)
        __nonzero__ = __bool__ # for compatible with python 2.7
