# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
import heapq

CELL_POS = 1

class GreedyPlanner(CellBasedForwardSearch):

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self._priorityQueue = self.PriorityQueue()

        self._curLenOfQueue = 0
        self._maxLenOfQueue = 0

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self._priorityQueue.push(cell)

        # track for q size and max q size for task 1.1
        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue

    def isQueueEmpty(self):
        return not bool(self._priorityQueue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        self._curLenOfQueue -= 1 # for task 1.1
        return self._priorityQueue.pop()

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    # for task 1.1
    def getMaxLenOfQueue(self):
        return self._maxLenOfQueue


    class PriorityQueue(object):
        def __init__(self):
            self._queue = [] # list of (Euclidean distance, Cell) tuple pairs. eg. [(1, cellA),(3, cellB),(4, cellC), (12, cellD)]
            self._lastPop = None # I expect this to be the source to be searched for other cells regarding the gfs_algo

        def push(self, cell):
            def findDistance(fromCell, toCell):
                # x1, y1, x2, y2 = *fromCell.coords, *toCell.coords # seems python 2.7 can't parse this
                x1, y1 = fromCell.coords[0], fromCell.coords[1]
                x2, y2 = toCell.coords[0], toCell.coords[1]
                return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

            if self._lastPop == None: heapq.heappush(self._queue, (0, cell)); return # The first push case. This instr might be optimized later

            dist = findDistance(self._lastPop, cell)
            heapq.heappush(self._queue, (dist, cell))

        def pop(self):
            cell = heapq.heappop(self._queue)[CELL_POS]
            self._lastPop = cell # ie. The next source cell
            return cell

        def __bool__(self):
            return bool(self._queue)
        __nonzero__ = __bool__ # for compatible with python 2.7