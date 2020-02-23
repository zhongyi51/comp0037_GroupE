# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch

class LIFOPlanner(CellBasedForwardSearch):

    # This implements a simple LIFO (last in first out or depth first) search algorithm

    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()
        self._curLenOfQueue = 0
        self._maxLenOfQueue = 0

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.lifoQueue.append(cell)

        # track for q size and max q size for task 1.1
        self._curLenOfQueue += 1
        if self._curLenOfQueue > self._maxLenOfQueue:
            self._maxLenOfQueue = self._curLenOfQueue

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.lifoQueue.pop()
        self._curLenOfQueue -= 1 # for task 1.1
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    # for task 1.1
    def getMaxLenOfQueue(self):
        return self._maxLenOfQueue
