import heapq

COST_POS = 0
CELL_POS = 1

# I have seen an email saying that use the python built-in class priority queue. I have not tried that one myself, maybe some one can try that if it is of interest.
class PriorityQueueByCost(object):
    def __init__(self):
        self._queue = []

    # enqueue cell by how costEvalFunc is defined
    def enqueue(self, cell, cmpCell = None, costEvalFunc = None):
        if costEvalFunc:
            cost = costEvalFunc(cell, cmpCell)
            heapq.heappush(self._queue, [cost, cell,])

        else:
            raise TypeError("I expect a cost evaluation is defined for using this .*ByCost class")

    def pop(self):
        cell = heapq.heappop(self._queue)[CELL_POS]
        return cell

    def updateCostOfCell(self, cost, cell):  # for updating the queue after the resolving the resolveDuplicate. assume the cell has to be in the queue
        for i in range(len(self._queue)):
            cell_local = self._queue[i][CELL_POS]
            if cell_local == cell:
                self._queue[i][COST_POS] = cost

        heapq.heapify(self._queue) # update the position
        return

    def __bool__(self):
        return bool(self._queue)
    __nonzero__ = __bool__ # for compatible with python 2.7
