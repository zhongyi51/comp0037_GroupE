import heapq

CELL_POS = 1

class PriorityQueueByCost(object):
    def __init__(self):
        self._queue = []

    # enqueue cell by how costEvalFunc is defined, *args are for the eval func
    def enqueue(self, cell, cmpCell = None, costEvalFunc = None):
        if costEvalFunc:
            cost = costEvalFunc(cell, cmpCell)
            # print "enqueueing {} to queue {}, cost is {}\n".format(cell, self._queue, cost) # debug del
            heapq.heappush(self._queue, (cost, cell,))
        else:
            # self._queue.append((None, cell,)) # by default it is a FIFO queue. Not a good impl since the mem space for dummy None. Possibly raise an error is better?
            raise TypeError("I expect a cost evaluation is defined for using this .*ByCost class")

    def pop(self): #sorry for using the wrong name, possibly fix it to dequeue later
        # cell = self._queue.pop(0)[CELL_POS]
        cell = heapq.heappop(self._queue)[CELL_POS]
        # print "dequeueing {} to queue {}\n".format(cell, self._queue) # debug del
        return cell

    def __bool__(self):
        return bool(self._queue)
    __nonzero__ = __bool__ # for compatible with python 2.7
