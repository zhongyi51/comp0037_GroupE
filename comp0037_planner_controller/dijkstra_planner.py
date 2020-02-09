# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from fifo_planner import FIFOPlanner

class DijkstraPlanner(FIFOPlanner):

    def __init__(self, title, occupancyGrid):
        FIFOPlanner.__init__(self, title, occupancyGrid)

    # @OVERRIDE
    def resolveDuplicate(self, cell, parentCell):
        newCost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)
        if cell.pathCost > newCost:
            cell.parent = parentCell
            cell.pathCost = newCost
