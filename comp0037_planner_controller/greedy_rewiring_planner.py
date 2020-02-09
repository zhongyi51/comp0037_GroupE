# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from greedy_planner import GreedyPlanner

class GreedyRewiringPlanner(GreedyPlanner):

    def __init__(self, title, occupancyGrid):
        GreedyPlanner.__init__(self, title, occupancyGrid)

    # @OVERRIDE: simply adding this one from dijkstra to make it 'cleverer'
    def resolveDuplicate(self, cell, parentCell):
        newCost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)
        if cell.pathCost > newCost:
            cell.parent = parentCell
            cell.pathCost = newCost
