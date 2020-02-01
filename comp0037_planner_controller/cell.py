# -*- coding: utf-8 -*-

# This class stores information about each cell - its coordinates in the grid,
# its label, its parents and the path cost.

from enum import Enum


class CellLabel(Enum):
    OBSTRUCTED = -3
    START = -2
    GOAL = -1
    UNVISITED = 0
    DEAD = 1
    ALIVE = 2


class Cell(object):

    def __init__(self, coords, isOccupied):

        # Set coordinates
        self.coords = coords
        # obstruction ranges from 0. to 1. so Adding 1 so it could be multiplied.
        self.terrainCost = min(1+isOccupied,1.7)# Terrain cost should not go over 1.7 in order to leave space for  dynamic obstacles (its fine, don't worry about it, this works)
        # Label the cell
        if (isOccupied == 1):
            self.label = CellLabel.OBSTRUCTED
        else:
            self.label = CellLabel.UNVISITED

        # Initially the cell has no parents.
        self.parent = None

        # The initial path cost is infinite. For algorithms that need
        # it, this is the necessary initial condition.
        self.pathCost = float("inf")
