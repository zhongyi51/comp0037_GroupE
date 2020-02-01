# -*- coding: utf-8 -*-

from general_forward_search_algorithm import GeneralForwardSearchAlgorithm
from collections import deque
from cell import *

class CellBasedForwardSearch(GeneralForwardSearchAlgorithm):

    # This implements the logic for doing a forward search for an environment which consists of a set of goals.
    
    
    def __init__(self, title, occupancyGrid):
        GeneralForwardSearchAlgorithm.__init__(self, title, occupancyGrid)

    # Check if the goal has been reached. We do this by checking position rather than object instance values,
    # because the latter failed sometimes.
    def hasGoalBeenReached(self, cell):
        goalReached = (cell.coords[0] == self.goal.coords[0]) & (cell.coords[1] == self.goal.coords[1])
        return goalReached

    # This method gets the list of cells which potentially could be
    # visited next. Each candidate position has to be tested
    # separately. It encodes the "clock around" logic.
    def getNextSetOfCellsToBeVisited(self, cell):

        # This stores the set of valid actions / cells
        cells = list()

        # Go through all the neighbours and add the cells if they
        # don't fall outside the grid and they aren't the cell we
        # started with. The order has been manually written down to
        # create a spiral.
        self.pushBackCandidateCellIfValid(cell, cells, 0, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, -1)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, 1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, 0, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 1)
        self.pushBackCandidateCellIfValid(cell, cells, -1, 0)
        self.pushBackCandidateCellIfValid(cell, cells, -1, -1)

        return cells

    # This helper method checks if the robot, at cell.coords, can move
    # to cell.coords+(offsetX, offsetY). Reasons why it can't do this
    # include falling off the edge of the map or running into an
    # obstacle.
    def pushBackCandidateCellIfValid(self, cell, cells, offsetX, offsetY):
        newX = cell.coords[0] + offsetX
        newY = cell.coords[1] + offsetY
        extent = self.occupancyGrid.getExtentInCells()
        if ((newX >= 0) & (newX < extent[0]) \
            & (newY >= 0) & (newY < extent[1])):
            newCoords = (newX, newY)
            newCell = self.searchGrid.getCellFromCoords(newCoords)
            if (newCell.label != CellLabel.OBSTRUCTED):
                cells.append(newCell)

    # This method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(self, cell):
        return (cell.label == CellLabel.OBSTRUCTED) | (cell.label == CellLabel.DEAD) \
            | (cell.label == CellLabel.ALIVE)
