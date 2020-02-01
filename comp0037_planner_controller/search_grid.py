# -*- coding: utf-8 -*-

from cell import Cell
class SearchGrid(object):

    # This class stores the state of a search grid to illustrate forward search

    def __init__(self, width, height):
        self.width = width
        self.height = height

    # Construct the class using an occupancy grid object
    @classmethod
    def fromOccupancyGrid(cls, occupancyGrid):

        self = cls(occupancyGrid.getWidthInCells(), occupancyGrid.getHeightInCells())

        self.occupancyGrid = occupancyGrid
        
        # Populate the search grid from the occupancy grid
        self.updateFromOccupancyGrid()
        
        return self

    def getExtent(self):
        return self.occupancyGrid.getExtent()

    def getExtentInCells(self):
        return self.occupancyGrid.getExtentInCells()

    def getResolution(self):
        return self.occupancyGrid.getResolution()

    # Reset the state of the search grid to the value of the occupancy grid
    def updateFromOccupancyGrid(self):        
        self.grid = [[Cell((x, y), self.occupancyGrid.getCell(x,y)) for y in range(self.height)] \
                     for x in range(self.width)]
        # print([[self.occupancyGrid.getCell(x,y) for y in range(self.height)] \
                    #  for x in range(self.width)])
     

    def getCellFromCoords(self, coords):
        return self.grid[coords[0]][coords[1]]
