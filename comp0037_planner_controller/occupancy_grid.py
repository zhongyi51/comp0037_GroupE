# -*- coding: utf-8 -*-

import math
import rospy
import copy


def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

# This class stores the occupancy grid. This is a "chessboard-like"
# representation of the environment. The environment is represented by
# a set of square cells. Each cell encodes whether that bit of the
# environment is free, or whether it is blocked. A "0" says that a
# cell is free and so the robot can travel over it. A "1" means that
# it is blocked and the robot cannot travel over it.


class OccupancyGrid(object):

    # Construct a new occupancy grid with a given width and
    # height. The resolution says the length of the side of each cell
    # in metres. By default, all the cells are set to "0" which means
    # that there are no obstacles.
    def __init__(self, widthInCells, heightInCells, resolution):
        self.widthInCells = widthInCells
        self.heightInCells = heightInCells
        self.extentInCells = (self.widthInCells, self.heightInCells)

        self.resolution = resolution

        self.width = widthInCells * self.resolution
        self.height = heightInCells * self.resolution
        self.extent = (self.width, self.height)
        self.grid = [[0 for y in range(self.heightInCells)]
                     for x in range(self.widthInCells)]
        self.scale = 1

        self.originalGrid = None

    # Set the scale for the map.
    def setScale(self, scale):
        self.scale = scale

    # Set the data from the array received from the map server. The
    # memory layout is different, so we have to flip it here. The map
    # server also scales 100 to mean free and 0 to mean blocked. We
    # use 0 for free and 1 for blocked.
    def setFromDataArrayFromMapServer(self, data):
        print("Length of Map: {}".format(len(data)))
        print("Map Resolution: {}".format(str(self.resolution)))
        print("Map Width: {}".format(self.width))
        print("Map Height: {}".format(self.height))
        print("Map Width (cells): {}".format(self.widthInCells))
        print("Map Height (cells): {}".format(self.heightInCells))

        self.grid = [[0 for y in range(self.heightInCells)]
                     for x in range(self.widthInCells)]

        # Get the complete map
        for x in range(self.widthInCells):
            for y in range(self.heightInCells):
                data_val = data[len(
                    data)-(self.widthInCells-x-1)-self.widthInCells*y-1]
                if data_val < 1:
                    self.grid[x][self.heightInCells-y-1] = 0
                else:
                    self.grid[x][self.heightInCells -
                                 y-1] = float(data_val)/100.0

        self.scaleMap()

    # Pre process the map so that we expand all the obstacles by a
    # circle of radius robotRadius metres. This is a way to account
    # for the geometry. Technically, this is known as taking the
    # Minkowski sum. Practically, this is a really bad way to write
    # this! It also currently uses the crude aproximation that the
    # robot shape is a square
    def expandObstaclesToAccountForCircularRobotOfRadius(self, robotRadius):

        # Compute the size we need to grow the obstacle
        s = int(math.ceil(robotRadius / self.resolution))
        print 's=' + str(s)

        # Always scale from the original grid. If it doesn't exist, make a copy
        if self.originalGrid is None:
            self.originalGrid = [[self.grid[x][y] for y in range(
                self.heightInCells)] for x in range(self.widthInCells)]

        # Allocate the new occupancy grid, which will contain the new obstacles
        newGrid = copy.deepcopy(self.originalGrid)  # Preserve cost values
        # newGrid = [[0 for y in range(self.heightInCells)] for y in range(self.widthInCells)]

        # Iterate through all the cells in the first grid. If they are a 1, set all
        # cells within radius robotRadius to occupied as well. Note the magic +1 in the
        # range. This is needed because range(a,b) actually gives [a, a+1, ..., b-1]. See
        # https://www.pythoncentral.io/pythons-range-function-explained/
        for x in range(self.widthInCells):
            for y in range(self.heightInCells):
                if self.originalGrid[x][y] > .99:
                    for gridX in range(clamp(x-s, 0, self.widthInCells), clamp(x+s+1, 0, self.widthInCells)):
                        for gridY in range(clamp(y-s, 0, self.heightInCells), clamp(y+s+1, 0, self.heightInCells)):
                            newGrid[gridX][gridY] = 1.

        self.grid = newGrid

    def scaleMap(self):

        planning_map = [[0 for y in range(
            self.heightInCells/self.scale)] for x in range(self.widthInCells/self.scale)]
        print("Planning map size\nWidth: {}\nHeight: {}".format(
            len(planning_map), len(planning_map[0])))

        plan_x_range = range(self.widthInCells)[0::self.scale]
        plan_y_range = range(self.heightInCells)[0::self.scale]

        # Checks every cell in the range and if any is occupied the scaled is occupied
        for x in range(len(plan_x_range)):
            for y in range(len(plan_y_range)):

                # if it is the last planning cell just use that single cell
                if x == len(plan_x_range) - 1:
                    x_range = [plan_x_range[x]]
                else:
                    x_range = range(plan_x_range[x], plan_x_range[x+1] - 1)

                if y == len(plan_y_range) - 1:
                    y_range = [plan_y_range[y]]
                else:
                    y_range = range(plan_y_range[y], plan_y_range[y + 1] - 1)

                # Start with the planning cell being unoccupied and search for an occupied cell as soon as you find one
                # stop searching.
                occupied = 0
                total_cost = 0
                for x_check in x_range:

                    if occupied == 1:
                        break

                    for y_check in y_range:
                        value = self.grid[x_check][y_check]
                        total_cost += value
                        if value > .99 or occupied == 1:
                            occupied = 1
                            break
                if occupied == 1:
                    total_cost = 1.
                else:        
                    total_cost = total_cost/(len(x_range)*len(y_range))

                planning_map[x][y] = total_cost

        self.grid = planning_map

        if self.originalGrid is None:
            self.originalGrid = copy.deepcopy(self.grid)

        self.widthInCells = self.widthInCells / self.scale
        self.heightInCells = self.heightInCells / self.scale
        self.extentInCells = (self.widthInCells, self.heightInCells)

        self.resolution = self.resolution * self.scale

        self.width = self.widthInCells * self.resolution
        self.height = self.heightInCells * self.resolution
        self.extent = (self.width, self.height)

    # The width of the occupancy map in cells
    def getWidth(self):
        return self.width

    # The height of the occupancy map in cells
    def getHeight(self):
        return self.height

    # The resolution of each cell (the length of its side in metres)
    def getResolution(self):
        return self.resolution

    # Take a position in world coordinates (i.e., m) and turn it into
    # cell coordinates. Clamp the value so that it always falls within
    # the grid. The conversion uses integer rounding.
    def getCellCoordinatesFromWorldCoordinates(self, worldCoords):

        cellCoords = (clamp(int(worldCoords[0] / self.resolution), 0, self.widthInCells - 1),
                      clamp(int(worldCoords[1] / self.resolution), 0, self.heightInCells - 1))

        return cellCoords

    # Convert a position in cell coordinates to world coordinates. The
    # conversion uses the centre of a cell, hence the mysterious 0.5
    # addition. No clamping is currently done.
    def getWorldCoordinatesFromCellCoordinates(self, cellCoords):

        worldCoords = ((cellCoords[0] + 0.5) * self.resolution,
                       (cellCoords[1] + 0.5) * self.resolution)

        return worldCoords

    # The width of the occupancy map in cells
    def getWidthInCells(self):
        return self.widthInCells

    # The height of the occupancy map in cells
    def getHeightInCells(self):
        return self.heightInCells

    def getExtentInCells(self):
        return self.extentInCells

    # The resolution of each cell (the length of its side in metres)
    def getResolution(self):
        return self.resolution

    # Get the status of a cell.
    def getCell(self, x, y):
        return self.grid[x][y]

    # Set the status of a cell.
    def setCell(self, x, y, c):
        self.grid[x][y] = c

    # Take a position in workspace coordinates (i.e., m) and turn it into
    # cell coordinates. Clamp the value so that it always falls within
    # the grid. The conversion uses integer rounding.
    def getCellCoordinatesFromWorldCoordinates(self, worldCoords):

        cellCoords = (clamp(int(worldCoords[0] / self.resolution), 0, self.extentInCells[0]),
                      clamp(int(worldCoords[1] / self.resolution), 0, self.extentInCells[1]))

        return cellCoords

    # Convert a position in cell coordinates to workspace coordinates. The
    # conversion uses the centre of a cell, hence the mysterious 0.5
    # addition. No clamping is currently done.
    def getWorldCoordinatesFromCellCoordinates(self, cellCoords):

        worldCoords = ((cellCoords[0] + 0.5) * self.resolution,
                       (cellCoords[1] + 0.5) * self.resolution)

        return worldCoords
