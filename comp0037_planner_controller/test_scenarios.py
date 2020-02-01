# -*- coding: utf-8 -*-

# Get a set of scenarios

from occupancy_grid import OccupancyGrid

def emptyGridScenario():
    
    occupancyGrid = OccupancyGrid(21, 21, 1)
    start = (5, 10)
    goal = (15, 10)

    return start, goal, occupancyGrid

def pythagorasScenario():
    
    start, goal, occupancyGrid = emptyGridScenario()
    start = (1,1)
    goal = (17, 13)
    return start, goal, occupancyGrid

def horizontalWallScenario():
    
    # Create the occupancy grid
    occupancyGrid = OccupancyGrid(21, 21, 1)

    for x in xrange(2, 20):
        occupancyGrid.setCell(x, 11, 1)
    
    start = (2, 2)
    goal = (20, 20)
    
    return start, goal, occupancyGrid


def verticalWallScenarioEasy():
    
    # Create the occupancy grid
    occupancyGrid = OccupancyGrid(21, 21, 1)
    
    for y in xrange(1, 19):
        occupancyGrid.setCell(11, y, 1)
    
    start = (2, 18)
    goal = (20, 0)
    
    return start, goal, occupancyGrid

def verticalWallScenarioHard():
    start, goal, occupancyGrid = verticalWallScenarioEasy()
    occupancyGrid.setCell(11,0,1)
    return start, goal, occupancyGrid

