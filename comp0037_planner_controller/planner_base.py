# -*- coding: utf-8 -*-

from occupancy_grid import OccupancyGrid
from search_grid import SearchGrid
from grid_drawer import SearchGridDrawer
import time
from collections import deque
from cell import *
from planned_path import PlannedPath
from math import *

class PlannerBase(object):

    # This class implements the basic graphical support for the planners
    
    def __init__(self, title, occupancyGrid):
        self.title = title
        self.occupancyGrid = occupancyGrid
        self.searchGrid = None
        
        print "Occupancy grid dimensions = " + str(occupancyGrid.getWidthInCells()) + "x" + str(occupancyGrid.getHeightInCells())
        
        # Graphics and debug output support
        self.showGraphics = True
        self.pauseTimeInSeconds = 0
        self.iterationsBetweenGraphicsUpdate = 10000
        self.iterationsSinceLastGraphicsUpdate = 0
        self.plannerDrawer = None
        self.windowHeightInPixels = 700
        self.runInteractively = False

    # Pause for key presses?
    def setRunInteractively(self, runInteractively):

        # Record the decision here. We can only configure the drawer if it's already been
        # created.
        self.runInteractively = runInteractively
        if self.plannerDrawer is not None:
            self.plannerDrawer.setRunInterctively(runInteractively)
        
    # Show graphics?
    def setShowGraphics(self, showGraphics):
        self.showGraphics = showGraphics

    # Getter to get the graphics
    def getPlannerDrawer(self):
        return self.plannerDrawer
        
    # Change the default window height in pixels
    def setWindowHeightInPixels(self, windowHeightInPixels):
        self.windowHeightInPixels = windowHeightInPixels
        
    # Set the pause time. When a window is updated, how long
    # should we pause for? Default is 0s.
    def setDrawingPauseTime(self, pauseTimeInSeconds):
        self.pauseTimeInSeconds = pauseTimeInSeconds
        
    # Set the number of iterations between drawning the window. The
    # default is 50
    def setIterationsBetweenDrawing(self, iterationsBetweenGraphicsUpdate):
        self.iterationsBetweenGraphicsUpdate = iterationsBetweenGraphicsUpdate

    # Reset the graphics; clear and close the window and then open a new window
    def resetGraphics(self):
        
        if (self.showGraphics == False):
            return
        
        # HACK: AT THE MOMENT I CAN'T GET THE WINDOW TO DRAW NEATLY
        if (self.plannerDrawer is not None):
            self.plannerDrawer.close()
            self.plannerDrawer = None
        
        # If we don't have the planner set up yet, create it      
        if (self.plannerDrawer is None):
            self.createPlannerDrawer()
            self.plannerDrawer.setRunInteractively(self.runInteractively)
            self.plannerDrawer.setStartAndGoal(self.start, self.goal)
            self.plannerDrawer.open()
        else:
            self.plannerDrawer.reset()
            
        # Now force an initial draw
        self.drawCurrentState(forceUpdate=True)
        
    # Draw the output and sleep for the pause time.
    def drawCurrentState(self, forceUpdate=False):

        # If graphics is disabled, return
        if self.showGraphics is False:
            return
            
        # Check if we need to do an update
        self.iterationsSinceLastGraphicsUpdate = self.iterationsSinceLastGraphicsUpdate + 1
        if forceUpdate is False:     
            if (self.iterationsSinceLastGraphicsUpdate < self.iterationsBetweenGraphicsUpdate):
                return

        # Reset the draw counter
        self.iterationsSinceLastGraphicsUpdate = 0

        self.plannerDrawer.update()
        time.sleep(self.pauseTimeInSeconds)

    # Create the drawer which shows the planner's progress
    def createPlannerDrawer(self):
        self.plannerDrawer = SearchGridDrawer(self.title, self.searchGrid, self.windowHeightInPixels)

    # Set the pause time
    def setPauseTime(self, pauseTimeInSeconds):
        self.pauseTimeInSeconds = pauseTimeInSeconds
