# -*- coding: utf-8 -*-

# The planned path the robot will take. This consists of a set of waypoints.
from collections import deque

class PlannedPath(object):

    # Construct a new planner object and set defaults.
    def __init__(self):

        # Does the path actually reach the goal or not?
        self.goalReached = False
        
        # The list of waypoints, from start to finish, which make up the path.
        # The type of data stored here depends on the 
        self.waypoints = deque()

        # Performance information - number of waypoints, and the
        # travel cost of the path.
        self.numberOfWaypoints = 0
        self.travelCost = 0
