#! /usr/bin/env python
import map_getter
import rospy
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.dijkstra_planner import DijkstraPlanner
from comp0037_planner_controller.fifo_planner import FIFOPlanner
from comp0037_planner_controller.lifo_planner import LIFOPlanner
from comp0037_planner_controller.greedy_planner import GreedyPlanner
from comp0037_planner_controller.greedy_rewiring_planner import GreedyRewiringPlanner

from comp0037_planner_controller.astar_by_C_planner import AStarByCPlanner
from comp0037_planner_controller.astar_by_ED_planner import AStarByEDPlanner
from comp0037_planner_controller.astar_by_OD_planner import AStarByODPlanner
from comp0037_planner_controller.astar_by_MD_planner import AStarByMDPlanner
from comp0037_planner_controller.astar_by_ED_planner import AStarByEDPlanner



Planners = (FIFOPlanner, LIFOPlanner, GreedyPlanner, GreedyRewiringPlanner, DijkstraPlanner, AStarByCPlanner,
            AStarByEDPlanner, AStarByODPlanner, AStarByMDPlanner) # append planners here for testing

# Initialise node
rospy.init_node('fifo_standalone', anonymous=True)

# Mapgetter  helps load maps off the map server
mapGetter = map_getter.MapGetter()
occupancyGrid = mapGetter.getMapFromServer()

start = rospy.get_param("start_pose")
goal = rospy.get_param("goal_pose")

occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0) # set cells size here for testing

for Planner in Planners:
    print Planner.__name__
    planner = Planner(Planner.__name__, occupancyGrid);
    if Planner == Planners[-1]:
        planner.setRunInteractively(True) # hold the drawing by the last case
    planner.setWindowHeightInPixels(400)
    goalReached = planner.search(start, goal)

    # print out infomation
    path = planner.extractPathToGoal()
    print '\n'
