#!/usr/bin/env python

import sys
import rospy
from nav_msgs.srv import GetMap
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_planner_controller.search_grid import SearchGrid


# This class pings the map server and gets the map.

class MapGetter(object):

    def __init__(self):
        rospy.loginfo('Waiting for static_map to become available.')
        # print "Hello1"
        rospy.wait_for_service('static_map') 
        # print "Hello2"
        self.mapServer = rospy.ServiceProxy('static_map', GetMap)
        # print "Hello3"

    def getMapFromServer(self):
        # print "starting"
        resp = self.mapServer()
        # rospy.logerr(resp)
        # print "got from server"
        occupancyGrid = OccupancyGrid(resp.map.info.width, resp.map.info.height, resp.map.info.resolution)
        occupancyGrid.setScale(rospy.get_param('plan_scale', 5))
        # print "make grid"
        occupancyGrid.setFromDataArrayFromMapServer(resp.map.data)  
        # occupancyGrid.expandObstaclesToAccountForCircularRobotOfRadius(0.2)
        return occupancyGrid
        
if __name__ == '__main__':
    mapGetter = MapGetter()
    mapGetter.getMapFromServer()

  
