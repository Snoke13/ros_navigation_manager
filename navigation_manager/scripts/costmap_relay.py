#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid


class Cr:

    _globalCostMap=OccupancyGrid()
    _localCostMap=OccupancyGrid()

    def __init__(self):
        rospy.init_node('costmap_relay')

        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.globalCostMap_callback)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.localCostMap_callback)

        globalCostMap_pub = rospy.Publisher("/move_base/global_costmap/costmap", OccupancyGrid, latch=True, queue_size=1)
        localCostMap_pub = rospy.Publisher("/move_base/local_costmap/costmap", OccupancyGrid, latch=True, queue_size=1)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            globalCostMap_pub.publish(self._globalCostMap)
            #localCostMap_pub.publish(self._localCostMap) currently not published
            rate.sleep() 




    def globalCostMap_callback(self,msg):
        self._globalCostMap=msg

    def localCostMap_callback(self,msg):
         self._localCostMap=msg

if __name__ == '__main__':
    cr = Cr()
