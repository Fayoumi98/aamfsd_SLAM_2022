#!/usr/bin/env python



import numpy as np
import math
import rospy
from sensor_msgs import msg
import tf
from aam_common_msgs.msg import Cone
from aam_common_msgs.msg import ConeDetections
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan







class RRT_Path_Plan_node:
  def __init__(self, namespace='rrt'):
    rospy.init_node("rrt_path_planner", anonymous = True)

    rospy.Subscriber("/lidar_cone_detection_cones",ConeDetections,self.cones_pipeline)



  def cones_pipeline(self,cones):
    
    cones_x = []
    cones_y  = []

    for cone in cones.cone_detections:
      cones_x.append(cone.position.x) 
      cones_y.append(cone.position.y) 

    print(cones_x,cones_y)








if __name__ == '__main__':
  try:
      my_path = RRT_Path_Plan_node()
  except rospy.ROSInterruptException:
      pass
  rospy.spin()
  
