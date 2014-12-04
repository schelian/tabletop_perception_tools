#!/usr/bin/env python

from tabletop_perception_tools.srv import *
import rospy
import sys

def extract_clusters(service_name="tools_server/extract_clusters", 
                     cloud_topic="/head/kinect2/depth/pointcloud",
                     segment_planes=True, 
                     num_planes=1, 
                     plane_distance=0.01, 
                     segment_depth=True, 
                     min_depth=0.1, 
                     max_depth=1.0, 
                     cluster_tolerance=0.01, 
                     min_cluster_size=100, 
                     max_cluster_size=10000):
    rospy.wait_for_service(service_name);
    try:
        service = rospy.ServiceProxy(service_name, ExtractClusters);
        response = service(cloud_topic, segment_planes, num_planes, plane_distance, segment_depth, min_depth, max_depth, cluster_tolerance, min_cluster_size, max_cluster_size);
        return response.clusters;
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print extract_clusters();
