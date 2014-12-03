#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <tabletop_perception_tools/PCLHelpers.h>
#include <tabletop_perception_tools/ExtractClusters.h>

#include <visualization_msgs/Marker.h>

using namespace tabletop_perception_tools;

ros::Publisher vis_publisher;
ros::NodeHandle* nodeHandle = 0x0;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
PointCloud::ConstPtr lastPointCloud;
bool pointCloudDirty = true;

void PointCloudCallback(PointCloud::ConstPtr pointCloud)
{
    lastPointCloud = pointCloud;
    pointCloudDirty = false;
}

bool ExtractClustersService(ExtractClusters::Request& request, ExtractClusters::Response& response)
{
    std::string topic = request.point_cloud_topic;
    pointCloudDirty = true;
    ros::Subscriber sub = nodeHandle->subscribe(topic, 1, &PointCloudCallback);
    ros::Duration timeout(5);
    ros::Time begin = ros::Time::now();
    ROS_INFO("Waiting for point cloud %s", topic.c_str());
    while (pointCloudDirty)
    {
        usleep(10000);
        ROS_INFO("Waiting...");
        if (ros::Time::now() - begin > timeout)
        {
            ROS_WARN("Timeout exceeded waiting for point cloud %s", topic.c_str());
            return false;
        }
    }

    ROS_INFO("Got point cloud. Extracting");
    PointCloud::Ptr pointCloud(new PointCloud(*lastPointCloud));

    if (request.segment_depth)
    {
        ROS_INFO("Segmenting depth");
        pointCloud = pcl_helpers::FilterBoundingBox<Point>(pointCloud, pcl_helpers::Vec3(-1000, -1000, request.min_depth), pcl_helpers::Vec3(1000, 1000, request.max_depth));
    }

    if (request.segment_planes)
    {
        ROS_INFO("Removing a plane");
        pointCloud = pcl_helpers::RemoveLargePlanes<Point>(pointCloud, request.num_planes, request.plane_distance);

    }

    ROS_INFO("Removing the clusters");
    std::vector<pcl_helpers::Cluster<Point> > clusters;
    pcl_helpers::ExtractClusters<Point>(pointCloud, &clusters, request.cluster_tolerance, request.min_cluster_size, request.max_cluster_size);

    ROS_INFO("Extracted %lu clusters", clusters.size());
    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl_helpers::Cluster<Point>& cluster = clusters.at(i);

        tabletop_perception_tools::Cluster cluster_msg;
        cluster_msg.header.stamp = ros::Time::now(); // TODO: get the right timestamp
        cluster_msg.header.frame_id = pointCloud->header.frame_id;
        cluster_msg.max.x = cluster.bounds.max.x();
        cluster_msg.max.y = cluster.bounds.max.y();
        cluster_msg.max.z = cluster.bounds.max.z();

        cluster_msg.min.x = cluster.bounds.min.x();
        cluster_msg.min.y = cluster.bounds.min.y();
        cluster_msg.min.z = cluster.bounds.min.z();

        cluster_msg.pose.position.x = cluster.bounds.transform.translation().x();
        cluster_msg.pose.position.y = cluster.bounds.transform.translation().y();
        cluster_msg.pose.position.z = cluster.bounds.transform.translation().z();

        pcl_helpers::Quaternion quat(cluster.bounds.transform.linear());

        cluster_msg.pose.orientation.x = quat.x();
        cluster_msg.pose.orientation.y = quat.y();
        cluster_msg.pose.orientation.z = quat.z();
        cluster_msg.pose.orientation.w = quat.w();

        visualization_msgs::Marker marker;
        marker.color.r = 1;
        marker.color.g = 0.8;
        marker.color.b = 0;
        marker.color.a = 0.5;
        marker.header.frame_id = cluster_msg.header.frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = cluster_msg.pose;
        marker.scale.x = (cluster_msg.max.x - cluster_msg.min.x);
        marker.scale.y = (cluster_msg.max.y - cluster_msg.min.y);
        marker.scale.z = (cluster_msg.max.z - cluster_msg.min.z);
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        vis_publisher.publish(marker);

        pcl::toROSMsg(*cluster.points, cluster_msg.points);
        response.clusters.push_back(cluster_msg);
    }

    ROS_INFO("Shutting down...");
    sub.shutdown();

    ROS_INFO("Done.");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tools_server");
    ros::NodeHandle node("~");
    nodeHandle = &node;
    ros::ServiceServer server = node.advertiseService(std::string("extract_clusters"), &ExtractClustersService);
    vis_publisher = node.advertise<visualization_msgs::Marker>(std::string("cluster_markers"), 1);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
