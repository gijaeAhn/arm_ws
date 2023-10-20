#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "Occupancy_Grid.h"

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace occupancymapserver{

class OccupancyMapServer : public occupancygrid::OccupancyGrid
{
    private :

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher markerPub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* PointCloudSub_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
    tf::TransformListner tfListener;

    double temp_worldDimension[3];
    std::string camera_link_id_;
    double groundDistance_, groundAngle_, xDistance,pointcloudMinZ_,pointcloudMaxZ_;
   
    bool filterGroundPlane_;

    public:

    OccupancyMapServer(const ros::Nodehandle nh, const ros::Nodehandle pnh;);

    void InsertPC(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void PublishOcMap(const ros::Time& rostime = ros::Time::now());



}
}