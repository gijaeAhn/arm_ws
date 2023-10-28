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


typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

namespace occupancymapserver{

class OccupancyMapServer :
{
    private :

    occupancygrid::OccupancyGrid ocgrid_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher markerPub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* PointCloudSub_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
    tf::TransformListener tfListener;

    double bufworldDimension[3];
    std::string cameraLink_id, worldFrame_id;
    double groundDistance_, groundAngle_, xDistance_,pointcloudMinZ_,pointcloudMaxZ_;
   
    bool filterGroundPlane_;
    
    double bufThreshold_;
    double bufResolution_;

    double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;

    double bufOrigin_[3];



    public:

    OccupancyMapServer(const ros::NodeHandle nh, const ros::NodeHandle pnh,
                       const double threshold, const double resolution);

                       
    ~OccupancyMapServer();

    
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void cloudToOcMap(const PCLPointCloud& pc);


};
}