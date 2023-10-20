#include "Occupancy_Map_Server.h"


namespace occupancymapserver{

OccupancyMapServer(const ros::Nodehandle nh; const ros::Nodehandle pnh):
    OccupancyGrid(),nh_(nh),pnh_(pnh), PointCloudSub_(NULL), tfPointCloud_(NULL)
{
    pnh_.param("camera_1_id",Camera_link_id_,camera_link_id_);
    pnh_.param("filter/groundDistance",groundDistance_);
    pnh_.param("fliter/groundAngle",groundAngle_,groundAngle_);
    pnh_.param("filter/xDistance",xDistance_,xDistance_);
    pnh_.param("filterGroundPlane",filterGroundPlane_,filterGroundPlane_);
    pnh_.param("pointcloudMinZ,",pointcloudMinZ_,pointcloudMinZ_);
    pnh_.param("pointcloudMaxZ",pointcloudMaxZ_,pointcloudMaxZ_);
    pnh_.param("worldDimesionsX",temp_worldDimension[0],temp_worldDimension[0]);
    pnh_.param("worldDimesionsY",temp_worldDimension[1],temp_worldDimension[1]);
    pnh_.param("worldDimesionsX",temp_worldDimension[2],temp_worldDimension[2]);
    pnh_.param("originX",Origin_[0],Origin_[0]);
    pnh_.param("originY",Origin_[1],Origin_[1]);
    pnh_.param("originZ",Origin_[2],Origin_[2]);
    pnh_.param("threshold",Threshold_,Threshold_;)
    pnh_.param("resolution",Resolution_,Resolution_);

    if(filterGroundPlane_ && (pointcloudMinZ_ >0 || pointcloudMinZ_ <0))
    {
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<pointcloudMinZ_ <<", "<< pointcloudMaxZ_ << "], excluding the ground level z=0. "
              << "This will not work.");
    }

    for(int i = 0; i<3; i++) Grid_Dimensions_[i] = static_cast<int>(temp_worldDimension[i]*Resolution_);

    markerPub_ = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_marker_array", 1);
    PointCloudSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
    tfPointCloudSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*PointCloudSub_, tfListener, camera_link_id_, 5);
    tfPointCloudSub->registerCallback(boost::bind(&OccupancyMapServer::insertCloudCallback, this, boost::placeholders::_1));



}





}