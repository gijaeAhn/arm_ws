#include "Occupancy_Map_Server.h"


namespace occupancymapserver{

OccupancyMapServer::OccupancyMapServer(const ros::NodeHandle nh, const ros::NodeHandle pnh,
                                       const double threshold, const double resolution)
:
    nh_(nh),pnh_(pnh), PointCloudSub_(NULL), tfPointCloudSub_(NULL)
{
    pnh_.param("camera_1_id",cameraLink_id,cameraLink_id);
    pnh_.param("filter/groundDistance",groundDistance_);
    pnh_.param("fliter/groundAngle",groundAngle_,groundAngle_);
    pnh_.param("filter/xDistance",xDistance_,xDistance_);
    pnh_.param("filterGroundPlane",filterGroundPlane_,filterGroundPlane_);
    pnh_.param("pointcloudMinZ,",pointcloudMinZ_,pointcloudMinZ_);
    pnh_.param("pointcloudMaxZ",pointcloudMaxZ_,pointcloudMaxZ_);
    pnh_.param("worldDimesionsX",bufworldDimension[0],bufworldDimension[0]);
    pnh_.param("worldDimesionsY",bufworldDimension[1],bufworldDimension[1]);
    pnh_.param("worldDimesionsX",bufworldDimension[2],bufworldDimension[2]);
    pnh_.param("originX",bufOrigin_[0],bufOrigin_[0]);
    pnh_.param("originY",bufOrigin_[1],bufOrigin_[1]);
    pnh_.param("originZ",bufOrigin_[2],bufOrigin_[2]);
    pnh_.param("resolution",bufResolution_,bufResolution_);
    pnh_.param("threshold",bufThreshold_,bufThreshold_);

    ocgrid_.getThreshold(bufThreshold_);
    ocgrid_.getResoltuion(bufResolution_);

    if(filterGroundPlane_ && (pointcloudMinZ_ >0 || pointcloudMinZ_ <0))
    {
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<pointcloudMinZ_ <<", "<< pointcloudMaxZ_ << "], excluding the ground level z=0. "
              << "This will not work.");
    }

    ocgrid_.getGridDimesion(bufworldDimension,bufResolution_);
    markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_marker_array", 1);
    PointCloudSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 5);
    tfPointCloudSub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*PointCloudSub_, tfListener, cameraLink_id, 5);
    tfPointCloudSub_->registerCallback(boost::bind(&OccupancyMapServer::insertCloudCallback, this, boost::placeholders::_1));



}

OccupancyMapServer::~OccupancyMapServer()
{
    if(PointCloudSub_){
        delete PointCloudSub_;
        PointCloudSub_ = nullptr;
    }
    if(tfPointCloudSub_){
        delete tfPointCloudSub_;
        tfPointCloudSub_ = nullptr;
    }
}

void OccupancyMapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time startTime = ros::Time::now();
    PCLPointCloud pc;

    pcl::fromROSMsg(*msg,pc);
    tf::StampedTransform sensorToWorldTf;

    try{
        tfListener.lookupTransform(worldFrame_id,msg->header.frame_id,msg->header.stamp,sensorToWorldTf);
    }catch(tf::TransformException& ex){
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback.");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf,sensorToWorld);

    pcl::PassThrough<PCLPoint> pass_x;
    pcl::PassThrough<PCLPoint> pass_y;
    pcl::PassThrough<PCLPoint> pass_z;

    pass_x.setFilterFieldName("x");
    pass_y.setFilterFieldName("y");
    pass_z.setFilterFieldName("z");
    
    pass_x.setFilterLimits(MinX,MaxX);
    pass_y.setFilterLimits(MinY,MaxY);
    pass_z.setFilterLimits(MinZ,MaxZ);
    
    pcl::transformPointCloud(pc,pc,sensorToWorld);

    pass_x.setInputCloud(pc.makeShared());
    pass_y.setInputCloud(pc.makeShared());
    pass_z.setInputCloud(pc.makeShared());
    
    pass_x.filter(pc);
    pass_y.filter(pc);
    pass_z.filter(pc);

    double total_elapsed = (ros::Time::now() - startTime).toSec();
    ROS_DEBUG("PointCloud insertion done in %f sec",total_elapsed);

    
}

void OccupancyMapServer::cloudToOcMap(const PCLPointCloud& pc)
{

    for(PCLPointCloud::const_iterator it = pc.begin(); it !=pc.end();++it ){
        pcl::PointXYZ point(it->x,it->y,it->z);
        ocgrid_.UpdateValue(point,true);
    }
}


}