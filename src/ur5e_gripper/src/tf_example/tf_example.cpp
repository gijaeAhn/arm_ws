#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
//#include"../Transform/UR5Kinematics.cpp"
#include"../Transform/UR5Kinematics.h"

#include<sensor_msgs/JointState.h>





void tfCallback(const sensor_msgs::JointState &msg){
    
    ros::Rate r(100);
    tf::TransformBroadcaster br;
    ROS_INFO("HELLO");
    br.sendTransform(
    tf::StampedTransform(
    tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.0,0.0,0.0)),
    ros::Time::now(),"base_link","world"));

    r.sleep();


}



int main(int argc, char **argv){

    ros::init(argc,argv,"tf_sub_pub");

    ros::NodeHandle node;


    ros::Subscriber sub = node.subscribe("/joint_states",1000, tfCallback);

    ros::spin();

    return 0;

}
