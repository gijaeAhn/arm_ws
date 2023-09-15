#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include"config.h"
#include"../Transform/UR5Kinematics.h"
#include"../ROSWRAPPER/roswrapper.h"



int main(int argc ,char **argv){
    ros::init(argc,argv,"test_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(0.5);
    ros::Publisher pub = node.advertise<sensor_msgs::JointState>("/joint_command",1000);


    sensor_msgs::JointState joint_msg;
    joint_msg.name.push_back("shoulder_pan_joint");
    joint_msg.name.push_back("shoulder_lift_joint");
    joint_msg.name.push_back("elbow_joint");
    joint_msg.name.push_back("wrist_1_joint");
    joint_msg.name.push_back("wrist_2_joint");
    joint_msg.name.push_back("wrist_3_joint");

    joint_msg.position.push_back(1.0);
    joint_msg.position.push_back(2.0);
    joint_msg.position.push_back(3.0);
    joint_msg.position.push_back(4.0);
    joint_msg.position.push_back(5.0);
    joint_msg.position.push_back(6.0);

    pub.publish(joint_msg);
    loop_rate.sleep();

    pub.publish(joint_msg);

    ros::spin();

}