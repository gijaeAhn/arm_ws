#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include"config.h"
#include"../Transform/UR5Kinematics.h"
#include"../ROSWRAPPER/roswrapper.h"

int main(int argc,char **argv){
    ros::init(argc,argv,"main_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ros::Publisher pub = node.advertise<sensor_msgs::JointState>("/joint_state",1000);

    sensor_msgs::JointState joint_msg;
    joint_msg.name.push_back("shoulder_pan_joint");
    joint_msg.name.push_back("shoulder_lift_joint");
    joint_msg.name.push_back("elbow_joint");
    joint_msg.name.push_back("wrist_1_joint");
    joint_msg.name.push_back("wrist_2_joint");
    joint_msg.name.push_back("wrist_3_joint");


    ROS_INFO("Starting task");


    Eigen::VectorXd joint_pick = UR5e_inverse(Eigen::MatrixXd SET_POINT_1, const double *qorg, double wristYaw);
    for ( int i = 0 ; i < sizeof(joint_msg.name); i++){
        joint_msg.postion.push_back(joint_pick[i]);}
    send_Jointstate(Eigen::VectorXd joint_pick);
    send_Grasp_Command();

    loop_rate.sleep();

    Eigen::VectorXd join_place = UR5e_inverse(Eigen::MatrixXd SET_POINT_2, const double *qorg, double wristYaw);
    for ( int i = 0 ; i < sizeof(joint_msg.name); i++){
        joint_msg.postion.push_back(joint_pick[i]);}
    send_Jointstate(Eigen::VectorXd joint_place);
    send_UnGrasp_Command();

    ROS_INFO("Task Finished");


    

}