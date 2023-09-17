#include "UR5Kinematics.h"
using namespace std;
double mod_angle(double theta){
    if(theta>PI)  return theta-2*PI;
    else if(theta<-PI) return theta+2*PI;
    return theta;
}

Transform UR5e_forward_kinematics(double *q){
    Transform t;
    t = t                                   //BaseFrame
        .rotateZ(q[0])                      
        .translateZ(shoulderOffsetZ)
        .rotateX(PI/2)                      //ShoulderFrame
        .rotateZ(q[1])
        .translateX(-lowerArmLength)        //ForearmFrame
        .rotateZ(q[2])
        .translateX(-upperArmLength)        //Wrist1Frame
        .rotateZ(q[3])
        .translateZ(wrist1)                 
        .rotateX(PI/2)                      //Wrist2Frame
        .rotateZ(q[4])
        .translateZ(wrist2)
        .rotateX(-PI/2)
        .rotateZ(q[5])
        .translateZ(handLength)             //Wrist3Frame
        .translateZ(toolOffsetZ);           //ToolFrame
    return t;
}

//Reference Frame of Kinematics is World Frame which its x axis pointing Wrist Frame of the Home configuration

std::vector<double> UR5e_inverse_kinematics(Transform trArm,const double *qOrg, bool wrist_2_Flipped){
    Transform trArm_sub = trArm;
    printTransform(trArm);
    std::vector<double> Joint_Position(6);
    double shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint;
    double handOffsetZ = handLength + toolOffsetZ;                              //TCP Position
    trArm = trArm.translateZ(-toolOffsetZ);                                     //Tool0 Position

    

    Transform t1;
    Transform t_temp;
    t1 = 
    t1
        .translateZ(-shoulderOffsetZ)
        *trArm
        .translateZ(-toolOffsetZ);

   
    double E_p[3]; t1.apply0(E_p);
    double E_dis = sqrt(E_p[1]*+E_p[1]+E_p[0]*E_p[0]);

    t1 = t1.translateZ(-handLength);
    double Wrist_p[3]; t1.apply0(Wrist_p);                                      //Wrist Position relative to shoulder_pan_joint    Wrist Position = wrist_2_link frame
    double shoulder_wrist_2D = sqrt(Wrist_p[0]*Wrist_p[0] + Wrist_p[1]*Wrist_p[1]);


    double yaw_offset = asin(wrist1/shoulder_wrist_2D);
    double shoulder_pan_joint_xy = atan2(Wrist_p[1],Wrist_p[0]) - yaw_offset;
    shoulder_pan_joint = atan2(Wrist_p[1],Wrist_p[0]) - yaw_offset- PI;
    t_temp = t_temp.rotateZ(-shoulder_pan_joint)*t1;
    shoulder_pan_joint = mod_angle(shoulder_pan_joint);

    double E_angle;
    if(E_p[1] >0) E_angle = mod_angle(atan2(E_p[1],E_p[0])) ;
    else E_angle = mod_angle(atan2(E_p[1],E_p[0])+PI);
    if (E_p[1]>0) E_angle -= PI;

    double cwr = (E_dis*sin(E_angle-shoulder_pan_joint)-wrist1)/handLength;
    if(cwr > 1.0)  cwr =  1.0;
    if(cwr < -1.0) cwr = -1.0;
    wrist_2_joint = abs(acos(cwr));                                             //This wrist_2_joint angle is abs value
    if(t_temp(0,2) > 0) wrist_2_joint = -wrist_2_joint; 
                                                                                //Not Conclusive



    if(fabs(wrist_2_joint)<0.01){                                               // It means that sin(wrist_2_joint) is almost 0
        printf("Pitches are Parallel \n");
        shoulder_lift_joint = qOrg[1];

        Transform t2;
        t2 = 
        t2  
            .translateZ(lowerArmLength)                                         // Elbow to Second Wrist Frame
            .rotateZ(-shoulder_lift_joint)
            .rotateX(-PI/2)
            .rotateZ(-shoulder_pan_joint)
            .translateZ(-shoulderOffsetZ)
            *trArm
            .translateZ(-handOffsetZ);

        double elbowWrist2_p[3]; t2.apply0(elbowWrist2_p);                      //Elbow to wrist 2 Frame
        double elbowWrist2Dis = sqrt(elbowWrist2_p[0]*elbowWrist2_p[0] + elbowWrist2_p[2]*elbowWrist2_p[2]);
        double elbowPitchOffset = acos(
            (elbowWrist2Dis*elbowWrist2Dis+upperArmLength*upperArmLength-wrist2*wrist2)/(2*elbowWrist2Dis*upperArmLength));
        elbow_joint = atan2(elbowWrist2_p[0]/elbowWrist2Dis,elbowWrist2_p[2]/elbowWrist2Dis)-elbowPitchOffset;

        Transform t6;
        t6 = 
        t6
            .translateX(upperArmLength)
            .rotateZ(-elbow_joint)
            .translateX(lowerArmLength)
            .rotateZ(-shoulder_lift_joint)
            .rotateX(-PI/2)
            .rotateZ(-shoulder_pan_joint)
            .translateZ(-shoulderOffsetZ)
            *trArm
            .translateZ(-handOffsetZ);
        
        double wrist1_2_p[3]; t6.apply0(wrist1_2_p);
        wrist_1_joint = atan2(-wrist1_2_p[1]/wrist2, -wrist1_2_p[0]/wrist2);

        Transform t7;
        t7 = t7.rotateZ(-wrist_1_joint)*t6;
        wrist_3_joint = atan2(t7(0,0),t7(0,1));
        
        Joint_Position.push_back(shoulder_pan_joint);
        Joint_Position.push_back(shoulder_lift_joint);
        Joint_Position.push_back(elbow_joint);
        Joint_Position.push_back(wrist_1_joint);
        Joint_Position.push_back(wrist_2_joint);
        Joint_Position.push_back(wrist_3_joint);
        
        std::cout<<shoulder_pan_joint <<std::endl;
        std::cout<<shoulder_lift_joint <<std::endl;
        std::cout<<elbow_joint<<std::endl;
        std::cout<<wrist_1_joint <<std::endl;
        std::cout<<wrist_2_joint <<std::endl;
        std::cout<<wrist_3_joint <<std::endl;

        return Joint_Position;
    }


    Transform t3;

    t3 =t3
        .rotateX(-PI/2)
        .rotateZ(-shoulder_pan_joint)
        .translateZ(-shoulderOffsetZ)
        *trArm;
                                     //T2 to T6

    wrist_3_joint = atan2(-t3(2,1)/sin(wrist_2_joint),t3(2,0)/sin(wrist_2_joint))  ;           //Now we found 1,5,6 Joint


    Transform t4;
    t4=
    t4  
        .rotateX(-PI/2)
        .rotateZ(-shoulder_pan_joint)
        .translateZ(-shoulderOffsetZ)
        *trArm;
    t4 =                                                                                       // There is a bug here
    t4
        .rotateZ(-wrist_3_joint)
        .translateZ(-handLength)
        .rotateX(PI/2)
        .translateZ(-wrist2)
        .rotateZ(-wrist_2_joint)                                                                 //T1 to T4 Matrix
        .rotateX(-PI/2)
        .translateZ(-wrist1);                                                                   //T1 to T3 Matrix wrist_1_joint not reflected



    double T1_T3_p[3]; t4.apply0(T1_T3_p);

    elbow_joint = acos((T1_T3_p[0]*T1_T3_p[0]+T1_T3_p[1]*T1_T3_p[1] - upperArmLength*upperArmLength - lowerArmLength*lowerArmLength)/(2*upperArmLength*lowerArmLength));
    double gamma = atan2(T1_T3_p[1],-T1_T3_p[0]);
    double beta = acos((-upperArmLength*upperArmLength +(T1_T3_p[0]*T1_T3_p[0]+T1_T3_p[1]*T1_T3_p[1])  +(lowerArmLength*lowerArmLength))/(2*sqrt(T1_T3_p[0]*T1_T3_p[0]+T1_T3_p[1]*T1_T3_p[1])*lowerArmLength));

    shoulder_lift_joint = -(gamma+beta);
                                             //This Soulution is only for elbow_joint<0

    Transform t5;
    t5=
    t5  
        .rotateZ(-elbow_joint)
        .rotateZ(-shoulder_lift_joint)
        .rotateX(-PI/2)
        .rotateZ(-shoulder_pan_joint)
        *trArm;
    t5=
    t5
        .rotateZ(-wrist_3_joint)
        .rotateX(PI/2)
        .rotateZ(-wrist_2_joint)                                                             
        .rotateX(-PI/2);
        


    printTransform(t5);

    
    wrist_1_joint= -atan2(t5(0,1),t5(0,0)) ;
    


      
    Joint_Position.push_back(shoulder_pan_joint);
    Joint_Position.push_back(shoulder_lift_joint);
    Joint_Position.push_back(elbow_joint);
    Joint_Position.push_back(wrist_1_joint);
    Joint_Position.push_back(wrist_2_joint);
    Joint_Position.push_back(wrist_3_joint);

    std::cout<<shoulder_pan_joint <<std::endl;
    std::cout<<shoulder_lift_joint <<std::endl;
    std::cout<<elbow_joint<<std::endl;
    std::cout<<wrist_1_joint <<std::endl;
    std::cout<<wrist_2_joint <<std::endl;
    std::cout<<wrist_3_joint <<std::endl;

    return Joint_Position;
}