#include "UR5Kinematics.h"
using namespace std;
double mod_angle(double theta){
    if(theta>PI) return theta-2*PI;
    if(theta<-PI) return theta+2*PI;
    return theta;
}

Transform UR5e_forward_kinematics(double *q){
    Transform t;
    for(int i =0 ; i <6; i++){
        cout << q[i]<< endl;
    }
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
        .translateZ(handLength);             //Wrist3Frame
        // .translateZ(toolOffsetZ);           //ToolFrame
    return t;
}

//Reference Frame of Kinematics is World Frame which its x axis pointing Wrist Frame of the Home configuration

std::vector<double> UR5e_inverse_kinematics(Transform trArm,const double *qOrg, bool wrist_2_Flipped){
    std::vector<double> Joint_Position(6);
    double shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint;
    double handOffsetZ = handLength + toolOffsetZ;
    trArm = trArm.translate(0,0,0);                  //TCP Position
    // trArm = trArm.translateZ(-toolOffsetZ);                             //Tool0 Position
    
    Transform t1; 
    t1 = t1.translateZ(-shoulderOffsetZ)*trArm;
    double EE_p[3]; t1.apply0(EE_p);                                       //EE relative to shoulder_pan_joint

    t1 = t1.translateZ(-handLength);
    double Wrist_p[3]; t1.apply0(Wrist_p);                                 //Wrist Position relative to shoulder_pan_joint    Wrist Position = wrist_2_link frame

    double shoulder_wrist_2D = sqrt(Wrist_p[0]*Wrist_p[0] + Wrist_p[1]*Wrist_p[1]);
    double yaw_offset = asin(wrist1/shoulder_wrist_2D);
    shoulder_pan_joint = atan2(Wrist_p[1],Wrist_p[0]) - yaw_offset;

    double cwr = (-EE_p[0]*sin(shoulder_pan_joint)+EE_p[1]*cos(shoulder_pan_joint)-wrist1)/handOffsetZ;
    if(cwr > 1.0)  cwr =  1.0;
    if(cwr < -1.0) cwr = -1.0;
    wrist_2_joint = acos(cwr);
    if(wrist_2_Flipped) wrist_2_joint = -wrist_2_joint;



    if(fabs(wrist_2_joint)<0.01){                                                               // It means that sin(wrist_2_joint) is almost 0
        printf("Pitches are Parallel");
        shoulder_lift_joint = qOrg[1];

        Transform t2;
        t2  
            .translateZ(lowerArmLength)                                                            // Elbow to Second Wrist Frame
            .rotateZ(-shoulder_lift_joint)
            .rotateX(-PI/2)
            .rotateZ(-shoulder_pan_joint)
            .translateZ(-shoulderOffsetZ)
            *trArm
            .translateZ(-handOffsetZ);

        double elbowWrist2_p[3]; t2.apply0(elbowWrist2_p);                                      //Elbow to wrist 2 Frame
        double elbowWrist2Dis = sqrt(elbowWrist2_p[0]*elbowWrist2_p[0] + elbowWrist2_p[2]*elbowWrist2_p[2]);
        double elbowPitchOffset = acos(
            (elbowWrist2Dis*elbowWrist2Dis+upperArmLength*upperArmLength-wrist2*wrist2)/(2*elbowWrist2Dis*upperArmLength));
        elbow_joint = atan2(elbowWrist2_p[0]/elbowWrist2Dis,elbowWrist2_p[2]/elbowWrist2Dis)-elbowPitchOffset;

        Transform t6;
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

        for (int i =0; i<6; i++){
            std::cout<<Joint_Position[i] <<std::endl;
        }


        return Joint_Position;
    }

    Transform t3;
    t3
        .rotateZ(-PI/2)
        .rotateZ(-shoulder_pan_joint)
        .translateZ(-shoulderOffsetZ)
        *trArm;

    wrist_3_joint = atan2(-t3(1,2)/sin(wrist_2_joint),t3(2,0)/sin(wrist_2_joint));              //We found 1,5,6 Joint
    
    Transform t4;
    t4
        .rotateZ(-shoulder_pan_joint)
        .translateZ(-shoulderOffsetZ)
        *trArm
        .rotateZ(-wrist_3_joint)
        .translateZ(-handLength)
        .rotateX(PI/2)
        .translateZ(-wrist2)
        .rotateZ(-wrist_2_joint)                                                                 //T1 to T4 Matrix
        .translateY(-wrist1);                                                                    //T1 to T3 Matrix wrist_1_joint not reflected
    
    double T1_T3_p[3]; t4.apply0(T1_T3_p);

    elbow_joint = acos((T1_T3_p[0]*T1_T3_p[0]+T1_T3_p[1]*T1_T3_p[1] - upperArmLength*upperArmLength - lowerArmLength*lowerArmLength)/2*upperArmLength*lowerArmLength);
    shoulder_lift_joint = -atan2(T1_T3_p[1],-T1_T3_p[0])+asin(upperArmLength/sqrt(T1_T3_p[0]*T1_T3_p[0]+T1_T3_p[1]*T1_T3_p[1]));

    Transform t5;
    t5
        .translateX(lowerArmLength)
        .rotateZ(-shoulder_lift_joint)
        .rotateX(-PI/2)
        .rotateZ(-shoulder_pan_joint)
        .translateZ(-shoulderOffsetZ)
        *trArm
        .rotateZ(-wrist_3_joint)
        .translateZ(-handLength)
        .rotateX(PI/2)
        .translateZ(-wrist2)
        .rotateZ(-wrist_2_joint)                                                                
        .translateY(-wrist1);                                                                     //T3 to T4 Matrix
    

      wrist_1_joint = atan2(t5(0,1),t5(0,0));


      
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
    for (int i =0; i<6; i++){
            std::cout<<Joint_Position[i] <<std::endl;
        }
    return Joint_Position;
      






}