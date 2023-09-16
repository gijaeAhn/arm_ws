#include "../Transform/Transform.cpp"
#include "../Transform/UR5Kinematics.cpp"
#include <iostream>

using namespace std;

int main()
{
    // double qOrg[] ={-0.698 , -0.49883, -0.34898,-0.29913,-0.299,-0.698}; // end effector = (-0.84531, 0.14647,0.48697)
    // double qOrg[] = {-0.29898, -0.49884, -0.44894, -0.698064, 0.299, -1.297};
    // double qOrg[] ={-0.49898, -0.49892,-0.15002,-1.29689,-0.299,-0.00146};
    // double qOrg[] ={0, -PI/2,PI/2,PI/4,PI/4,PI/3};
    double qOrg[] = {0.785,-1.22372,1.96377,0.46316,1.35981,-0.00082};
    Transform trArm;
    Transform trArm_test;

    trArm.clear();
    trArm_test.clear();
    // trArm_test.rotateX(PI/2).rotateZ(-PI/2).rotateZ(PI/2)
    // .rotateZ(PI/4).rotateX(PI/2).rotateZ(PI/4).rotateX(-PI/2)
    // .rotateZ(PI/3);
    printTransform(trArm);
    std::vector<double> q_position(6);
    trArm = UR5e_forward_kinematics(qOrg);
    printTransform(trArm);
   
    q_position = UR5e_inverse_kinematics(trArm, qOrg, false);

}

//2번째
// -0.9594 0.1082 -0.2604 -0.7355 
// 0.2124 -0.3301 -0.9197 -0.01238 
// -0.1855 -0.9377 0.2937 0.7211 
// 0 0 0 1 