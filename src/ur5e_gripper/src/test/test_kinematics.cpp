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
    // double qOrg[] = {-0.0049,-1.29829,0.44895,-0.698,-1.496,-1.57};
    double qOrg[] = {0.785,-1.22372,0.26377,-0.46316,-0.5,-0.00082};
    Transform trArm;

    std::vector<double> q_position(6);
    printTransform(trArm);
    
    trArm = UR5e_forward_kinematics(qOrg);
    printTransform(trArm);
   
    q_position = UR5e_inverse_kinematics(trArm, qOrg, false);

}
