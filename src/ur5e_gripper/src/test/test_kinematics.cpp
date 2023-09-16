#include "../Transform/Transform.cpp"
#include "../Transform/UR5Kinematics.cpp"
#include <iostream>

using namespace std;

int main()
{
    // double qOrg[] ={-0.698 , -0.49883, -0.34898,-0.29913,-0.299,-0.698}; // end effector = (-0.84531, 0.14647,0.48697)
    double qOrg[] = {-0.29898, -0.49884, -0.44894, -0.698064, 0.299, -1.297};
    // double qOrg[] ={-0.49898, -0.49892,-0.15002,-1.29689,-0.299,-0.00146};
    // double qOrg[] ={0, 0,0,0,0,0};
    std::vector<double> test(6, 0);
    double theta = -3.141;
    Transform trArm;
    trArm.clear();
    printTransform(trArm);
    std::vector<double> q_position(6);
    trArm = UR5e_forward_kinematics(qOrg);
    printTransform(trArm);
    q_position = UR5e_inverse_kinematics(trArm, qOrg, false);

    mod_angle(theta);
    // for( int i ; i < sizeof(q_positi on); i++){
    //     cout << q_position[i] << " ";
    // }
}