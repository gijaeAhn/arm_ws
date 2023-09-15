#include"../Transform/Transform.cpp"
#include"../Transform/UR5Kinematics.cpp"
#include<iostream>

using namespace std;

int main(){
    // double qOrg[] ={-1.09699, -0.69764, -0.44906,-0.69799,-0.499,-1.097};
    double qOrg[] ={-0.69776, -0.92496,-0.43079,-1.10084,-0.49927,-0.299};
    // double qOrg[] ={0, 0,0,0,0,0};
    std::vector<double> test(6,0);

    Transform trArm;
    trArm.clear();
    std::vector<double> q_position(6);
    printTransform(trArm);
    trArm = UR5e_forward_kinematics(qOrg);
    printTransform(trArm);

    q_position = UR5e_inverse_kinematics(trArm,qOrg,false);

    for( int i ; i < sizeof(q_position); i++){
        cout << q_position[i] << " ";
    }

}