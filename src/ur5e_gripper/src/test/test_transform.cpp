#include"../Transform/Transform.cpp"
#include"../Transform/UR5Kinematics.cpp"
#include<iostream>



using namespace std;

int main(void){
        Transform t1;
        
        printTransform(t1);
        t1.translateX(-0.5);
        printTransform(t1);
    }