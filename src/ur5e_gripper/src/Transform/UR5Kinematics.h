#ifndef UR5E_KINEMATIC_H_
#define UR5E_KINEMATIC_H_

#include "Transform.h"
#include <stdio.h>
#include <math.h>
#include <vector>

const double PI = 2*asin(1);
const double RAD_TO_DEG = 180.0/PI;
const double DEG_TO_RAD = 2*PI/180.0;

const double shoulderOffsetZ = 0.1625; //ground to shoulder pitch height
const double lowerArmLength=0.425;    
const double upperArmLength=0.3922;
const double wrist1 = 0.1333;
const double wrist2 = 0.0997;
const double handLength = 0.0996;
double toolOffsetZ = 0.00;


const double lowerArmEff=sqrt(lowerArmLength*lowerArmLength);
const double upperArmEff=sqrt(upperArmLength*upperArmLength);


double mod_angle(double q);
Transform UR5e_forward_kinematics(double *q);
std::vector<double>UR5e_inverse_kinematics(Transform tr, const double *qOrg, bool wrist_2_Flipped);

#endif