#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <math.h>
#include <stdio.h>
#include <vector>


#include <iostream>

//Some problems in it
//Transform matrix changes when calculations come after the Matrix

class Transform {
public:
    Transform() {clear();}
    virtual ~Transform() {}
    void clear();
    Transform &translate(double x, double y, double z);
    Transform &translate(const double *p);
    Transform &translateNegative(const double *p);
    Transform &translateX(double x = 0);
    Transform &translateY(double y = 0);
    Transform &translateZ(double y = 0);
    Transform &rotateX(double theta = 0);
    Transform &rotateY(double theta = 0);
    Transform &rotateZ(double theta = 0);
    Transform &rotateDotX(double theta = 0);
    Transform &rotateDotY(double theta = 0);
    Transform &rotateDotZ(double theta = 0);
    Transform &mDH(double alpha, double xDis, double theta, double zDis);
    void apply(double list[3]);
    void apply0(double list[3]) ;
    double getZ();
    void showTrans();
    double& operator() (int i , int j);
    const double operator() (int i, int j) const;

private:
    double t[4][4];
};

Transform operator*(const Transform &t1, const Transform &t2);
Transform inv(const Transform &t);
Transform trycopy (const Transform &t);
Transform transform6D(const double p[6]);
Transform transformQuatP(const double q[7]);

// std::vector<double> position6D(const Transform &t);
// std::vector<double> to_quatp(const Transform &t);

void getAngularVelocityTensor(const Transform &adot, const Transform &ainv,double *av);
void printTransform(Transform t);
void printVector(std::vector<double> v);
std::vector<double> position6D(const Transform &t1);
class Jacobian{
public:
    Jacobian();
    virtual ~Jacobian() {}

    Jacobian &caculate6(
    const Transform &A,
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    double mass, const double* inertiaMatrix);


  Jacobian &calculate7(
    const Transform &A,
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6,
    double mass, const double* inertiaMatrix);

  Jacobian &calculateVel7(
    const Transform &A,
    const Transform &Adot0,
    const Transform &Adot1,
    const Transform &Adot2,
    const Transform &Adot3,
    const Transform &Adot4,
    const Transform &Adot5,
    const Transform &Adot6);

  void clear();


  void calculate_b_matrix(const double*inertiaMatrix);
  void dump_b_matrix(double* ret);
  void dump_jacobian(double* ret);
  void accumulate_stall_torque(double* torque,double forcex, double forcey, double forcez);
  void print();

private:
  int num_of_joints;
  double m;
  double b[7][7];
  double v[7][3];
  double w[7][3];
};






#endif
