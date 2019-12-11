#ifndef CUBIC_INCLUDED
#define CUBIC_INCLUDED
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;

class Spline{

private:
    VectorXd x;
    VectorXd y;
    int nx;
    VectorXd a;
    VectorXd b;
    VectorXd c;
    VectorXd d;
public:
    Spline(const VectorXd &, const VectorXd &);
    Eigen::MatrixXd __calc_A(const VectorXd &);
    Eigen::VectorXd __calc_B(const VectorXd &);
};

class Spline2D{
public:
    VectorXd s;
    Spline sx;
    Spline sy;
};
#endif