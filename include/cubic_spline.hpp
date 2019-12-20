#ifndef CUBIC_INCLUDED
#define CUBIC_INCLUDED
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
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
    Spline();
    Spline(const Spline&);
    Spline(const VectorXd &, const VectorXd &);
    ~Spline();
    Eigen::MatrixXd __calc_A(const VectorXd &);
    Eigen::VectorXd __calc_B(const VectorXd &);
    double calc(double);
    double calcd(double);
    int __search_index(double);
    double calcdd(double t);
};

class Spline2D{
private:
    Spline sx;
    Spline sy;
    VectorXd ds;
public:
    VectorXd s;
    Spline2D(const VectorXd &, const VectorXd &);
    Eigen::VectorXd __calc_s(const VectorXd &, const VectorXd &);
    std::pair<double, double> calc_position(double);
    double calc_curvature(double);
    double calc_yaw(double);
};
#endif