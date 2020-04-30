#ifndef FRENET_INCLUDED
#define FRENET_INCLUDED
#include "Eigen/Dense"
#include "cubic_spline.hpp"
#include <vector>
#include <tuple>
#include <chrono>
#include <cmath>
using namespace Eigen;
class quintic_polynomial{
private:
    double xs;
    double vxs;
    double axs;
    double xe;
    double vxe;
    double axe;
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
public:
    quintic_polynomial();
    ~quintic_polynomial();
    quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T);
    double calc_point(double);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);


};

class quartic_polynomial{
private:
    // calc coefficient of quintic polynomial
    double xs;
    double vxs;
    double axs;
    double vxe;
    double axe;

    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
public:
    quartic_polynomial();
    ~quartic_polynomial();
    quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double T);
    double calc_point(double);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
};

class Frenet_path{
public:
    Frenet_path();
    ~Frenet_path();
    Frenet_path(const Frenet_path&);

    VectorXd t;
    VectorXd d;
    VectorXd d_d;
    VectorXd d_dd;
    VectorXd d_ddd;
    VectorXd s;
    VectorXd s_d;
    VectorXd s_dd;
    VectorXd s_ddd;
    double cd;
    double cv;
    double cf;

    VectorXd x;
    VectorXd y;
    VectorXd yaw;
    VectorXd ds;
    VectorXd c;
};

class Frenet_plan{
public:
    std::vector<Frenet_path> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0);
    std::vector<Frenet_path> calc_global_paths(std::vector<Frenet_path> &, Spline2D &);
    bool check_collision(Frenet_path&, MatrixXd&);
    std::vector<Frenet_path> check_paths(std::vector<Frenet_path> &, MatrixXd &);
    Frenet_path frenet_optimal_planning(Spline2D &, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, MatrixXd &ob);
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D>
    generate_target_course(VectorXd&, VectorXd&);
};
#endif
