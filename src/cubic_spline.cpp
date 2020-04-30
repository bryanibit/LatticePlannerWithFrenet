#include "cubic_spline.hpp"
#include "Eigen/Dense"
using namespace Eigen;

/*   Cubic Spline class   */
//default constructor
Spline::Spline() {}
Spline::~Spline() {}
/* copy constructor*/
Spline::Spline(const Spline& S){
    this->x= S.x;
    this->y = S.y;
    this->nx = S.nx;
    this->a = S.a;
    this->b = S.b;
    this->c = S.c;
    this->d = S.d;
}
Spline::Spline(const VectorXd &x_, const VectorXd &y_){
        this->x = x_;
        this->y = y_;
        this->nx = x_.size();
        VectorXd h(x.size() - 1);
        this->a = this->y;
        for(int i = 1; i < x.size(); ++i){
            h(i - 1) = x(i) -x(i - 1);
        }
        auto A = __calc_A(h);
        auto B = __calc_B(h);
        this->c = A.colPivHouseholderQr().solve(B);
        std::cout << "c: " << std::endl << c << std::endl;
        // std::cout << "A * c: " << std::endl << A*c << std::endl;
        //calc spline coefficient b and d
        this->d.resize(nx - 1);
        this->b.resize(nx - 1);
        for(int i = 0; i < nx - 1; ++i){
                this->d(i) = (this->c(i + 1) - this->c(i)) / (3.0 * h(i));
                auto tb = (this->a(i + 1) - this->a(i)) / h(i) - h(i) * (this->c(i + 1) + 2.0 * this->c(i)) / 3.0;
                this->b(i) = tb;
        }

}
// t should be positive always?????
int Spline::__search_index(double t){
    for(int i = x.size() - 1; i >= 0; --i){
        if(x(i) <= t)
            return i;
    }
    return -1;
}

double Spline::calc(double t){
    if(t < this->x(0))
        return NAN;
    else if(t > this->x(x.size() - 1))
        return NAN;
    auto i = this->__search_index(t);
    if(i == -1)
        return NAN;

    auto dx = t - this->x(i);
    auto result = this->a(i) + this->b(i) * dx + this->c(i) * dx * dx + this->d(i) * dx * dx * dx;
    return result;
}

double Spline::calcd(double t){
    //Calc first derivative
    //if t is outside of the input x, return -1
    if(t < this->x(0))
        return NAN;
    else if(t > this->x(x.size() - 1))
        return NAN;
    auto i = this->__search_index(t);
    auto dx = t - this->x(i);
    auto result = this->b(i) + 2.0 * this->c(i) * dx + 3.0 * this->d(i) * pow(dx,2.0);
    return result;
}

double Spline::calcdd(double t){
    //Calc second derivative
    if(t < this->x(0))
        return NAN;
    else if(t > this->x(x.size() - 1))
        return NAN;

    auto i = this->__search_index(t);
    auto dx = t - this->x(i);
    auto result = 2.0 * this->c(i) + 6.0 * this->d(i) * dx;
    return result;
}

Eigen::MatrixXd Spline::__calc_A(const VectorXd &h){
        //calc matrix A for spline coefficient c
        Eigen::MatrixXd A(nx,nx);
        A.setZero();
        A(0,0) = 1.0;
        for(int i = 0; i < nx - 1; ++i){
                if(i != (nx - 2))
                        A(i + 1, i + 1) = 2.0 * (h(i) + h(i + 1));
                A(i + 1, i) = h(i);
                A(i, i + 1) = h(i);
        }
        A(0, 1) = 0.0;
        A(nx - 1, nx - 2) = 0.0;
        A(nx - 1, nx - 1) = 1.0;
        std::cout << "A: " << std::endl << A << std::endl;
        return A;
}

Eigen::VectorXd Spline::__calc_B(const VectorXd &h) {
//calc matrix B for spline coefficient c

        Eigen::VectorXd B(this->nx);
        B.setZero();
        for (int i = 0; i < nx - 2; ++i)
                B(i + 1) = 3.0 * (this->a(i + 2) - this->a(i + 1)) / h(i + 1) -
                           3.0 * (this->a(i + 1) - this->a(i)) / h(i);
        std::cout << "B: " << std::endl << B << std::endl;
        return B;
}

/*    2D Cubic Spline class  */
Spline2D::Spline2D(const VectorXd &x_, const VectorXd &y_){
    this->s = this->__calc_s(x_, y_);
    this->sx = Spline(this->s, x_);
    this->sy = Spline(this->s, y_);
}

Eigen::VectorXd Spline2D::__calc_s(const VectorXd &x, const VectorXd &y){
    VectorXd dx(x.size() - 1);
    for(int i = 1; i < x.size(); ++i)
        dx(i - 1) = x(i) - x(i - 1);
    VectorXd dy(y.size() - 1);
    for(int i = 1; i < y.size(); ++i)
        dy(i - 1) = y(i) - y(i - 1);
    this->ds = (sqrt(dx.array() * dx.array() + dy.array() * dy.array())).matrix();
    VectorXd s(ds.size() + 1);
    s(0) = 0;
    double sum = 0;
    for(int i = 0; i < ds.size(); ++i){
        sum += ds(i);
        s(i + 1) = sum;
    }
    return s;
}

std::pair<double, double> Spline2D::calc_position(double s){
    auto x = this->sx.calc(s);
    auto y = this->sy.calc(s);
    return std::make_pair(x,y);
};

double Spline2D::calc_curvature(double s){
    auto dx = this->sx.calcd(s);
    auto ddx = this->sx.calcdd(s);
    auto dy = this->sy.calcd(s);
    auto ddy = this->sy.calcdd(s);
    auto k = (ddy * dx - ddx * dy) / (dx *dx + dy *dy);
    return k;
}

double Spline2D::calc_yaw(double s){
    auto dx = this->sx.calcd(s);
    auto dy = this->sy.calcd(s);
    auto yaw = atan(dy/dx);
    return yaw;
}