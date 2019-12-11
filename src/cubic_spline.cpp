#include <cubic_spline.hpp>
#include <Eigen/Dense>
using namespace Eigen;

Spline::Spline(const VectorXd &x_, const VectorXd &y_){
        x = x_;
        y = y_;
        VectorXd h;
        a = y;
        for(int i = 1; i < x.size(); ++i){
            h << x(i) -x(i - 1);
        }
        auto A = __calc_A(h);
        auto B = __calc_B(h);
        c = A.colPivHouseholderQr().solve(B);
        std::cout << "c: " << std::endl << c << std::endl;
        //calc spline coefficient b and d
        for(int i = 0; i < nx - 1; ++i){
                this->d << (this->c(i + 1) - this->c(i)) / (3.0 * h(i));
                auto tb = (this->a(i + 1) - this->a(i)) / h(i) - h(i) * (this->c(i + 1) + 2.0 * this->c(i)) / 3.0;
                this->b << tb;
        }

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