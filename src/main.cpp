#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Frenet_path.hpp>
#include <cubic_spline.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;
int main() {
//    Matrix3d m;
//    m << 1,-1,-1,-1,1,1,0,-4,-2;
//    cout << "m =" << endl << m << endl;
//    Vector3d v(3);
//    v << -1, 1, -2;
//    auto x = m.colPivHouseholderQr().solve(v);
//    std::cout << "ans: " << std::endl << x << std::endl;
//    std::cout << "multiple ans: " << std::endl << m*x << std::endl;
//    std::cout << "with the same with v: " << std::endl << v << std::endl;
    std::cout << "Start...\n";
    // Current status will be shown in this file
    // Way points
    Eigen::VectorXd wx;
    wx << 0.0, 10.0, 20.5, 35.0, 70.5;
    Eigen::VectorXd wy;
    wy << 0.0, -6.0, 5.0, 6.5, 0.0;
    // obstacles
    Eigen::Matrix<float, 5, 2> ob;
    ob << 20.0, 10.0,
            30.0, 6.0,
            30.0, 8.0,
            35.0, 8.0,
            50.0, 3.0;

    return 0;
}