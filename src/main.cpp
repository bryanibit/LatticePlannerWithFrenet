#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Frenet_path.hpp>
#include <cubic_spline.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;

const int SIM_LOOP = 500;
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
    auto show_animation = true;
    std::cout << "Start...\n";
    // Current status will be shown in this file
    // Way points
    Eigen::VectorXd wx(5);
    wx << 0.0, 10.0, 20.5, 35.0, 70.5;
    Eigen::VectorXd wy(5);
    wy << 0.0, -6.0, 5.0, 6.5, 0.0;
    // obstacles
    Eigen::MatrixXd ob(5,2);
    ob <<   20.0, 10.0,
            30.0, 6.0,
            30.0, 8.0,
            35.0, 8.0,
            50.0, 3.0;
    auto fplan = Frenet_plan();
    auto contuple = fplan.generate_target_course(wx, wy);
    auto tx = std::get<0>(contuple);
    auto ty = std::get<1>(contuple);
    auto tyaw = std::get<2>(contuple);
    auto tc = std::get<3>(contuple);
    auto csp = std::get<4>(contuple);
    // initial state
    auto c_speed = 10.0 / 3.6;  // current speed [m/s]
    auto c_d = 2.0;  // current lateral position [m]
    auto c_d_d = 0.0;  // current lateral speed [m/s]
    auto c_d_dd = 0.0;  // current latral acceleration [m/s]
    auto s0 = 0.0;  // current course position

    auto area = 20.0;  // animation area length [m]
    for(int i = 0; i < SIM_LOOP; ++i){
        auto path = fplan.frenet_optimal_planning(
                csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);
        std::cout << "path.s size: " << path.s.size() << std::endl;
        std::cout << "path.d size: " << path.d.size() << std::endl;
        std::cout << "path.d_d size: " << path.d_d.size() << std::endl;
        std::cout << "path.d_dd size: " << path.d_dd.size() << std::endl;
        auto s0 = path.s[1];
        auto c_d = path.d[1];
        auto c_d_d = path.d_d[1];
        auto c_d_dd = path.d_dd[1];
        auto c_speed = path.s_d[1];
        if(pow(path.x(1) - tx[tx.size() - 1], 2) + pow(path.y(1) - ty[ty.size() - 1], 2) <= 1.0){
            std::cout << "Goal\n";
            break;
        }
        if(show_animation) {
            // draw
            std::cout << "draw\n";
        }
    }
    std::cout << "finish\n";
    return 0;
}