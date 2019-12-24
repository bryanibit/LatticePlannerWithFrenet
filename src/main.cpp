#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Frenet_path.hpp>
#include <cubic_spline.hpp>
#include "../matplotlib-cpp/matplotlibcpp.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace Eigen;
namespace plt = matplotlibcpp;

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
    // tx ty tyaw tc is vector<double>
    vector<double> tx = std::get<0>(contuple);
    vector<double> ty = std::get<1>(contuple);
    auto origxmin = std::min_element(tx.begin(), tx.end());
    auto origxmax = std::max_element(tx.begin(), tx.end());
    auto origymin = std::min_element(ty.begin(), ty.end());
    auto origymax = std::max_element(ty.begin(), ty.end());
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
    auto map_width = (*origxmax - *origxmin) * 10 + 1;
    auto map_height = (*origymax - *origymin) * 20 + 1;
//    std::cout <<  "(xmin, ymin): (" << *origxmin << ", " << *origymin << ")" << std::endl;
//    std::cout <<  "(xmax, ymax): (" << *origxmax << ", " << *origymax << ")" << std::endl;

    Mat map(static_cast<int>(ceil(map_height)), static_cast<int>(ceil(map_width)), CV_8UC3, Scalar(255,255,255));

    for(int i = 0; i < SIM_LOOP; ++i){
//        std::cout << "7\n";
        //auto t_start = std::chrono::high_resolution_clock::now();


        auto path = fplan.frenet_optimal_planning(
                csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);

        std::cout << "path1x, path1y: " << path.x(1) << ", " << path.y(1) << std::endl;

        //auto t_end = std::chrono::high_resolution_clock::now();
        //double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
//        std::cout << "frenet_optimal_planning dur: " << elapsed_time_ms << " ms\n";
        //std::cout << "path.s size: " << path.s.size() << std::endl;
        //std::cout << "path.d size: " << path.d.size() << std::endl;
        //std::cout << "path.d_d size: " << path.d_d.size() << std::endl;
        //std::cout << "path.d_dd size: " << path.d_dd.size() << std::endl;
//        std::cout << "8\n";
        s0 = path.s[1];
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];

        if(i % 10 == 0)
            std::cout << "----------" << i << "----------" << std::endl;
        if(pow(path.x(1) - tx[tx.size() - 1], 2) + pow(path.y(1) - ty[ty.size() - 1], 2) <= 1.0){
            std::cout << "Goal\n";
            break;
        }

        if(show_animation) {
            // draw
            plt::clf();
            plt::plot(tx, ty);
            vector<double> vec_x(path.x.data(), path.x.data() + path.x.rows() * path.x.cols());
            vector<double> vec_y(path.y.data(), path.y.data() + path.y.rows() * path.y.cols());
            plt::plot(vec_x, vec_y, "-or");
            vector<double> ob_x(ob.data(), ob.data() + ob.size()/2);
            vector<double> ob_y(ob.data() + ob.size()/2, ob.data() + ob.size());
            plt::plot(ob_x, ob_y, "xk");
//            plt::xlim(path.x[1] - area, path.x[1] + area);
//            plt::ylim(path.y[1] - area, path.y[1] + area);
            plt::title("v[km/h]:" + to_string(c_speed * 3.6));
            plt::grid(true);
            plt::pause(0.1);
//            map = cv::Scalar(255,255,255);
//            for(int i = 0; i < ob.rows(); ++i){
//                cv::circle(map, cv::Point2d((ob(i, 0)- *origxmin) * 10, (ob(i, 1)- *origymin) * 10), 4,
//                           cv::Scalar(255,255,255), -1);
//            }
//            for(int i= 0; i < tx.size() - 1; ++i){
//                cv::line(map, cv::Point2d((tx[i] - *origxmin) * 10, (ty[i] - *origymin)* 10),
//                         cv::Point2d((tx[i+1] - *origxmin) * 10, (ty[i + 1] - *origymin) * 10), cv::Scalar(255,0,0),2);
//            }
//            for(int i = 1; i < path.x.size() - 1; ++i){
//                cv::line(map, cv::Point2d((path.x(i) - *origxmin) * 10, (path.y(i) - *origymin) * 10),
//                         cv::Point2d((path.x(i + 1) - *origxmin) * 10, (path.y(i + 1) - *origymin) * 10), cv::Scalar(0, 0, 255), 2);
//            }
//            cv::imshow("img", map);
//            cv::waitKey(100);

        }
    }
    std::cout << "finish\n";
    std::cin.get();
    return 0;
}