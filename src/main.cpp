#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "Frenet_path.hpp"
#include "cubic_spline.hpp"
//#include "../matplotlib-cpp/matplotlibcpp.h"
#include <chrono>

using namespace std;
using namespace cv;
using namespace Eigen;
//namespace plt = matplotlibcpp;

int main() {
    auto show_animation = true;
    std::cout << "Start...\n";
    // Current status will be shown in this file
    // Way points
    Eigen::VectorXd wx(6);
    wx << 0.0, 10.0, 20.5, 35.0, 70.5, 90.0;
    Eigen::VectorXd wy(6);
    wy << 0.0, -6.0, 5.0, 6.5, 0.0, 0.5;
    // obstacles
    Eigen::MatrixXd ob(5,2);
	// (30.057, 7.10775)
    ob <<   20.0, 10.0,
            30.0, 5.5,
            30.0, 8.5,
            35.0, 8.0,
            50.0, 2.5;
    auto fplan = Frenet_plan();
    auto contuple = fplan.generate_target_course(wx, wy);
    // tx ty tyaw tc is vector<double>
    vector<double> tx = std::get<0>(contuple);
    vector<double> ty = std::get<1>(contuple);
	//for (int k = 0; k < tx.size(); ++k)
	//	std::cout << "(" << tx.at(k) << ", " << ty.at(k) << ")" << std::endl;
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

    Mat map(static_cast<int>(ceil(map_height)), static_cast<int>(ceil(map_width)), CV_8UC3, Scalar(255,255,255));
	//std::cout << "(" << *origxmin << ", " << *origymin << "), (" << *origxmax << ", " << *origymax << std::endl;
    while(true) {
        auto path = fplan.frenet_optimal_planning(
                csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob);
		if (!path.valid){
            std::cout << "Not any paths found\n";
            break;
        }
        s0 = path.s[1];
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];
		//std::cout << s0 << ", " << c_d << std::endl;
//        if(i % 10 == 0)
//            std::cout << "----------" << i << "----------" << std::endl;
        if(pow(path.x(1) - tx[tx.size() - 1], 2) + pow(path.y(1) - ty[ty.size() - 1], 2) <= 1.0){
            std::cout << "Goal\n";
            break;
        }

        if(show_animation) {
            // draw
//            plt::clf();
//            plt::plot(tx, ty);
//            vector<double> vec_x(path.x.data(), path.x.data() + path.x.rows() * path.x.cols());
//            vector<double> vec_y(path.y.data(), path.y.data() + path.y.rows() * path.y.cols());
//            plt::plot(vec_x, vec_y, "-or");
//            vector<double> ob_x(ob.data(), ob.data() + ob.size()/2);
//            vector<double> ob_y(ob.data() + ob.size()/2, ob.data() + ob.size());
//            plt::plot(ob_x, ob_y, "xk");
////            plt::xlim(path.x[1] - area, path.x[1] + area);
////            plt::ylim(path.y[1] - area, path.y[1] + area);
//            plt::title("v[km/h]:" + to_string(c_speed * 3.6));
//            plt::grid(true);
//            plt::pause(0.1);

            map = cv::Scalar(255,255,255);
            for(int i = 0; i < ob.rows(); ++i){
                cv::circle(map, cv::Point2d((ob(i, 0)- *origxmin) * 10, (ob(i, 1)- *origymin) * 10), 4,
                           cv::Scalar(0,255,0), -1);
            }
            for(int i= 0; i < tx.size() - 1; ++i){
                cv::line(map, cv::Point2d((tx[i] - *origxmin) * 10, (ty[i] - *origymin)* 10),
                         cv::Point2d((tx[i+1] - *origxmin) * 10, (ty[i + 1] - *origymin) * 10), cv::Scalar(255,0,0),2);
            }
            for(int i = 1; i < path.x.size() - 1; ++i){
                cv::line(map, cv::Point2d((path.x(i) - *origxmin) * 10, (path.y(i) - *origymin) * 10),
                         cv::Point2d((path.x(i + 1) - *origxmin) * 10, (path.y(i + 1) - *origymin) * 10), cv::Scalar(0, 0, 255), 2);
            }
			cv::flip(map, map, 0);
            cv::imshow("img", map);
            cv::waitKey(100);

        }
    }
    std::cout << "finish\n";
    std::cin.get();
    return 0;
}
