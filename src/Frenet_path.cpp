#include <Frenet_path.hpp>
#define M_PI 3.14159265358979323846

const double MAX_SPEED = 50.0 / 3.6; // maximum speed [m/s]
const double MAX_ACCEL = 2.0;  // maximum acceleration [m/ss]
const double MAX_CURVATURE = 1.0;  // maximum curvature [1/m]
const double MAX_ROAD_WIDTH = 7.0;  // maximum road width [m]
const double D_ROAD_W = 1.0;  // road width sampling length [m]
const double DT = 0.2;  // time tick [s]
const double MAXT = 5.0;  // max prediction time [m]
const double MINT = 4.0;  // min prediction time [m]
const double TARGET_SPEED = 30.0 / 3.6;  // target speed [m/s]
const double D_T_S = 5.0 / 3.6;  // target speed sampling length [m/s]
const double N_S_SAMPLE = 1;  // sampling number of target speed
const double ROBOT_RADIUS = 2.0;  // robot radius [m]

// cost weights
const double KJ = 0.1;
const double KT = 0.1;
const double KD = 1.0;
const double KLAT = 1.0;
const double KLON = 1.0;

const bool show_animation = true;

// quintic_polynomial class
quintic_polynomial::quintic_polynomial(){}
quintic_polynomial::~quintic_polynomial(){}

quintic_polynomial::quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T){
    // calc coefficient of quintic polynomial
    this->xs = xs;
    this->vxs = vxs;
    this->axs = axs;
    this->xe = xe;
    this->vxe = vxe;
    this->axe = axe;

    this->a0 = xs;
    this->a1 = vxs;
    this->a2 = axs / 2.0;

    MatrixXd A(3,3);
    A << pow(T,3), pow(T,4), pow(T, 5),
    3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
    6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
//    std::cout << "A: " << std::endl << A << std::endl;
//    std::cout << "A.size(): " << A.rows() << ",  " << A.cols() << std::endl;

    VectorXd b(3);
    b << xe - this->a0 - this->a1 * T - this->a2 * pow(T, 2),
            vxe - this->a1 - this->a2 * T * 2,
            axe - this->a2 * 2;
//    std::cout << "b: " << std::endl << b << std::endl;
//    std::cout << "b size(): " << b.size() << std::endl;
//    if(A.determinant() != 0)
//        std::cout << "A is nonsingular\n";
    VectorXd x = A.colPivHouseholderQr().solve(b);

    this->a3 = x(0);
    this->a4 = x(1);
    this->a5 = x(2);
}

double quintic_polynomial::calc_point(double t){
    auto xt = this->a0 + this->a1 * t + this->a2 * pow(t, 2) + this->a3 * pow(t, 3) + this->a4 * pow(t, 4) + this->a5 * pow(t, 5);
    return xt;
}

double quintic_polynomial::calc_first_derivative(double t) {
    auto xt = this->a1 + 2* this->a2 * t + 3 * this->a3 * pow(t, 2) + 4* this->a4 * pow(t, 3) + 5* this->a5 * pow(t, 4);
    return xt;
}

double quintic_polynomial::calc_second_derivative(double t){
    auto xt = 2* this->a2 + 6 * this->a3 * t + 12* this->a4 * pow(t, 2) + 20* this->a5 * pow(t, 3);
    return xt;
}

double quintic_polynomial::calc_third_derivative(double t){
    auto xt = 6 * this->a3 + 24* this->a4 * t + 60* this->a5 * pow(t, 2);
    return xt;
}


// quartic_polynomial class

quartic_polynomial::quartic_polynomial(){}
quartic_polynomial::~quartic_polynomial(){}

quartic_polynomial::quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double T) {
    this->xs = xs;
    this->vxs = vxs;
    this->axs = axs;
    this->vxe = vxe;
    this->axe = axe;

    this->a0 = xs;
    this->a1 = vxs;
    this->a2 = axs / 2.0;

    MatrixXd A(2, 2);
    A << 3 * pow(T, 2), 4 * pow(T, 3),
            6 * T, 12 * pow(T, 2);

    VectorXd v(2);
    v << vxe - this->a1 - 2 * this->a2 * T,
            axe - 2 * this->a2;
    VectorXd x = A.colPivHouseholderQr().solve(v);
    this->a3 = x(0);
    this->a4 = x(1);
}
double quartic_polynomial::calc_point(double t){
        auto xt = this->a0 + this->a1 * t + this->a2 * pow(t, 2) + this->a3 * pow(t, 3) + this->a4 * pow(t, 4);
        return xt;
}

double quartic_polynomial::calc_first_derivative(double t){
    auto xt = this->a1 + 2 * this->a2 * t + 3 * this->a3 * pow(t, 2) + 4 * this->a4 * pow(t, 3);
    return xt;
}

double quartic_polynomial::calc_second_derivative(double t) {
    auto xt = 2 * this->a2 + 6 * this->a3 * t + 12 * this->a4 * pow(t, 2);
    return xt;
}

double quartic_polynomial::calc_third_derivative(double t) {
    auto xt = 6 * this->a3 + 24 * this->a4 * t;
    return xt;
}

Frenet_path::Frenet_path(){}
Frenet_path::~Frenet_path() {}

Frenet_path::Frenet_path(const Frenet_path& fp){
    this->t = fp.t;
    this->d = fp.d;
    this->d_d = fp.d_d;
    this->d_dd = fp.d_dd;
    this->d_ddd = fp.d_ddd;
    this->s = fp.s;
    this->s_d = fp.s_d;
    this->s_dd = fp.s_dd;
    this->s_ddd = fp.s_ddd;
    this->cd = fp.cd;
    this->cv = fp.cv;
    this->cf = fp.cf;

    this->x = fp.x;
    this->y = fp.y;
    this->yaw = fp.yaw;
    this->ds = fp.ds;
    this->c = fp.c;
}

std::vector<Frenet_path> Frenet_plan::calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
    std::vector<Frenet_path> frenet_paths;
    //  generate path to each offset goal
    for(double di = 0.0 - MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W){
//        std::cout << "start path\n";
        //  Lateral motion planning
        for(double Ti = MINT; Ti < MAXT; Ti += DT){
            Frenet_path fp = Frenet_path();
            //std::cout << "Ti / DT: " << Ti/DT << std::endl;
            //std::cout << "ceil of Ti / DT:" << ceil(Ti/DT) << std::endl;
            fp.t.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_ddd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_ddd.resize(static_cast<int> (ceil(Ti/DT)));

            auto lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for(auto i_t = std::make_pair(0, 0.0); i_t.first < static_cast<int>(ceil(Ti/DT)) && i_t.second < Ti; ++i_t.first, i_t.second += DT) {
                fp.t(i_t.first) = i_t.second;
            }
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d(i) = lat_qp.calc_point(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_d(i) = lat_qp.calc_first_derivative(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_dd(i) = lat_qp.calc_second_derivative(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_ddd(i) = lat_qp.calc_third_derivative(fp.t(i));

            // Loongitudinal motion planning (Velocity keeping)
            for(double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE; tv < TARGET_SPEED + D_T_S * N_S_SAMPLE; tv += D_T_S){
                auto tfp = fp;
                auto lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti);
                //std::cout << "fp.t.size(): " << fp.t.size() << std::endl;
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s(i) = lon_qp.calc_point(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_d(i) = lon_qp.calc_first_derivative(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_dd(i) = lon_qp.calc_second_derivative(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_ddd(i) = lon_qp.calc_third_derivative(fp.t(i));

                auto Jp = (tfp.d_ddd.array() * tfp.d_ddd.array()).matrix().sum();
                auto Js = (tfp.s_ddd.array() * tfp.s_ddd.array()).matrix().sum();
                // square of diff from target speed
                auto ds = pow((TARGET_SPEED - tfp.s_d[tfp.s_d.size() - 1]),2);

                tfp.cd = KJ * Jp + KT * Ti + KD * pow(tfp.d[tfp.d.size() -1], 2);
                tfp.cv = KJ * Js + KT * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;

                frenet_paths.push_back(tfp);
            }
        }
    }
    return frenet_paths;
}

std::vector<Frenet_path> Frenet_plan::calc_global_paths(std::vector<Frenet_path> &fplist, Spline2D &csp){
    for(Frenet_path &fp : fplist){
        std::vector<double> fpx;
        std::vector<double> fpy;
        for(int i = 0; i < fp.s.size() - 1; ++i){
            auto ixy_pair = csp.calc_position(fp.s(i));
            if(std::isnan(ixy_pair.first)){
                break;
            }
            auto iyaw = csp.calc_yaw(fp.s(i));
            auto di = fp.d(i);
            auto fx = ixy_pair.first + di * cos(iyaw + M_PI / 2.0);
            auto fy = ixy_pair.second + di * sin(iyaw + M_PI / 2.0);
            fpx.push_back(fx);
            fpy.push_back(fy);
        }
        fp.x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpx.data(), fpx.size());
        fp.y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpy.data(), fpy.size());

        // calc yaw and ds
        fp.yaw.resize(fp.x.size());
        fp.ds.resize(fp.x.size());
        for(int i = 0; i < fp.x.size() - 1; ++i){
            auto dx = fp.x(i + 1) - fp.x(i);
            auto dy = fp.y(i + 1) - fp.y(i);
            fp.yaw(i) = atan(dy/dx);
            fp.ds(i) = sqrt(dx*dx + dy*dy);
        }
        fp.yaw(fp.yaw.size() - 1) = fp.yaw(fp.yaw.size() - 2);
        fp.ds(fp.ds.size() - 1) = fp.ds(fp.ds.size() - 2);

        //calc curvature !! fp.c size is smaller than fp.yaw
        fp.c.resize(fp.yaw.size() - 1);
        for(int i = 0; i < fp.yaw.size() - 1; ++i)
            fp.c(i) = (fp.yaw(i + 1) - fp.yaw(i)) / fp.ds(i);
    }
    return fplist;
}

bool Frenet_plan::check_collision(Frenet_path& fp, MatrixXd& ob){
    bool collision = false;
    for(int i = 0; i < ob.rows(); ++i){
        VectorXd d(fp.x.size());
        for(int j = 0; j < fp.x.size() && j < fp.y.size(); ++j){
            d(j) = pow(fp.x(j) - ob(i,0), 2) + pow(fp.y(j) - ob(i, 1), 2);
        }

        for(int i = 0; i < d.size(); ++i) {
            if (d(i) <= ROBOT_RADIUS * ROBOT_RADIUS)
                collision = true;
        }
        if(collision)
            return false;
    }
    return true;

}

std::vector<Frenet_path> Frenet_plan::check_paths(std::vector<Frenet_path> &fplist, MatrixXd &ob){
    std::vector<int> okind;
    for(int i = 0; i < fplist.size(); ++i){
        //Max speed check
        if([&fplist, i](){
            for(int j = 0; j < fplist[i].s_d.size(); ++j){
                if(fplist[i].s_d(j) > MAX_SPEED)
                    return true;
            }
            return false;
        }())
            continue;
        else if([&fplist, i](){
            for(int j = 0; j < fplist[i].s_dd.size(); ++j){
                if(abs(fplist[i].s_dd(j)) > MAX_ACCEL)
                    return true;
            }
            return false;
        }())
            continue;
        else if([&fplist, i](){
            for(int j = 0; j < fplist[i].c.size(); ++j){
                if(abs(fplist[i].c(j)) > MAX_CURVATURE)
                    return true;
            }
            return false;
        }())
            continue;
        else if(!check_collision(fplist[i], ob))
            continue;
        okind.push_back(i);
    }
    std::vector<Frenet_path> res;
    for(auto ind: okind){
        res.push_back(fplist[ind]);
    }
    return res;
}

Frenet_path Frenet_plan::frenet_optimal_planning(Spline2D& csp, double s0, double c_speed, double c_d, double c_d_d, double c_d_dd, MatrixXd &ob){
    auto fplist = this->calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    fplist = this->calc_global_paths(fplist, csp);
    fplist = this->check_paths(fplist, ob);

    // find minimum cost path
    auto mincost = std::numeric_limits<double>::max();
    auto bestpath = Frenet_path();
    for(auto fp: fplist){
        if(mincost >= fp.cf){
            mincost = fp.cf;
            bestpath = fp;
        }
    }
    return bestpath;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D>
Frenet_plan::generate_target_course(VectorXd& x, VectorXd& y){
    auto csp = Spline2D(x, y);
    std::vector<double> rx;
    std::vector<double>ry;
    std::vector<double> ryaw;
    std::vector<double> rk;
    std::cout << csp.s(csp.s.size() - 1) << std::endl;
    for(double i_s = 0; i_s < csp.s(csp.s.size() - 1); i_s += 0.1){
        auto ixy = csp.calc_position(i_s);
        auto ix = ixy.first;
        auto iy = ixy.second;
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(csp.calc_yaw(i_s));
        rk.push_back(csp.calc_curvature(i_s));
    }
    return std::make_tuple(rx, ry, ryaw, rk, csp);
}