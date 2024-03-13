#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"
#include "WMRA/rosReferenceManager.h"
#include "math.h"
#include <chrono>
#include <thread>
#include <fstream>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

std::ofstream myfile;
Manipulator *manipulator;
WheelChair *wheelChair;
RosReferenceManager *rosReferenceManage;

void armRun();
void baseRun();
void calDesiredVW(double interval, double &vxd, double &vyd, double &vd, double &wd);
void simBase(double interval, double &vd, double &wd,
             double &x_sim, double &y_sim, double &theta_sim);
void record(double time,
            double x, double y, double theta,
            double xd, double yd, double theta_d,
            double v, double w, double vd, double wd,
            double vl, double vr, double vld, double vrd);

void Velcontroller1(double interval, double x, double y, double theta,
                    double xd, double yd, double &vxd, double &vyd,
                    double &uv, double &uw)
{
    double vd = 0;
    double wd = 0;
    calDesiredVW(interval, vxd, vyd, vd, wd);
    uv = vd;
    uw = wd;
}
void velPlan1(double &time, double &xd, double &yd, double &theta_d, double &vxd, double &vyd)
{
    double para = 0.1;
    xd = para * time;
    yd = 0;
    vxd = para;
    vyd = 0;
    theta_d = atan2(vyd, vxd);
}
void velPlan2(double &time, double &xd, double &yd, double &theta_d, double &vxd, double &vyd)
{
    double para = PI / 16;
    xd = sin(para * time);
    yd = -1 + cos(para * time);
    vxd = para * cos(para * time);
    vyd = -para * sin(para * time);
    theta_d = atan2(vyd, vxd);
}

int main(int argc, char *argv[])
{
    std::cout << "[------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------] 编译时刻:" << __TIME__ << "\n";

    // ros
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    myfile.open("/home/wd/workSpace/WMRA/WMRA_ROS/recordWMRA.txt");
    myfile << "WMRA" << std::endl;

    // case
    rosReferenceManage = new RosReferenceManager(n);

    // run
    std::thread threadObj(baseRun);
    armRun();

    // END
    threadObj.join();
    std::cout << "[------] End Program" << std::endl;
    return 0;
}

void baseRun()
{
    std::cout << "[------] baseRun" << std::endl;
    wheelChair = new WheelChair();

    // vary
    double vd, wd, xd, yd, theta_d, vxd, vyd, vrd, vld, uv, uw = 0; // plan and control
    double v, w, x, y, theta, vr, vl = 0;                           // reality
    double xd_sim, yd_sim, theta_sim = 0;                           // sim

    // time
    double time = 0.1;     // all time
    int cycle = 100;       // 期望周期
    double interval = 0.1; // 实际周期
    while (ros::ok())
    {
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();

        // 轨迹
        velPlan1(time, xd, yd, theta_d, vxd, vyd);
        // velPlan2(time, xd, yd, theta_d, vxd, vyd);
        // calBaseU
        Velcontroller1(interval, x, y, theta, xd, yd, vxd, vyd, uv, uw);

        // run
        // uv = 0;
        // uw = 0.0;
        // send and read
        wheelChair->run(uv, uw, time);
        simBase(interval, uv, uw, xd_sim, yd_sim, theta_sim);

        // update
        wheelChair->getData(v, w, x, y, theta, vr, vl, vrd, vld);

        // ros pub
        // rosReferenceManage->pubBaseData(x, y, theta, xd_sim, yd_sim, theta_sim, V, W, vr, vl, uv, uw, vrd, vld);// 仿真
        rosReferenceManage->pubBaseData(x, y, theta, xd, yd, theta_d, v, w, vr, vl, uv, uw, vrd, vld); // 期望
        record(time, x, y, theta, xd, yd, theta_d, v, w, vd, wd, vl, vr, vld, vrd);

        // Time
        high_resolution_clock::time_point comEndTime = high_resolution_clock::now();
        auto comTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(comEndTime - beginTime);
        // std::cout << "baseRun com Time：" << comTimeInterval.count() << "微秒" << std::endl;
        int sleepTime = cycle * 1000 - comTimeInterval.count();
        if (sleepTime < 0)
            sleepTime = 0;
        usleep(sleepTime);
        high_resolution_clock::time_point endTime = high_resolution_clock::now();
        auto allTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(endTime - beginTime);
        // std::cout << "baseRun all Time：" << allTimeInterval.count() << "微秒" << std::endl;
        interval = allTimeInterval.count() / 1000.0 / 1000.0;
        time = time + interval;
        // std::cout << "interval: " << interval << " s" << std::endl;
        myfile << "----_\n";
    }
    delete wheelChair;
}
void record(double time,
            double x, double y, double theta,
            double xd, double yd, double thetad,
            double v, double w, double vd, double wd,
            double vl, double vr, double vld, double vrd)
{
    myfile << " time:" << time << "\n";
    myfile << " x: " << x << " |y: " << y << " |theta: " << theta << "\n";
    myfile << " xd: " << xd << " |yd: " << yd << " |thetad: " << thetad << "\n";
    myfile << " v: " << v << " |w: " << w << "\n";
    myfile << " vd: " << vd << " |wd: " << wd << "\n";
    // myfile << " vl: " << vl << " |vr: " << vr << "\n";
    // myfile << " vld: " << vld << " |wrd: " << vrd << "\n";
}

void armRun()
{
    std::cout << "[------] armRun" << std::endl;
    manipulator = new Manipulator(false);

    // // vary
    Eigen::Matrix<double, 6, 1> q0, q_d, dq_d, tau_feedforward = Eigen::MatrixXd::Zero(6, 1); // plan and control
    Eigen::Matrix<double, 6, 1> q = Eigen::MatrixXd::Zero(6, 1);                              // reality

    // // 轨迹参数
    double posPara = 0.4;
    double velPara = 1.5;

    // time
    double time = 0;

    while (ros::ok())
    {
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();

        // // update +=2ms
        // time = time + cycle / 1000.0; // 单位：秒
        // // get
        // q = manipulator->getq();
        // // 轨迹

        // // run
        // manipulator->run(q_d, dq_d, tau_feedforward);
        // // rosReferenceManager->pubArmData(V, W, x, y, theta); // pub
        // // sleep
        usleep(2 * 1000);

        high_resolution_clock::time_point endTime = high_resolution_clock::now();
        auto timeInterval = std::chrono::duration_cast<std::chrono::microseconds>(endTime - beginTime);
        std::cout << "armRun Running Time：" << timeInterval.count() << "微秒" << std::endl;
    }
    delete manipulator;
}

void simBase(double interval, double &vd, double &wd, double &x_sim, double &y_sim, double &theta_sim)
{
    theta_sim += wd * interval;
    double vx_sim = vd * cos(theta_sim);
    double vy_sim = vd * sin(theta_sim);
    x_sim += vx_sim * interval;
    y_sim += vy_sim * interval;
}
void calDesiredVW(double interval, double &vxd, double &vyd, double &vd, double &wd)
{
    static double last_theta_d = 0;
    vd = sqrt(vxd * vxd + vyd * vyd); //

    double theta_d = atan2(vyd, vxd);
    if (theta_d < 0)
        theta_d += 2 * M_PI;
    double delta_theta = fmod((theta_d - last_theta_d + 3 * M_PI), (2 * M_PI)) - M_PI;
    std::cout << "delta_theta: " << delta_theta << " delta_theta: " << delta_theta << "_\n";

    wd = delta_theta / interval; //
    last_theta_d = theta_d;
}