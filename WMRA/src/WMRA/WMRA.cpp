#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"
#include "WMRA/rosReferenceManager.h"

#include "math.h"

#include <chrono>

#include <thread>
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

void armRun();
void baseRun();

// 开环控制
void Velcontroller1(double x, double y, double theta, double xd, double yd, double vxd, double vyd, double &uv, double &uw)
{
    double Vd = sqrt(vxd * vxd + vyd * vyd);
    double Wd = atan2(vyd, vxd);
    uv = Vd;
    uw = Wd;
}
// 控制1
void Velcontroller2(double x, double y, double theta, double xd, double yd, double theta_d, double vxd, double vyd, double &uv, double &uw)
{
    double k1 = 4;
    double k2 = 500;
    double k3 = 3;

    double x_e = cos(theta) * (xd - x) + sin(theta) * (yd - y);
    double y_e = -sin(theta) * (xd - x) + cos(theta) * (yd - y);
    double theta_e = theta_d - theta;

    double Vd = sqrt(vxd * vxd + vyd * vyd);
    double Wd = atan2(vyd, vxd);
    uv = Vd * cos(theta_e) + k1 * x_e;
    uw = Wd + k2 * Vd * y_e + k3 * sin(theta_e);
}

void velPlan1(double &time, double &xd, double &yd, double &vxd, double &vyd, double &theta_d)
{
    double para = PI / 1024;
    xd = 1 - cos(para * time);
    yd = sin(para * time);
    theta_d = atan2(yd, xd);
    vxd = para * sin(para * time);
    vyd = para * cos(para * time);
}

Manipulator *manipulator;
WheelChair *wheelChair;
RosReferenceManager *rosReferenceManage;

int main(int argc, char *argv[])
{
    std::cout << "[------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------] 编译时刻:" << __TIME__ << "\n";

    // ros
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

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

void baseRun()
{
    std::cout << "[------] baseRun" << std::endl;
    wheelChair = new WheelChair();

    // vary
    double Vd, Wd, xd, yd, theta_d, vxd, vyd, uv, uw = 0; // plan and control
    double V, W, x, y, theta = 0;                         // reality

    // time
    double time = 0; // 1000microSec
    int cycle = 100; // ms
    while (ros::ok())
    {
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();

        // get
        wheelChair->getData(V, W, x, y, theta);

        // 轨迹
        velPlan1(time, xd, yd, vxd, vyd, theta_d);

        // calBaseU
        // Velcontroller1(x, y, theta, xd, yd, vxd, vyd, uv, uw);
        Velcontroller2(x, y, theta, xd, yd, theta_d, vxd, vyd, uv, uw);
        std::cout << "uv: " << uv << "uw: " << uw << std::endl;
        // run
        // uv = 0;
        // uw = 0.5;
        wheelChair->run(uv, uw);

        // pub
        rosReferenceManage->pubBaseData(V, W, x, y, theta, xd, yd, theta_d);
        // sleep

        high_resolution_clock::time_point comEndTime = high_resolution_clock::now();
        auto comTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(comEndTime - beginTime);
        // std::cout << "baseRun Running Time：" << comTimeInterval.count() << "微秒" << std::endl;

        int sleepTime = cycle * 1000 - comTimeInterval.count();
        if (sleepTime < 0)
            sleepTime = 0;
        usleep(sleepTime);

        high_resolution_clock::time_point endTime = high_resolution_clock::now();
        auto allTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(endTime - beginTime);
        // std::cout << "baseRun Running Time：" << allTimeInterval.count() << "微秒" << std::endl;
        time = time + allTimeInterval.count() / 1000.0 / 1000.0;
        // std::cout << "baseRun Running Time：" << time << " s" << std::endl;
    }

    delete wheelChair;
}