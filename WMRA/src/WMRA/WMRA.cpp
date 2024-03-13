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

void armRun();
void baseRun();

// 开环控制
void Velcontroller1(double interval, double x, double y, double theta,
                    double xd, double yd, double &vxd, double &vyd, double &uv, double &uw)
{
    static double last_theta_d = 0;
    double Vd = sqrt(vxd * vxd + vyd * vyd);
    double theta_d = atan2(vyd, vxd);

    if (theta_d < 0)
        theta_d += 2 * M_PI;
    double delta_theta = fmod((theta_d - last_theta_d + 3 * M_PI), (2 * M_PI)) - M_PI;
    double Wd = delta_theta / interval;
    last_theta_d = theta_d;

    uv = Vd;
    uw = Wd;
    std::cout << "theta_d: " << theta_d << std::endl;
}

void velPlan1(double &time, double &xd, double &yd, double &vxd, double &vyd)
{
    double para = 0.1;
    xd = para * time;
    yd = 0;
    vxd = para;
    vyd = 0;
}
void velPlan2(double &time, double &xd, double &yd, double &vxd, double &vyd)
{
    double para = PI / 16;
    xd = sin(para * time);
    yd = -1 + cos(para * time);
    vxd = para * cos(para * time);
    vyd = -para * sin(para * time);
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
    double Vd, Wd, xd, yd, theta_d, vxd, vyd, vrd, vld, uv, uw = 0; // plan and control
    double V, W, x, y, theta, vr, vl = 0;                           // reality
    double xd_cal, yd_cal, v_sim, w_sim, vx_sim, vy_sim, theta_sim, theta_sim_d = 0;
    double theta_old;

    // time
    double time = 0.1;     // all time
    int cycle = 100;       // 期望周期
    double interval = 0.1; // 实际周期

    while (ros::ok())
    {
        myfile << "time: " << time << " interval: " << interval << "_\n";
        std::cout << "time: " << time << " interval: " << interval << "_\n";
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();

        // 轨迹
        // velPlan1(time, xd, yd, theta_d);
        velPlan2(time, xd, yd, vxd, vyd);

        // calBaseU
        Velcontroller1(interval, x, y, theta, xd, yd, vxd, vyd, uv, uw);
        // run
        // uv = 0;
        // uw = 0.0;
        wheelChair->run(uv, uw, time);

        wheelChair->getData(V, W, x, y, theta, vr, vl, vrd, vld);
        myfile << " x:" << x << " y:" << y << " theta: " << theta << std::endl;
        myfile << " v:" << V << " w:" << W << std::endl;
        myfile << "实际左轮: " << wheelChair->leftWheelV << "  实际右轮: " << wheelChair->rightWheelV << std::endl;
        myfile << "uv: " << uv << " uw: " << uw << std::endl;
        myfile << "期望左轮: " << vld << "  期望右轮: " << vrd << std::endl;
        // pub des
        v_sim = sqrt(vxd * vxd + vyd * vyd);
        theta_sim_d = atan2(vyd, vxd);
        w_sim = (theta_sim_d - theta_old) / interval;
        theta_old = theta_sim_d;
        theta_sim += w_sim * interval;
        vx_sim = v_sim * cos(theta_sim);
        vy_sim = v_sim * sin(theta_sim);
        xd_cal += vx_sim * interval;
        yd_cal += vy_sim * interval;
        rosReferenceManage->pubBaseData(x, y, theta, xd_cal, yd_cal, theta_d, V, W, vr, vl, uv, uw, vrd, vld);

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
        interval = allTimeInterval.count() / 1000.0 / 1000.0;
        time = time + interval;
        // std::cout << "baseRun Running interval" << interval << " s" << std::endl;
        myfile << "----"
               << "_\n";
    }

    delete wheelChair;
}