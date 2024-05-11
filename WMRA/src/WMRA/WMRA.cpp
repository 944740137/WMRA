#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"
#include "WMRA/rosReferenceManager.h"
#include "math.h"
#include <chrono>
#include <thread>
#include <fstream>
#include <fcntl.h>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

bool runFlag = true;

std::ofstream myfile;
Manipulator *manipulator;
WheelChair *wheelChair;
RosReferenceManager *rosReferenceManage;
enum Command
{
    stop = 0,
    move = 1
};
enum DIM
{
    roll = 0,
    pitch = 1,
    yaw = 2,
    x = 3,
    y = 4,
    z = 5
};

void armRun();
void baseRun();
void KeyboardTask();

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
void velPlan3(double &time, double &xd, double &yd, double &theta_d, double &vxd, double &vyd)
{
    double a = 0.4;
    double vmax = 0.4;
    vxd = 0;
    if (time >= 0 && time <= 1)
        vxd = a * time;
    if (time >= 1 && time <= 2.5)
        vxd = vmax;
    if (time >= 2.5 && time <= 3.5)
        vxd = vmax - a * (time - 2.5);
    vyd = 0;
    theta_d = atan2(vyd, vxd);
}

int main(int argc, char *argv[])
{
    std::cout << "[------] 编译日期:" << __DATE__ << "\n";
    std::cout << "[------] 编译时刻:" << __TIME__ << "\n";

    // ros
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    // myfile.open("/home/wd/workSpace/WMRA/WMRA_ROS/recordWMRA.txt");
    // myfile << "WMRA" << std::endl;

    // case
    rosReferenceManage = new RosReferenceManager(n);

    // run
    std::thread threadKeyArmRun(armRun);
    std::thread threadBaseRun(baseRun);
    threadKeyArmRun.detach();
    threadBaseRun.detach();

    KeyboardTask();
    // END

    std::cout << "[------] End Program" << std::endl;

    delete wheelChair;
    delete manipulator;
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
    while (runFlag)
    {
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();

        // 轨迹
        // velPlan1(time, xd, yd, theta_d, vxd, vyd);
        // velPlan2(time, xd, yd, theta_d, vxd, vyd);

        // Velcontroller1(interval, x, y, theta, xd, yd, vxd, vyd, uv, uw);

        // run
        // wheelChair->run(uv, uw, time);
        wheelChair->run(time);
        // simBase(interval, uv, uw, xd_sim, yd_sim, theta_sim);

        // update
        wheelChair->getData(v, w, x, y, theta, vr, vl, vrd, vld);

        // ros pub record
        // rosReferenceManage->pubBaseData(x, y, theta, xd_sim, yd_sim, theta_sim, V, W, vr, vl, uv, uw, vrd, vld);// 仿真
        rosReferenceManage->pubBaseData(x, y, theta, xd, yd, theta_d, v, w, vr, vl, uv, uw, vrd, vld); // 期望
        record(time, x, y, theta, xd, yd, theta_d, v, w, vd, wd, vl, vr, vld, vrd);

        // Time
        high_resolution_clock::time_point comEndTime = high_resolution_clock::now();
        auto comTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(comEndTime - beginTime);
        int sleepTime = cycle * 1000 - comTimeInterval.count();

        if (sleepTime < 0)
            sleepTime = 0;

        usleep(sleepTime);
        high_resolution_clock::time_point endTime = high_resolution_clock::now();
        auto allTimeInterval = std::chrono::duration_cast<std::chrono::microseconds>(endTime - beginTime);
        interval = allTimeInterval.count() / 1000.0 / 1000.0;
        time = time + interval;
    }
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

    Eigen::Matrix<double, 6, 1> targetQ;
    targetQ << 0.0, 1.5, -1, -0.54, 0, 0;
    q0 = manipulator->getq();

    // time
    double time = 0;
    double cycle = 0.002; // 2ms

    while (runFlag)
    {
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();
        // update +=2ms
        time = time + cycle; // 单位：秒

        // // 轨迹

        // run
        // manipulator->run(q_d, dq_d, tau_feedforward);
        manipulator->run();
        // rosReferenceManager->pubArmData(V, W, x, y, theta); // pub

        usleep(2 * 1000);
    }
}

void KeyboardTask()
{
    std::cout << "[------] KeyboardTask" << std::endl;
    sleep(2);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    // 设置终端为非规范模式，禁用回显和缓冲
    new_tio = old_tio;
    new_tio = old_tio;            //
    new_tio.c_lflag &= (~ICANON); //
    new_tio.c_cc[VTIME] = 0;
    new_tio.c_cc[VMIN] = 1;

    int attr = 1;
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    char nowInput = 0;
    char lastInput = 0;
    bool isPress = false;
    int count = 0;
    while (runFlag)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        if (read(STDIN_FILENO, &nowInput, 1) == 1)
        {
            if (nowInput == 'q')
            {
                runFlag = false;
                return;
            }
            switch (nowInput)
            {
            case '8':
                if (wheelChair != nullptr)
                    wheelChair->setCommand(1, 0, 1);
                break;
            case '2':
                if (wheelChair != nullptr)
                    wheelChair->setCommand(-1, 0, 1);
                break;
            case '4':
                if (wheelChair != nullptr)
                    wheelChair->setCommand(0, 1, 1);
                break;
            case '6':
                if (wheelChair != nullptr)
                    wheelChair->setCommand(0, -1, 1);
                break;
                // arm xyz
            case 'w':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::x, 1, Command::move);
                break;
            case 's':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::x, -1, Command::move);
                break;
            case 'a':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::y, 1, Command::move);
                break;
            case 'd':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::y, -1, Command::move);
                break;
            case 'r':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::z, 1, Command::move);
                break;
            case 'f':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::z, -1, Command::move);
                break;
                // arm rpy
            case 'u':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::roll, 1, Command::move);
                break;
            case 'j':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::roll, -1, Command::move);
                break;
            case 'i':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::pitch, 1, Command::move);
                break;
            case 'k':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::pitch, -1, Command::move);
                break;
            case 'o':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::yaw, 1, Command::move);
                break;
            case 'l':
                if (manipulator != nullptr)
                    manipulator->setCommand(DIM::yaw, -1, Command::move);
                break;
            default:
                break;
            }
            isPress = true;
            lastInput = nowInput;
            count = 0;
        }
        else
        {
            count++;
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        if (isPress && count >= 50)
        {
            std::cout << "松开" << std::endl;
            if (manipulator != nullptr)
                manipulator->setCommand(0, -1, Command::stop);
            if (wheelChair != nullptr)
                wheelChair->setCommand(0, 0, 0);
            isPress = false;
            count = 0;
        }
        nowInput = 0;
        usleep(1000);
    }
}
void record(double time,
            double x, double y, double theta,
            double xd, double yd, double thetad,
            double v, double w, double vd, double wd,
            double vl, double vr, double vld, double vrd)
{
    // myfile << " time:" << time << "\n";
    // myfile << " x: " << x << " |y: " << y << " |theta: " << theta << "\n";
    // myfile << " xd: " << xd << " |yd: " << yd << " |thetad: " << thetad << "\n";
    // myfile << " v: " << v << " |w: " << w << "\n";
    // myfile << " vd: " << vd << " |wd: " << wd << "\n";
    // myfile << " vl: " << vl << " |vr: " << vr << "\n";
    // myfile << " vld: " << vld << " |wrd: " << vrd << "\n";
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

    wd = delta_theta / interval; //
    last_theta_d = theta_d;
}