#include "WMRA/base.h"

#include "fstream"
#include "math.h"
#include <unistd.h>

WheelChair::WheelChair()
{
    this->kvaserInterface = new Kvaser();
    this->kvaserInterface->canInit(0);

    // connect motor
    this->kvaserInterface->connectMotor(1);
    this->kvaserInterface->connectMotor(2);
    sleep(1);

    // Enable motor
    this->kvaserInterface->motorEnable(1);
    this->kvaserInterface->motorEnable(2);

    // SPEED MODE mode
    this->kvaserInterface->modeChoose(1, this->kvaserInterface->SPEED_MODE);
    this->kvaserInterface->modeChoose(2, this->kvaserInterface->SPEED_MODE);

    sleep(1);
    std::cout << "[------] WheelChair init success" << std::endl;
}

WheelChair::~WheelChair()
{
    this->kvaserInterface->motorDisable(1);
    this->kvaserInterface->motorDisable(2);
    this->kvaserInterface->motorDisable(3);
    this->kvaserInterface->canRelease();

    std::ofstream data;
    data.open("/home/lab306/catkin_ws/src/robot_start/trajectory.txt", std::ios::trunc);
    // for (int i = 0; i < odom_path.poses.size(); i++)
    // {
    //     float x = odom_path.poses.at(i).pose.position.x;
    //     float y = odom_path.poses.at(i).pose.position.y;
    //     float z = odom_path.poses.at(i).pose.position.z;
    //     float qx = odom_path.poses.at(i).pose.orientation.x;
    //     float qy = odom_path.poses.at(i).pose.orientation.y;
    //     float qz = odom_path.poses.at(i).pose.orientation.z;
    //     float qw = odom_path.poses.at(i).pose.orientation.w;
    //     data << mTimeStamp.at(i) << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
    // }
    data.close();
}

void WheelChair::setCommand(double Vd, double Wd)
{
    if (Vd == this->Vd && Wd == this->Wd)
    {
        if (Vd != 0 || Wd != 0)
            this->updateFlag = true;
        return;
    }

    this->updateFlag = true;

    this->Vd = Vd;
    this->Wd = Wd;

    this->leftWheelVd = (2 * this->Vd - this->twoWheelDis * this->Wd) / 2 * 32 * 4096 / (2 * PI) / this->wheelRadius;
    if (this->leftWheelVd > 0.3 * 32 * 4096 / 2 / PI / wheelRadius)
        this->leftWheelVd = 0.3 * 32 * 4096 / 2 / PI / wheelRadius;

    if (this->leftWheelVd < -0.3 * 32 * 4096 / 2 / PI / wheelRadius)
        this->leftWheelVd = -0.3 * 32 * 4096 / 2 / PI / wheelRadius;

    this->rightWheelVd = (2 * this->Vd + this->twoWheelDis * this->Wd) / 2 * 32 * 4096 / (2 * PI) / this->wheelRadius;
    if (this->rightWheelVd > 0.3 * 32 * 4096 / 2 / PI / wheelRadius)
        this->rightWheelVd = 0.3 * 32 * 4096 / 2 / PI / wheelRadius;

    if (this->rightWheelVd < -0.3 * 32 * 4096 / 2 / PI / wheelRadius)
        this->rightWheelVd = -0.3 * 32 * 4096 / 2 / PI / wheelRadius;
    std::cout << "leftWheelVd:" << this->leftWheelVd << "  rightWheelVd:" << this->rightWheelVd << std::endl;

    // 发送
    this->kvaserInterface->speedMode(1, this->leftWheelVd);
    this->kvaserInterface->speedMode(2, this->rightWheelVd);

    this->kvaserInterface->beginMovement(1);
    this->kvaserInterface->beginMovement(2);
}

void WheelChair::updateData()
{
    if (!this->updateFlag)
        return;

    // 读取轮子线速度
    this->rightWheelV = this->kvaserInterface->getVelocity(1, 1024, 32) * this->wheelRadius;
    this->leftWheelV = this->kvaserInterface->getVelocity(1, 1024, 32) * this->wheelRadius;

    // V W
    this->V = (this->leftWheelV + this->rightWheelV) / 2;
    this->W = (this->rightWheelV - this->leftWheelV) / this->twoWheelDis;

    // 状态
    this->theta = this->theta + this->W * cycle;
    if (this->theta < -PI)
        this->theta += 2 * PI;
    if (this->theta >= PI)
        this->theta -= 2 * PI;

    this->x += cycle / 1000.0 * this->V * cos(this->theta);
    this->y += cycle / 1000.0 * this->V * sin(this->theta);
    std::cout << " V: " << this->V << " W: " << this->W << " x" << this->x << " y: " << this->y << std::endl;
    this->updateFlag = false; // 运动了则更新标志位记录新里程数据
}

void WheelChair::run(double Vd, double Wd)
{
    this->setCommand(Vd, Wd);
    this->updateData();
}