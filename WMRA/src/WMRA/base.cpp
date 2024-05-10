#include "WMRA/base.h"

#include "fstream"
#include "math.h"
#include <unistd.h>
#include <iostream>

extern std::ofstream myfile;

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
    std::cout << "[------] WheelChair del success" << std::endl;
}

void WheelChair::setCommand(double Vd, double Wd)
{
    // if (Vd == this->Vd && Wd == this->Wd)
    // {
    //     if (Vd != 0 || Wd != 0)
    //         this->updateFlag = true;
    //     return;
    // }

    this->updateFlag = true;

    this->Vd = Vd;
    this->Wd = Wd;

    this->leftWheelVd = (2 * this->Vd - this->twoWheelDis * this->Wd) / 2.0;
    this->leftWheelV_count_d = this->leftWheelVd * 32 * 4096 / (2 * PI) / this->wheelRadius;
    if (this->leftWheelV_count_d > 0.4 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        std::cout << "[------] leftWheelVd limit!!!" << std::endl;
        this->leftWheelV_count_d = 0.4 * 32 * 4096 / 2 / PI / wheelRadius;
    }
    if (this->leftWheelV_count_d < -0.4 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        std::cout << "[------] leftWheelVd limit!!!" << std::endl;
        this->leftWheelV_count_d = -0.4 * 32 * 4096 / 2 / PI / wheelRadius;
    }

    this->rightWheelVd = (2 * this->Vd + this->twoWheelDis * this->Wd) / 2.0;
    this->rightWheelV_count_d = this->rightWheelVd * 32 * 4096 / (2 * PI) / this->wheelRadius;
    if (rightWheelV_count_d > 0.4 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        std::cout << "[------] rightWheelVd limit!!!" << std::endl;
        rightWheelV_count_d = 0.4 * 32 * 4096 / 2 / PI / wheelRadius;
    }
    if (rightWheelV_count_d < -0.4 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        std::cout << "[------] rightWheelVd limit!!!" << std::endl;
        rightWheelV_count_d = -0.4 * 32 * 4096 / 2 / PI / wheelRadius;
    }

    // 发送
    this->kvaserInterface->speedMode(1, this->leftWheelV_count_d);
    this->kvaserInterface->speedMode(2, this->rightWheelV_count_d);
}

void WheelChair::updateData(double timeInterval)
{
    // if (!this->updateFlag)
    //     return;

    // 读取轮子线速度
    this->leftWheelV = this->kvaserInterface->getVelocity(1, 1024, 32) * this->wheelRadius;
    this->rightWheelV = this->kvaserInterface->getVelocity(2, 1024, 32) * this->wheelRadius;

    // V W
    this->V = (this->leftWheelV + this->rightWheelV) / 2.0;
    this->W = (this->rightWheelV - this->leftWheelV) / this->twoWheelDis;

    // 状态
    this->theta += this->W * timeInterval;
    if (this->theta < -PI)
        this->theta += 2 * PI;
    if (this->theta >= PI)
        this->theta -= 2 * PI;

    this->x += timeInterval * this->V * cos(this->theta);
    this->y += timeInterval * this->V * sin(this->theta);

    this->updateFlag = false; // 运动了则更新标志位记录新里程数据
}

void WheelChair::run(double Vd, double Wd, double nowtime)
{
    static double lastTime = 0;
    double timeInterval = nowtime - lastTime;
    this->setCommand(Vd, Wd);
    this->updateData(timeInterval);
    lastTime = nowtime;
}
void WheelChair::setCommand(double vd_direction, double wd_direction, double move)
{
    this->vd_direction = vd_direction;
    this->wd_direction = wd_direction;
    this->move = move;
}
void WheelChair::run(double nowtime)
{
    static double lastTime = 0;
    double timeInterval = nowtime - lastTime;
    this->setCommand(this->key_vd * this->vd_direction * this->move, this->key_wd * this->wd_direction * this->move);
    this->updateData(timeInterval);
    lastTime = nowtime;
}
void WheelChair::getData(double &v, double &w, double &x, double &y, double &theta,
                         double &vr, double &vl, double &vrd, double &vld)
{
    v = this->V;
    w = this->W;
    x = this->x;
    y = this->y;
    theta = this->theta;
    vr = this->rightWheelV;
    vl = this->leftWheelV;
    vrd = this->rightWheelVd;
    vld = this->leftWheelVd;
}
