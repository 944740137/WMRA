#ifndef _BASE_H_
#define _BASE_H_

#include "kvaser/kvaser.h"

// const double cycle = 2; // ms

class WheelChair
{
public:
    WheelChair();
    ~WheelChair();

    void run(double Vd, double Wd, double nowtime);
    void setCommand(double Vd, double Wd);
    void updateData(double timeInterval);
    void getData(double &v, double &w, double &x, double &y, double &theta,
                 double &vr, double &vl, double &vrd, double &vld);
    double getV() { return V; }
    double getW() { return W; }
    double getX() { return x; }
    double getY() { return y; }
    double getTheta() { return theta; }
    double getRightWheelV() { return rightWheelV; }
    double getLeftWheelV() { return leftWheelV; }

public:
    Kvaser *kvaserInterface;
    bool updateFlag = false;

    // model
    const double twoWheelDis = 0.585;
    const double wheelRadius = 0.1575;

    // v w
    double Vd = 0;
    double Wd = 0;
    double V;
    double W;

    // 里程计算
    double x = 0;
    double y = 0;
    double theta = 0; // rad

    // 轮子线速度
    double rightWheelVd;
    double leftWheelVd;
    double rightWheelV;
    double leftWheelV;

    double leftWheelV_count_d;
    double rightWheelV_count_d;
    double leftWheelV_count;
    double rightWheelV_count;
};

#endif //_MOBILEBASW_H_