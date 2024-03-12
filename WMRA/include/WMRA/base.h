#ifndef _BASE_H_
#define _BASE_H_

#include "kvaser/kvaser.h"

const double cycle = 2; // ms

class WheelChair
{
public:
    WheelChair();
    ~WheelChair();

    void run(double Vd, double Wd);
    void setCommand(double Vd, double Wd);
    void updateData();
    void getData(double &v, double &w, double &x, double &y, double &theta);
    double getV() { return V; }
    double getW() { return W; }
    double getX() { return x; }
    double getY() { return y; }
    double getTheta() { return theta; }
    double getRightWheelV() { return rightWheelV; }
    double getLeftWheelV() { return leftWheelV; }

private:
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
};

#endif //_MOBILEBASW_H_