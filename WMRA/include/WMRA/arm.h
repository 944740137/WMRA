#ifndef _ARM_H_
#define _ARM_H_

#include "arm/control/unitreeArm.h"

using namespace UNITREE_ARM;

class Manipulator
{
public:
    Manipulator() = delete;
    Manipulator(bool hasGripper);
    ~Manipulator();

    void run(const Eigen::Matrix<double, 6, 1> &q_d,
             const Eigen::Matrix<double, 6, 1> &dq_d,
             const Eigen::Matrix<double, 6, 1> &tau);
    void run();
    void setCommand(const Eigen::Matrix<double, 6, 1> &q_d,
                    const Eigen::Matrix<double, 6, 1> &dq_d,
                    const Eigen::Matrix<double, 6, 1> &tau);
    void setCommand(int i, int direction, int command);

    void setControlGain(std::vector<double> Kp, std::vector<double> Kv);
    Eigen::Matrix<double, 6, 1> getq();

private:
    unitreeArm *arm;
    Eigen::Matrix<double, 7, 1> directions;
    double angularVel = 0.08;
    double linearVel = 0.08;
};

#endif //_MOBILEBASW_H_