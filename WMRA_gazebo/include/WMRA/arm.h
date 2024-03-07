#ifndef _ARM_H_
#define _ARM_H_

#include <ros/ros.h>
#include <ros/console.h>

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
    void setCommand(const Eigen::Matrix<double, 6, 1> &q_d,
                    const Eigen::Matrix<double, 6, 1> &dq_d,
                    const Eigen::Matrix<double, 6, 1> &tau);
    // void updateData();

    void setControlGain(std::vector<double> Kp, std::vector<double> Kv);
    Eigen::Matrix<double, 6, 1> getq();
    // const Eigen::Matrix<double, 6, 1> &getdq();

private:
    unitreeArm *arm;
};

#endif //_MOBILEBASW_H_