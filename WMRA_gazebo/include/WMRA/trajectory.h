#include <Eigen/Dense>
#include <iostream>
#define DIM 6
void JointCosTrajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                        const Eigen::Matrix<double, DIM, 1> &q0, const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                        Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d, Eigen::Matrix<double, DIM, 1> &ddq_d,
                        Eigen::Matrix<double, DIM, 1> &qerror, Eigen::Matrix<double, DIM, 1> &dqerror);