#include "WMRA/trajectory.h"

void JointCosTrajectory(Eigen::Matrix<double, DIM, 1> &selectAxis, double nowTime, double posRatio, double velRatio,
                        const Eigen::Matrix<double, DIM, 1> &q0,
                        const Eigen::Matrix<double, DIM, 1> &q, const Eigen::Matrix<double, DIM, 1> &dq,
                        Eigen::Matrix<double, DIM, 1> &q_d, Eigen::Matrix<double, DIM, 1> &dq_d)
{
    static double maxPos = M_PI / 4 * 0.5; // 单向
    static double maxVel = M_PI / 2 * 0.5; // 单向
    static Eigen::Matrix<double, DIM, 1> deltaAngle;
    static Eigen::Matrix<double, DIM, 1> dDeltaAngle;
    static Eigen::Matrix<double, DIM, 1> ddDeltaAngle;
    for (size_t i = 0; i < DIM; i++)
    {
        if (selectAxis[i] != 0)
            selectAxis[i] = 1;

        deltaAngle[i] = selectAxis[i] * maxPos * (1 - std::cos(maxVel * velRatio * nowTime)) * posRatio;
        dDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (std::sin((maxVel * velRatio) * nowTime)) * posRatio;
        ddDeltaAngle[i] = selectAxis[i] * maxPos * (maxVel * velRatio) * (maxVel * velRatio) * (std::cos((maxVel * velRatio) * nowTime)) * posRatio;
    }
    q_d = q0 + deltaAngle;
    dq_d = dDeltaAngle;

}