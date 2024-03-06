#include "WMRA/arm.h"

Manipulator::Manipulator(bool hasGripper)
{
    arm = new unitreeArm(hasGripper);
    arm->sendRecvThread->start();
    arm->setFsm(ArmFSMState::PASSIVE);
    arm->setFsm(ArmFSMState::LOWCMD);
    arm->sendRecvThread->shutdown();

    // default
    std::vector<double> Kp = arm->_ctrlComp->lowcmd->kp;
    std::vector<double> Kv = arm->_ctrlComp->lowcmd->kd;
    std::cout << "default Kp: ";
    for (const auto &element : Kp)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    std::cout << "default Kv: ";
    for (const auto &element : Kv)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    arm->_ctrlComp->lowcmd->setControlGain(Kp, Kv);
    ROS_INFO("Manipulator init success");
}
Manipulator::~Manipulator()
{
    arm->setFsm(ArmFSMState::JOINTCTRL);
    arm->setFsm(ArmFSMState::PASSIVE);
}
void Manipulator::setCommand(const Eigen::Matrix<double, 6, 1> &q_d,
                             const Eigen::Matrix<double, 6, 1> &dq_d,
                             const Eigen::Matrix<double, 6, 1> &tau)
{
    arm->setArmCmd(q_d, dq_d, tau);
}

void Manipulator::run(const Eigen::Matrix<double, 6, 1> &q_d,
                      const Eigen::Matrix<double, 6, 1> &dq_d,
                      const Eigen::Matrix<double, 6, 1> &tau)
{
    this->setCommand(q_d, dq_d, tau);
    arm->sendRecv();
}

void Manipulator::setControlGain(std::vector<double> Kp, std::vector<double> Kv)
{
    if (Kp.size() != 6 || Kv.size() != 6)
    {
        std::cout << "setControlGain error" << std::endl;
        return;
    }
    arm->_ctrlComp->lowcmd->setControlGain(Kp, Kv);
}
Eigen::Matrix<double, 6, 1> Manipulator::getq()
{
    return arm->lowstate->getQ();
}
// Eigen::Matrix<double, 6, 1> Manipulator::getdq()
// {
// }