#include "WMRA/arm.h"

Manipulator::Manipulator(bool hasGripper)
{
    arm = new unitreeArm(hasGripper);
    arm->sendRecvThread->start();
    arm->setFsm(ArmFSMState::PASSIVE);
    arm->setFsm(ArmFSMState::LOWCMD);
    arm->labelRun("forward");
    directions << 0, 0, 0, 0, 0, 0, 0.0;
    // default
    std::vector<double> Kp = arm->_ctrlComp->lowcmd->kp;
    std::vector<double> Kv = arm->_ctrlComp->lowcmd->kd;
    std::cout << "[------] default Kp: ";
    for (const auto &element : Kp)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    std::cout << "[------] default Kv: ";
    for (const auto &element : Kv)
    {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    arm->_ctrlComp->lowcmd->setControlGain(Kp, Kv);
    std::cout << "[------] Manipulator init success" << std::endl;
}
Manipulator::~Manipulator()
{
    arm->backToStart();
    arm->setFsm(ArmFSMState::JOINTCTRL);
    arm->setFsm(ArmFSMState::PASSIVE);
    arm->sendRecvThread->shutdown();

    std::cout << "[------] Manipulator del success" << std::endl;
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
void Manipulator::setCommand(int i, int direction, int command) // 0 stop 1 move
{
    if (command == 0)
        this->directions = Eigen::MatrixXd::Zero(7, 1);
    else
        this->directions[i] = direction;
}
void Manipulator::run()
{
    arm->cartesianCtrlCmd(this->directions, this->angularVel, this->linearVel); // roll, pitch, yaw, x, y, z, gripper
}
void Manipulator::setControlGain(std::vector<double> Kp, std::vector<double> Kv)
{
    if (Kp.size() != 6 || Kv.size() != 6)
    {
        std::cout << "[------] setControlGain error" << std::endl;
        return;
    }
    arm->_ctrlComp->lowcmd->setControlGain(Kp, Kv);
}
Eigen::Matrix<double, 6, 1> Manipulator::getq()
{
    return arm->lowstate->getQ();
}
