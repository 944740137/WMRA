#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = true;
    unitreeArm arm(hasGripper);
    arm.sendRecvThread->start();
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;
    // std::cout << "KP:" << KP.size() << "\n";
    // std::cout << "KW:" << KW.size() << "\n";
    for (int i = 0; i < KP.size(); i++)
    {
        std::cout << "i :" << i << "kp: " << KP[i] << "\n";
        std::cout << "i :" << i << "kw: " << KW[i] << "\n";
        // KP[i] = 0.0;
        // KW[i] = 0.0;
    }
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    arm.sendRecvThread->shutdown();

    Vec6 initQ = arm.lowstate->getQ();

    double duration = 1000;
    Vec6 targetQ;
    targetQ << 0.0, 1.5, -1, -0.54, 0, 0;
    // targetQ = initQ; // wd//wd

    Timer timer(arm._ctrlComp->dt);
    for (int i(0); i < duration; i++)
    {
        arm.q = initQ * (1 - i / duration) + targetQ * (i / duration);

        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
        arm.qd.setZero(); // wd

        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau.setZero(); // wd

        arm.gripperQ = -(i / duration);
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();

        // std::cout << "tau:" << arm.tau.transpose() << "\n";
        // std::cout << "tau:" << arm.q.transpose() << "\n";
        // std::cout << "q:" << arm.lowstate->getQ().transpose()  << "\n";
        // std::cout << "dq:" << arm.lowstate->getQd().transpose()  << "\n";
    }
    std::cout << "OVER \n";

    arm.sendRecvThread->start();

    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}