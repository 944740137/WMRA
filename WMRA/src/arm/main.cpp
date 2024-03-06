#include "arm/control/unitreeArm.h"

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
    std::cout << "initQ :" << initQ.transpose() << "\n";

    double duration = 1000;
    Vec6 targetQ;
    targetQ << 0.0, 1.5, -1, -0.54, 0, 0;
    // targetQ = initQ; // wd//wd

    Timer timer(arm._ctrlComp->dt);
    double i = 0;
    double posPara = 0.4;
    double velPara = 1.5;

    while (true)
    {
        arm.q[0] = initQ[0] + posPara * (1 - cos(velPara * i / 500.0));
        arm.q[1] = initQ[1] + posPara * (1 - cos(velPara * i / 500.0));
        arm.q[2] = initQ[2] - posPara * (1 - cos(velPara * i / 500.0));
        arm.q[3] = initQ[3] - posPara * (1 - cos(velPara * i / 500.0));
        arm.q[4] = initQ[4] - posPara * (1 - cos(velPara * i / 500.0));
        arm.q[5] = initQ[5] - posPara * (1 - cos(velPara * i / 500.0));

        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);

        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());

        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.sendRecv();
        timer.sleep();
        i = i + 1;
        // if (i < duration)
        //     i++;
    }

    std::cout << "OVER \n";

    arm.sendRecvThread->start();
    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    return 0;
}