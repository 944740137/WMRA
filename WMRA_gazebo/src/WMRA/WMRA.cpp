#include "ros/ros.h"
#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    WheelChair wheelChair;
    Manipulator manipulator(false); // 阻塞等待机械臂连接成功

    // d
    double Vd = 0;
    double Wd = 0;
    double V = 0;
    double W = 0;

    Eigen::Matrix<double, 6, 1> q0 = manipulator.getq();
    Eigen::Matrix<double, 6, 1> q = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> q_d = manipulator.getq();
    Eigen::Matrix<double, 6, 1> dq_d = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> tau_feedforward = Eigen::MatrixXd::Zero(6, 1);

    double posPara = 0.4;
    double velPara = 1.5;
    double time = 0;

    while (ros::ok())
    {
        // update +=2ms
        double time = time + cycle / 1000.0; // 单位：秒

        // get
        q = manipulator.getq();
        V = wheelChair.getV();
        W = wheelChair.getW();

        // 轨迹
        q_d[0] = q0[0] + posPara * (1 - cos(velPara * time));
        q_d[1] = q0[1] + posPara * (1 - cos(velPara * time));
        q_d[2] = q0[2] - posPara * (1 - cos(velPara * time));
        q_d[3] = q0[3] - posPara * (1 - cos(velPara * time));
        q_d[4] = q0[4] - posPara * (1 - cos(velPara * time));
        q_d[5] = q0[5] - posPara * (1 - cos(velPara * time));

        // run
        wheelChair.run(Vd, Wd);
        manipulator.run(q_d, dq_d, tau_feedforward);

        ros::spinOnce();
        usleep(cycle);
    }
    
    std::cout << "[------] End Program" << std::endl;
    return 0;
}
