#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"
#include "WMRA/rosReferenceManager.h"

#include "math.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    // case
    WheelChair wheelChair;
    // Manipulator manipulator(false); // 阻塞等待机械臂连接成功
    RosReferenceManager rosReferenceManager(n);

    // vary
    double Vd, Wd, vxd, vyd, V, W, x, y, theta = 0;
    // Eigen::Matrix<double, 6, 1> q0 = manipulator.getq();
    Eigen::Matrix<double, 6, 1> q_d = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> q = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> dq_d = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> tau_feedforward = Eigen::MatrixXd::Zero(6, 1);

    // 轨迹参数
    double posPara = 0.4;
    double velPara = 1.5;

    // time
    double time = 0;
    ros::Rate r(1000 / cycle);

    while (ros::ok())
    {
        // update +=2ms
        time = time + cycle / 1000.0; // 单位：秒

        // get
        // q = manipulator.getq();
        V = wheelChair.getV();
        W = wheelChair.getW();
        x = wheelChair.getX();
        y = wheelChair.getY();
        theta = wheelChair.getTheta();

        // 轨迹
        // xd = -1 + cos(PI / 2 * time);
        // yd = sin(PI / 2 * time);
        vxd = -PI / 4 * sin(PI / 4 * time);
        vyd = PI / 4 * cos(PI / 4 * time);
        Vd = sqrt(vxd * vxd + vyd * vyd);
        Wd = atan2(vyd, vxd);

        // std::cout << "x:" << x << "  y:" << y << std::endl;

        // run
        wheelChair.run(Vd, Wd);
        // manipulator.run(q_d, dq_d, tau_feedforward);
        rosReferenceManager.run(V, W, x, y, theta); // pub

        // sleep
        ros::spinOnce();
        r.sleep();
    }

    std::cout << "[------] End Program" << std::endl;
    return 0;
}
