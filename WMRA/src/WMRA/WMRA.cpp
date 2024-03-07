#include "ros/ros.h"
#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "math.h"

void pub(const ros::Publisher &odomPublisher, tf::TransformBroadcaster &odomBroadcaster,
         double &v, double &w, double &x, double &y, double &theta);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    WheelChair wheelChair;
    // Manipulator manipulator(false); // 阻塞等待机械臂连接成功

    ros::Publisher odomPublisher = n.advertise<nav_msgs::Odometry>("/WMRA/odom", 10);
    tf::TransformBroadcaster odomBroadcaster;

    //
    double Vd = 0;
    double Wd = 0;
    double vxd = 0;
    double vyd = 0;
    double V = 0;
    double W = 0;
    double x = 0;
    double y = 0;
    double theta = 0;

    // Eigen::Matrix<double, 6, 1> q0 = manipulator.getq();
    Eigen::Matrix<double, 6, 1> q_d = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> q = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> dq_d = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> tau_feedforward = Eigen::MatrixXd::Zero(6, 1);

    // 轨迹
    double posPara = 0.4;
    double velPara = 1.5;

    // time
    double time = 0;
    ros::Rate r(1000 / cycle);

    while (ros::ok())
    {
        // update +=2ms
        double time = time + cycle / 1000.0; // 单位：秒

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
        vxd = PI / 2 * sin(PI / 2 * time);
        vyd = PI / 2 * cos(PI / 2 * time);
        Vd = sqrt(vxd * vxd + vyd * vyd);
        Wd = atan2(vyd, vxd);

        // run
        wheelChair.run(Vd, Wd);
        // manipulator.run(q_d, dq_d, tau_feedforward);

        // pub
        pub(odomPublisher, odomBroadcaster, V, W, x, y, theta);
        //
        ros::spinOnce();
        r.sleep();
    }

    std::cout << "[------] End Program" << std::endl;
    return 0;
}
void pub(const ros::Publisher &odomPublisher, tf::TransformBroadcaster &odomBroadcaster,
         double &v, double &w, double &x, double &y, double &theta)
{
    ros::Time currentTime = ros::Time::now();
    // odom_trans
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odomBroadcaster.sendTransform(odom_trans);
    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v * cos(theta);
    odom.twist.twist.linear.y = v * sin(theta);
    odom.twist.twist.angular.z = w;
    odomPublisher.publish(odom);
}