#include "ros/ros.h"
#include "WMRA/base.h"
#include "WMRA/arm.h"
#include "kvaser/kvaser.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "math.h"

void pub(const ros::Publisher &odomPublisher, ros::Publisher &pathPublisher, tf::TransformBroadcaster &odomBroadcaster,
         nav_msgs::Path &path, double &v, double &w, double &x, double &y, double &theta);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "WMRA");
    ros::NodeHandle n;

    WheelChair wheelChair;
    // Manipulator manipulator(false); // 阻塞等待机械臂连接成功

    // pub
    ros::Publisher odomPublisher = n.advertise<nav_msgs::Odometry>("/WMRA/odom", 10);
    ros::Publisher pathPublisher = n.advertise<nav_msgs::Path>("/WMRA/trajectory", 1, true);
    tf::TransformBroadcaster odomBroadcaster;
    nav_msgs::Path path;
    ros::Time currentTime = ros::Time::now();
    path.header.stamp = currentTime;
    path.header.frame_id = "odom";

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
        Vd = 0.05;
        Wd = 0;

        std::cout << "x:" << x << "  y:" << y << std::endl;

        // run
        wheelChair.run(Vd, Wd);
        // manipulator.run(q_d, dq_d, tau_feedforward);

        // pub
        pub(odomPublisher, pathPublisher, odomBroadcaster, path, V, W, x, y, theta);

        //
        ros::spinOnce();
        r.sleep();
    }

    std::cout << "[------] End Program" << std::endl;
    return 0;
}
void pub(const ros::Publisher &odomPublisher, ros::Publisher &pathPublisher, tf::TransformBroadcaster &odomBroadcaster,
         nav_msgs::Path &path, double &v, double &w, double &x, double &y, double &theta)
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

    // path
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.stamp = currentTime;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;
    this_pose_stamped.pose.position.z = 0.0;
    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;
    this_pose_stamped.header.stamp = currentTime;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);
    pathPublisher.publish(path);
}

// #include <ros/ros.h>
// #include <ros/console.h>
// #include <nav_msgs/Path.h>
// #include <std_msgs/String.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>

// main (int argc, char **argv)
// {
//     ros::init (argc, argv, "showpath");

//     ros::NodeHandle ph;
//     ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);

//     ros::Time current_time, last_time;
//     current_time = ros::Time::now();
//     last_time = ros::Time::now();

//     nav_msgs::Path path;
//     //nav_msgs::Path path;
//     path.header.stamp=current_time;
//     path.header.frame_id="odom";

//     double x = 0.0;
//     double y = 0.0;
//     double th = 0.0;
//     double vx = 0.1;
//     double vy = -0.1;
//     double vth = 0.1;

//     ros::Rate loop_rate(1);
//     while (ros::ok())
//     {

//         current_time = ros::Time::now();
//         //compute odometry in a typical way given the velocities of the robot
//         double dt = 0.1;
//         double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
//         double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
//         double delta_th = vth * dt;

//         x += delta_x;
//         y += delta_y;
//         th += delta_th;

//         geometry_msgs::PoseStamped this_pose_stamped;
//         this_pose_stamped.pose.position.x = x;
//         this_pose_stamped.pose.position.y = y;

//         geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
//         this_pose_stamped.pose.orientation.x = goal_quat.x;
//         this_pose_stamped.pose.orientation.y = goal_quat.y;
//         this_pose_stamped.pose.orientation.z = goal_quat.z;
//         this_pose_stamped.pose.orientation.w = goal_quat.w;

//         this_pose_stamped.header.stamp=current_time;
//         this_pose_stamped.header.frame_id="odom";
//         path.poses.push_back(this_pose_stamped);

//         path_pub.publish(path);
//         ros::spinOnce();               // check for incoming messages

//         last_time = current_time;
//         loop_rate.sleep();
//     }

//     return 0;
// }
