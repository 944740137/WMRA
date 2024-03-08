#include "WMRA/rosReferenceManager.h"

RosReferenceManager::RosReferenceManager(ros::NodeHandle &n)
{
    // odomTrans

    // odom
    this->odomPublisher = n.advertise<nav_msgs::Odometry>("/WMRA/odom", 10);

    // path
    this->pathPublisher = n.advertise<nav_msgs::Path>("/WMRA/trajectory", 1, true);
    ros::Time currentTime = ros::Time::now();
    path.header.stamp = currentTime;
    path.header.frame_id = "world";

    std::cout << "[------] RosReferenceManager init success" << std::endl;
}
RosReferenceManager::~RosReferenceManager()
{
}

void RosReferenceManager::run(double &v, double &w, double &x, double &y, double &theta)
{
    ros::Time currentTime = ros::Time::now();
    // odomTrans
    geometry_msgs::Quaternion odomQuaternion = tf::createQuaternionMsgFromYaw(theta);
    this->odomTrans.header.stamp = currentTime;
    this->odomTrans.header.frame_id = "world";
    this->odomTrans.child_frame_id = "WMRA";
    this->odomTrans.transform.translation.x = x;
    this->odomTrans.transform.translation.y = y;
    this->odomTrans.transform.translation.z = 0.0;
    this->odomTrans.transform.rotation = odomQuaternion;
    this->odomBroadcaster.sendTransform(this->odomTrans);

    // odom
    this->odom.header.stamp = currentTime;
    this->odom.header.frame_id = "world";
    this->odom.child_frame_id = "WMRA";
    this->odom.pose.pose.position.x = x;
    this->odom.pose.pose.position.y = y;
    this->odom.pose.pose.position.z = 0.0;
    this->odom.pose.pose.orientation = odomQuaternion;
    this->odom.twist.twist.linear.x = v * cos(theta);
    this->odom.twist.twist.linear.y = v * sin(theta);
    this->odom.twist.twist.angular.z = w;
    this->odomPublisher.publish(this->odom);

    // path
    this->poseStamped.header.stamp = currentTime;
    this->poseStamped.header.frame_id = "world";
    this->poseStamped.pose.position.x = x;
    this->poseStamped.pose.position.y = y;
    this->poseStamped.pose.position.z = 0.0;
    this->poseStamped.pose.orientation = odomQuaternion;
    this->path.poses.push_back(this->poseStamped);
    this->pathPublisher.publish(this->path);
}