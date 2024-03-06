#include "wheelChair/wheelChair.h"
#include "wheelChair/wheelChair.h"
#include "fstream"
const int period_ms = 100;
#define _torqueMode false
ROBOT_MOBILE_BASE::ROBOT_MOBILE_BASE(bool flag)
{
    ros::NodeHandle nh_private("~");

    path_robot_pub = nh_private.advertise<nav_msgs::Path>("trajectory_robot", 1, true);
    nh_private.param<std::string>("robot_frame_id", this->robot_frame_id, "base_link");
    nh_private.param<std::string>("smoother_cmd_vel", this->smoother_cmd_vel, "/cmd_vel");

    nh_private.param<bool>("PubPath", this->mbIsPubPath, true);
    nh_private.param<bool>("PubOdom", this->mbIsPubOdom, true);
    nh_private.param<bool>("PubMarkers", this->mbIsPubMarkers, false);

    // set zero
    oV1 = oV2 = oV3 = oVw = oVx = oVy = 0;
    mV1 = mV2 = mV3 = 0;
    oX = oY = oTheta = 0;

    wheelRadius = 0.315 / 2;
    twoWheelDis = 0.585;
    VL = 0;
    VR = 0;

    sleep(1);
    ros_kvaser = new Kvaser();
    ros_kvaser->canInit(0);
    // connect motor
    ros_kvaser->connectMotor(1);
    ros_kvaser->connectMotor(2);
    sleep(1);
    // Enable motor
    ros_kvaser->motorEnable(1);
    ros_kvaser->motorEnable(2);
    // SPEED MODE mode

    ros_kvaser->modeChoose(1, ros_kvaser->SPEED_MODE);
    ros_kvaser->modeChoose(2, ros_kvaser->SPEED_MODE);

    sleep(1);
    ROS_INFO("Motor init success");
}

ROBOT_MOBILE_BASE::~ROBOT_MOBILE_BASE()
{
    ros_kvaser->motorDisable(1);
    ros_kvaser->motorDisable(2);
    ros_kvaser->motorDisable(3);
    ros_kvaser->canRelease();

    std::ofstream data;
    data.open("/home/lab306/catkin_ws/src/robot_start/trajectory.txt", std::ios::trunc);
    // for (int i = 0; i < odom_path.poses.size(); i++)
    // {
    //     float x = odom_path.poses.at(i).pose.position.x;
    //     float y = odom_path.poses.at(i).pose.position.y;
    //     float z = odom_path.poses.at(i).pose.position.z;
    //     float qx = odom_path.poses.at(i).pose.orientation.x;
    //     float qy = odom_path.poses.at(i).pose.orientation.y;
    //     float qz = odom_path.poses.at(i).pose.orientation.z;
    //     float qw = odom_path.poses.at(i).pose.orientation.w;
    //     data << mTimeStamp.at(i) << " " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
    // }
    data.close();
    ROS_INFO("end");
}

void ROBOT_MOBILE_BASE::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{

    // step1.
    ROS_INFO_STREAM(mV1);
    mVx = twist_aux.linear.x;
    mVw = twist_aux.angular.z;

    ROS_INFO_STREAM(mV1);

    VL = (2 * mVx - twoWheelDis * mVw) / 2 * 32 * 4096 / 2 / PI / wheelRadius;
    if (VL > 0.3 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        VL = 0.3 * 32 * 4096 / 2 / PI / wheelRadius;
    }
    if (VL < -0.3 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        VL = -0.3 * 32 * 4096 / 2 / PI / wheelRadius;
    }

    VR = (2 * mVx + twoWheelDis * mVw) / 2 * 32 * 4096 / 2 / PI / wheelRadius;
    if (VR > 0.3 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        VR = 0.3 * 32 * 4096 / 2 / PI / wheelRadius;
    }
    if (VR < -0.3 * 32 * 4096 / 2 / PI / wheelRadius)
    {
        VR = -0.3 * 32 * 4096 / 2 / PI / wheelRadius;
    }
    ROS_INFO_STREAM(VL);
    ROS_INFO_STREAM(VR);
    ros_kvaser->speedMode(1, VL);
    ros_kvaser->speedMode(2, VR);

    ros_kvaser->beginMovement(1);
    ros_kvaser->beginMovement(2);
}

void ROBOT_MOBILE_BASE::run()
{
    this->current_time = ros::Time::now();
    this->mdt = (current_time - last_time).toSec();
    odom_publisher();
}

void ROBOT_MOBILE_BASE::odom_publisher()
{

    update_odom();
    static tf::TransformBroadcaster odom_broadcaster; // 定义tf对象
    geometry_msgs::TransformStamped odom_trans;       // 创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::Quaternion odom_quat;              // 四元数变量

    // 载入坐标（tf）变换时间戳

    if (mbIsPubOdom)
    {
        nav_msgs::Odometry odom;
        // 载入坐标（tf）变换时间戳
        odom.header.stamp = current_time;
        // 发布坐标变换的父子坐标系
        odom.header.frame_id = "camera_color_frame";
        odom.child_frame_id = "camera_color_frame2";
        // tf位置数据：x,y,z,方向
        odom.pose.pose.position.x = oX;
        odom.pose.pose.position.y = oY;
        odom.pose.pose.position.z = 0.0;

        // ROS_INFO("oTheta:%f,oX:%f,oY:%f",oTheta,oX,oY);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(oTheta);
        //
        //        //first ,we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "camera_color_frame3";
        odom_trans.child_frame_id = this->robot_frame_id;
        //
        odom_trans.transform.translation.x = oX;
        odom_trans.transform.translation.y = oY;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // next,we'll publish the odomtry message over ROS

        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = oVx;
        odom.twist.twist.linear.y = oVy;
        odom.twist.twist.angular.z = oVw;

        memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
        // std::cout << "robot_start:   "<<odom.pose.pose.position.x << odom.pose.pose.position.y  << endl;
        odom_pub.publish(odom);
    }

    // finally, publish path
    if (mbIsPubPath)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = oX;
        pose_stamped.pose.position.y = oY;

        geometry_msgs::Quaternion path_quat = tf::createQuaternionMsgFromYaw(oTheta);
        pose_stamped.pose.orientation.x = path_quat.x;
        pose_stamped.pose.orientation.y = path_quat.y;
        pose_stamped.pose.orientation.z = path_quat.z;
        pose_stamped.pose.orientation.w = path_quat.w;

        pose_stamped.header.stamp = current_time;
        odom_path.header.stamp = current_time;

        // pose_stamped.header.frame_id = "odom_combined";
        // odom_path.header.frame_id = "odom_combined";
        pose_stamped.header.frame_id = "odom";
        odom_path.header.frame_id = "odom";
        odom_path.poses.push_back(pose_stamped);
        path_pubilsher.publish(odom_path);
        size_t t = time(NULL);
        mTimeStamp.push_back(t);
    }
}

void ROBOT_MOBILE_BASE::update_odom()
{
    read_Velocity();
    double mtheta = (oV1 - oV2) / twoWheelDis;

    oX += mdt * ((oV1 + oV2) / 2) * cos(oTheta);
    oY += mdt * ((oV1 + oV2) / 2) * sin(oTheta);
    oTheta += (mdt * mtheta);
    this_pose_stamped.pose.position.x = oX;
    this_pose_stamped.pose.position.y = oY;
    path_robot.poses.push_back(this_pose_stamped);
    path_robot_pub.publish(path_robot);
}

void ROBOT_MOBILE_BASE::read_Velocity()
{
    // oV1 = ros_kvaser->getVelocity(1, 2048, 60)*mR;
    // oV2 = ros_kvaser->getVelocity(2, 2048, 60)*mR*1.0125;
    // oV1 = m_kvaser->getVelocity(1, 1024, 32)*wheelRadius;//一个周期内记录的轮子线位移
    // oV2 = m_kvaser->getVelocity(2, 1024, 32)*wheelRadius;//0.12*0.1*0.1=0.0012
    // std::cout<<"1:"<<oV1<<"\n2:"<<oV2<<"\n3:"<<oV3<<std::endl;
    // ROS_INFO_STREAM(mV1);
    oV1 = mV1;
    oV2 = mV2;
}
