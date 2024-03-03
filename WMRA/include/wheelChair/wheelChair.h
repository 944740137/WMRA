#ifndef _MOBILEBASW_H_
#define _MOBILEBASW_H_

#include "ros/ros.h"
#include "iostream"
#include "string"
#include "string.h"
#include "math.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"


#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "kvaser.h"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "tf/transform_broadcaster.h"
#include "iostream"
#include "fstream"
#include "vector"
#include "time.h"
#include "visualization_msgs/Marker.h"

#include <control_toolbox/pid.h>

#include <math.h>

#include "math.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <nav_msgs/Path.h>


//#define SPEEDX0 8192*60/2/PI/0.063
#define SPEEDX0 1232198.75249

#define _torqueMode true

const double odom_pose_covariance[36] = {5e-4, 0, 0, 0, 0, 0,
                                         0, 5e-4, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                          0, 1e-3, 1e-9, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                          0, 1e-3, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0,
                                           0, 1e-3, 1e-9, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e-9};



class ROBOT_MOBILE_BASE
{
public:
    ROBOT_MOBILE_BASE(bool flag);
    ~ROBOT_MOBILE_BASE();
    void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
    void run();
    void odom_publisher();
    void update_odom() ;
    void read_Velocity();

public:  //参数
       //world velocity                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    float mVx,mVy,mVw;
    float mV1,mV2,mV3;
    float mTheta;

      //robot model
    float mL=0.26;
    float mR=0.0631;

    //odom
    float oV1,oV2,oV3;
    float oVx,oVy,oVw;
    float oX,oY,oTheta;
    float mRealTheta;

    float mdt;
    Eigen::Matrix3d calib_matrix;
    ros::Subscriber sub_z1_robot_state;
private:
    std::string robot_frame_id,smoother_cmd_vel;
    bool mbIsPubOdom, mbIsPubPath, mbIsPubMarkers;

    ros::NodeHandle n;
    
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    ros::Publisher path_pubilsher;
    ros::Publisher markers_pub;

    ros::Publisher joint_state_pubtorviz;
    nav_msgs::Path odom_path;

    Kvaser *ros_kvaser;

    std::vector<size_t> mTimeStamp;

    ros::Time current_time,last_time;
    
    sensor_msgs::JointState joint_msg;     

    double wheelRadius;
    double twoWheelDis;
    double VL;
    double VR;
    geometry_msgs::PoseStamped this_pose_stamped;
    nav_msgs::Path path_robot;
    ros::Publisher path_robot_pub;
};
#endif  //_MOBILEBASW_H_