#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <WMRA/paramForDebug.h>
class RosReferenceManager
{
public:
    RosReferenceManager() = delete;
    RosReferenceManager(ros::NodeHandle &n);
    ~RosReferenceManager();

    void pubBaseData(double &x, double &y, double &theta,
                     double &x_d, double &y_d, double &theta_d,
                     double &v, double &w, double &vr, double &vl,
                     double &uv, double &uw, double &vrd, double &vld);

private:
    // odom_trans
    tf::TransformBroadcaster odomBroadcaster;
    geometry_msgs::TransformStamped odomTrans;

    // odom
    ros::Publisher odomPublisher;
    nav_msgs::Odometry odom;

    // path
    ros::Publisher pathPublisher;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped poseStamped;

    ros::Publisher pathPublisher_d;
    nav_msgs::Path path_d;
    geometry_msgs::PoseStamped poseStamped_d;

    ros::Publisher velPublisher;
    WMRA::paramForDebug velparam;
};