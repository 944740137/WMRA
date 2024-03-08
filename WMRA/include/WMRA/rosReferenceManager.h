#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class RosReferenceManager
{
public:
    RosReferenceManager() = delete;
    RosReferenceManager(ros::NodeHandle &n);
    ~RosReferenceManager();

    void run(double &v, double &w, double &x, double &y, double &theta);

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
    
};