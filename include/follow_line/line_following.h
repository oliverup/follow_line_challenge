#ifndef FOLLOW_LINE_LINE_FOLLOWING_H
#define FOLLOW_LINE_LINE_FOLLOWING_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <luh_youbot_controller_api/controller_api.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <vector>

namespace ykin = luh_youbot_kinematics;
namespace yapi = youbot_api;

class LineFollowingNode
{
public:
    LineFollowingNode(ros::NodeHandle &node_handle);
    ~LineFollowingNode();

    // excute function
    void LineFollowingExcute(const geometry_msgs::PoseArray pose);

private:
    // ros node handle
    ros::NodeHandle *node_;

    // ros subscriber
    ros::Subscriber line_pose_sub_;

    // ros service
    ros::ServiceClient receive_client_;
    ros::ServiceClient reset_client_;

    // parameters
    std_srvs::Empty stop_send_;
    std_srvs::Empty do_reset_;
    double camera_link_offset_z_;
    double angle_theta_;
    double angle_q5_;
    double base_offset_;

    // youbot
    youbot_api::YoubotArm arm_;
    youbot_api::YoubotBase base_;
    youbot_api::YoubotGripper gripper_;

    // transform listener
    tf::TransformListener tf_listener_;

    // Line path
    std::vector<ykin::CartesianPosition> cartesian_path_;
};
#endif // FOLLOW_LINE_LINE_FOLLOWING_H

