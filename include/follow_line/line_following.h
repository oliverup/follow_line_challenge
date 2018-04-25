#ifndef FOLLOW_LINE_LINE_FOLLOWING_H
#define FOLLOW_LINE_LINE_FOLLOWING_H

#include <ros/ros.h>
#include <luh_youbot_controller_api/controller_api.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <signal.h>
#include <sstream>
#include <vector>

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

    // youbot
    youbot_api::YoubotArm arm_;
    youbot_api::YoubotBase base_;
    youbot_api::YoubotGripper gripper_;

    // transform listener
    tf::TransformListener tf_listener_;

    void stopAction();
};
#endif // FOLLOW_LINE_LINE_FOLLOWING_H

