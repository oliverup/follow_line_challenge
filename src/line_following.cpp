#include "follow_line/line_following.h"

namespace ykin = luh_youbot_kinematics;
namespace yapi = youbot_api;

//########## CONSTRUCTOR ###############################################################################################
LineFollowingNode::LineFollowingNode(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // Init youBot
    arm_.init(node_handle);
    base_.init(node_handle);
    gripper_.init(node_handle);

    line_pose_sub_ = node_->subscribe("line_following_coordinates", 1000, &LineFollowingNode::LineFollowingExcute, this);
}

//########## DESTRUCTOR ################################################################################################
LineFollowingNode::~LineFollowingNode()
{

}

//########## ACTION SERVER EXCUTE ######################################################################################
void LineFollowingNode::LineFollowingExcute(const geometry_msgs::PoseArray goal)
{
    // Open the gripper
    gripper_.open();
    gripper_.waitForCurrentAction();

    // Close the gripper
    gripper_.setWidth(0);
    gripper_.waitForCurrentAction();

    // Wait for transform
    bool transform_is_available = tf_listener_.waitForTransform("arm_link_0", goal.header.frame_id,
                                                                goal.header.stamp, ros::Duration(2.0));


    // Check, if transform is available
    if(!transform_is_available) {
        ROS_ERROR("Transform from %s to arm_link_0 is not available.", goal.header.frame_id.c_str());
        return;
    }
    else {
        ROS_INFO("Transform from %s to arm_link_0.", goal.header.frame_id.c_str());
    }

    // Transform the pose to arm_link_0
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    std::vector<ykin::CartesianPosition> cartesian_path;
    cartesian_path.resize(goal.poses.size());

    for(int i = 0; i < goal.poses.size(); i++) {
        pose_in.pose = goal.poses[i];
        pose_in.header.frame_id = "camera_link";
        pose_in.header.stamp = ros::Time::now();

        try {
            tf_listener_.transformPose("arm_link_0", pose_in, pose_out);
        }
        catch(tf::TransformException ex) {
            ROS_ERROR("Transform error: %s at item: %d", ex.what(), i);
            return;
        }

    cartesian_path[i].setX(pose_out.pose.position.x);
    cartesian_path[i].setY(pose_out.pose.position.y);
    cartesian_path[i].setZ(pose_out.pose.position.z);
    // Theta = k1 * position.x + b1
    cartesian_path[i].setTheta(-M_PI*pose_in.pose.position.x + 1.5*M_PI);
    cartesian_path[i].setQ5(M_PI*pose_in.pose.position.y);
    }

    // Move the arm alone a path
    arm_.moveAlongPath(cartesian_path);
    arm_.waitForCurrentAction();
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_following");

    ros::NodeHandle node_handle;
    LineFollowingNode line_following(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
