#include "follow_line/line_following.h"

//########## CONSTRUCTOR ###############################################################################################
LineFollowingNode::LineFollowingNode(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // Init youBot
    arm_.init(node_handle);
    base_.init(node_handle);
    gripper_.init(node_handle);

    // Init parameters
    node_->param("follow_line/camera_link_offset_z", camera_link_offset_z_, 0.004);
    node_->param("fllow_line/angle_theta", angle_theta_, M_PI);
    node_->param("follow_line/angle_q5", angle_q5_, 0.0);
    node_->param("follow_line/base_offset", base_offset_, -0.05);

    // Close the gripper
    ROS_INFO("The gripper closing...");
    gripper_.setWidth(0);

    // Move arm to search pose
    ROS_INFO("Move to search center...");
    arm_.moveToPose("SEARCH_CENTER");
    arm_.waitForCurrentAction();

    line_pose_sub_ = node_->subscribe("/line_following_coordinates", 1, &LineFollowingNode::LineFollowingExcute, this);
    receive_client_ = node_->serviceClient<std_srvs::Empty>("/datas_received");
    reset_client_ = node_->serviceClient<std_srvs::Empty>("/reset_line_vision");
}

//########## DESTRUCTOR ################################################################################################
LineFollowingNode::~LineFollowingNode()
{

}

//########## ACTION EXCUTE #############################################################################################
void LineFollowingNode::LineFollowingExcute(const geometry_msgs::PoseArray goal)
{
    // Call a service to stop sending the messeges
    receive_client_.call(stop_send_);
    ROS_INFO("Stopping send line pose messeges...");

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

        // Transform the poses to arm_link_0
        geometry_msgs::PoseStamped pose_in, pose_out;
        cartesian_path_.resize(goal.poses.size());

        for(int i = 0; i < goal.poses.size(); i++) {
            pose_in.pose.position.x = goal.poses[i].position.x;
            pose_in.pose.position.y = goal.poses[i].position.y;
            pose_in.pose.position.z = goal.poses[i].position.z - camera_link_offset_z_;
            pose_in.pose.orientation = goal.poses[i].orientation;
            pose_in.header = goal.header;

            try {
                tf_listener_.transformPose("arm_link_0", pose_in, pose_out);
            }
            catch(tf::TransformException ex) {
                ROS_ERROR("Transform error: %s at item: %d", ex.what(), i);
                return;
            }

            cartesian_path_[i].setX(pose_out.pose.position.x);
            cartesian_path_[i].setY(pose_out.pose.position.y);
            cartesian_path_[i].setZ(pose_out.pose.position.z);
            // Theta = k1 * position.x + b1
            //cartesian_path_[i].setTheta(-M_PI*pose_in.pose.position.x + 1.5*M_PI);
            //cartesian_path_[i].setQ5(-M_PI*pose_in.pose.position.y);
            cartesian_path_[i].setTheta(angle_theta_);
            cartesian_path_[i].setQ5(angle_q5_);
	    ROS_INFO("pose: %d: %f, %f, %f, %f, %f", i, cartesian_path_[i][0], cartesian_path_[i][1], cartesian_path_[i][2], cartesian_path_[i][3], cartesian_path_[i][4]); 
        }
    }

    ROS_INFO("Move along path");
    // Move the arm alone a path
    arm_.moveAlongPath(cartesian_path_);
    arm_.waitForCurrentAction();

    ROS_INFO("Done");
    // Move base a little right
    base_.move(0.0, base_offset_, 0.0);

    // Move arm to search pose
    arm_.moveToPose("SEARCH_CENTER");
    arm_.waitForCurrentAction();

    // Do reset
    reset_client_.call(do_reset_);
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

