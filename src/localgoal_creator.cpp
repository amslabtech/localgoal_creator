#include "localgoal_creator/localgoal_creator.h"
#include "geometry_msgs/PoseStamped.h"

LocalGoalCreator::LocalGoalCreator() : nh_(),
                                       private_nh_("~")
{

    // std::cout<<"============TEST================="<<std::endl;
    private_nh_.param("hz", hz_, 10);
    private_nh_.param("start_node", start_node_, 0);
    private_nh_.param("local_goal_interval", local_goal_interval_, 1.0);
    private_nh_.param("local_goal_dist", local_goal_dist_, 5.0);

    checkpoint_sub_ = nh_.subscribe("/checkpoint", 1, &LocalGoalCreator::checkpoint_callback, this);
    node_edge_sub_ = nh_.subscribe("/node_edge_map", 1, &LocalGoalCreator::node_edge_map_callback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LocalGoalCreator::current_pose_callback, this);
    local_goal_dist_sub_ = nh_.subscribe("/local_goal_dist", 1, &LocalGoalCreator::local_goal_dist_callback, this);
    reached_checkpoint_flag_sub_ = nh_.subscribe("/reached_checkpoint", 1, &LocalGoalCreator::reached_checkpoint_flag_callback, this);

    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
    current_checkpoint_id_pub_ = nh_.advertise<std_msgs::Int32>("/current_checkpoint", 1);
    next_checkpoint_id_pub_ = nh_.advertise<std_msgs::Int32>("/next_checkpoint", 1);

    checkpoint_received_ = false;
    node_edge_map_received_ = false;
    current_pose_updated_ = false;
    current_checkpoint_id_ = start_node_;
    next_checkpoint_id_ = start_node_;
    local_goal_index_ = 0;
}

void LocalGoalCreator::node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr &msg)
{
    if (!node_edge_map_received_)
    {
        node_edge_map_ = *msg;
        node_edge_map_received_ = true;
    }
}

void LocalGoalCreator::checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (!checkpoint_received_)
    {
        checkpoint_ = *msg;
        const auto init_itr = find(checkpoint_.data.begin(), checkpoint_.data.end(), start_node_);
        next_checkpoint_id_ = *next(init_itr);
        checkpoint_.data.erase(checkpoint_.data.begin(), init_itr+1);
        checkpoint_received_ = true;
    }
}

void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if (!current_pose_updated_)
    {
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        current_pose_updated_ = true;
    }
}

void LocalGoalCreator::reached_checkpoint_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
    reached_checkpoint_flag_ = msg->data;
}

void LocalGoalCreator::local_goal_dist_callback(const std_msgs::Float64::ConstPtr &msg)
{
    local_goal_dist_ = msg->data;
}

int LocalGoalCreator::get_index_from_node_id(int target_id)
{
    const auto target_itr = find(checkpoint_.data.begin(), checkpoint_.data.end(), target_id);
    const int index = distance(checkpoint_.data.begin(), target_itr);
    if (index == checkpoint_.data.size())
        return -1;
    return index;
}

void LocalGoalCreator::update_checkpoint()
{
    const auto itr = checkpoint_.data.begin();
    current_checkpoint_id_ = next_checkpoint_id_;
    next_checkpoint_id_ = *next(itr);
    checkpoint_.data.erase(itr);
    local_goal_index_ = 0;

    ROS_WARN("checkpoint updated");
    ROS_WARN("current_checkpoint_id: %d", current_checkpoint_id_);
    ROS_WARN("next_checkpoint_id: %d", next_checkpoint_id_);
}

void LocalGoalCreator::get_path_to_next_checkpoint()
{
    const int current_node_index = get_index_from_node_id(current_checkpoint_id_);
    const int next_node_index = get_index_from_node_id(next_checkpoint_id_);
    geometry_msgs::Point reference_pose = node_edge_map_.nodes[current_node_index].point;
    const geometry_msgs::Point target_pose = node_edge_map_.nodes[next_node_index].point;
    const double direction = atan2(target_pose.y - reference_pose.y, target_pose.x - reference_pose.x);

    while (true)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = current_pose_.header.frame_id;
        pose.pose.position.x = reference_pose.x + local_goal_interval_ * cos(direction);
        pose.pose.position.y = reference_pose.y + local_goal_interval_ * sin(direction);
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
        path_.poses.push_back(pose);

        const double distance_to_target = hypot(pose.pose.position.x - target_pose.x, pose.pose.position.y - target_pose.y);
        if (distance_to_target < local_goal_interval_)
            break;

        reference_pose = pose.pose.position;
    }
}

void LocalGoalCreator::publish_local_goal()
{
    for (int i=local_goal_index_; i<path_.poses.size(); i++)
    {
        const double goal_pose_x = path_.poses[i].pose.position.x;
        const double goal_pose_y = path_.poses[i].pose.position.y;
        const double distance_to_local_goal = hypot(current_pose_.pose.position.x - goal_pose_x, current_pose_.pose.position.y - goal_pose_y);
        if (distance_to_local_goal < local_goal_dist_)
            local_goal_index_ = i;
    }

    geometry_msgs::PoseStamped local_goal;
    local_goal.header.frame_id = current_pose_.header.frame_id;
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.position.x = path_.poses[local_goal_index_].pose.position.x;
    local_goal.pose.position.y = path_.poses[local_goal_index_].pose.position.y;
    local_goal.pose.position.z = 0;
    local_goal.pose.orientation = path_.poses[local_goal_index_].pose.orientation;

    local_goal_pub_.publish(local_goal);
}

void LocalGoalCreator::publish_checkpoint_id()
{
    std_msgs::Int32 current_checkpoint_id_msg;
    std_msgs::Int32 next_checkpoint_id_msg;
    current_checkpoint_id_msg.data = current_checkpoint_id_;
    current_checkpoint_id_msg.data = next_checkpoint_id_;
    current_checkpoint_id_pub_.publish(current_checkpoint_id_msg);
    current_checkpoint_id_pub_.publish(next_checkpoint_id_msg);
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    while (ros::ok())
    {
        if (checkpoint_received_ && node_edge_map_received_ && current_pose_updated_)
        {
            if (reached_checkpoint_flag_ && !checkpoint_.data.empty())
            {
                update_checkpoint();
                get_path_to_next_checkpoint();
            }
            publish_local_goal();
            publish_checkpoint_id();
        }
        ros::spinOnce();
        rate.sleep();
    }

}
