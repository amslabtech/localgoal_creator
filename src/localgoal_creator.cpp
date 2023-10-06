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
    is_end_of_path_ = false;
    update_checkpoint_flag_ = false;
    current_checkpoint_id_ = start_node_;
    next_checkpoint_id_ = start_node_;
    local_goal_index_ = 0;
    path_.poses.clear();
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
    if (is_end_of_path_ && msg->data == true)
        update_checkpoint_flag_ = true;
}

void LocalGoalCreator::local_goal_dist_callback(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_WARN("Update local_goal_dist: %lf -> %lf", local_goal_dist_, msg->data);
    local_goal_dist_ = msg->data;
}

int LocalGoalCreator::get_index_from_node_id(int target_id)
{
    int index = 0;
    for (const auto node : node_edge_map_.nodes)
    {
        if (node.id == target_id)
            return index;

        index++;
    }
    return -1;
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
    const double distance_between_nodes = hypot(target_pose.x - reference_pose.x, target_pose.y - reference_pose.y);
    const double direction = atan2(target_pose.y - reference_pose.y, target_pose.x - reference_pose.x);
    path_.poses.clear();

    bool enable_get_path;
    double interval;
    if (distance_between_nodes == 0)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = current_pose_.header.frame_id;
        pose.pose.position.x = target_pose.x;
        pose.pose.position.y = target_pose.y;
        pose.pose.position.z = current_pose_.pose.position.z;
        pose.pose.orientation = current_pose_.pose.orientation;
        path_.poses.push_back(pose);
        enable_get_path = false;
    }
    else if (distance_between_nodes < local_goal_interval_)
    {
        interval = distance_between_nodes;
        enable_get_path = true;
    }
    else
    {
        interval = local_goal_interval_;
        enable_get_path = true;
    }

    while (enable_get_path)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = current_pose_.header.frame_id;
        pose.pose.position.x = reference_pose.x + interval * cos(direction);
        pose.pose.position.y = reference_pose.y + interval * sin(direction);
        pose.pose.position.z = current_pose_.pose.position.z;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
        path_.poses.push_back(pose);

        const double distance_to_target = hypot(pose.pose.position.x - target_pose.x, pose.pose.position.y - target_pose.y);
        enable_get_path = (interval < distance_to_target);

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

    is_end_of_path_ = (local_goal_index_ == path_.poses.size()-1);

    geometry_msgs::PoseStamped local_goal;
    local_goal.header.frame_id = current_pose_.header.frame_id;
    local_goal.header.stamp = ros::Time::now();
    local_goal.pose.position.x = path_.poses[local_goal_index_].pose.position.x;
    local_goal.pose.position.y = path_.poses[local_goal_index_].pose.position.y;
    local_goal.pose.position.z = current_pose_.pose.position.z;
    local_goal.pose.orientation = path_.poses[local_goal_index_].pose.orientation;

    local_goal_pub_.publish(local_goal);
}

void LocalGoalCreator::publish_checkpoint_id()
{
    std_msgs::Int32 current_checkpoint_id_msg;
    std_msgs::Int32 next_checkpoint_id_msg;
    current_checkpoint_id_msg.data = current_checkpoint_id_;
    next_checkpoint_id_msg.data = next_checkpoint_id_;
    current_checkpoint_id_pub_.publish(current_checkpoint_id_msg);
    next_checkpoint_id_pub_.publish(next_checkpoint_id_msg);
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    while (ros::ok())
    {
        if (checkpoint_received_ && node_edge_map_received_ && current_pose_updated_)
        {
            if (path_.poses.empty() && !checkpoint_.data.empty())
            {
                get_path_to_next_checkpoint();
            }
            if (update_checkpoint_flag_ && !checkpoint_.data.empty())
            {
                update_checkpoint();
                get_path_to_next_checkpoint();
                update_checkpoint_flag_ = false;
            }
            publish_local_goal();
            publish_checkpoint_id();
            current_pose_updated_ = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

}
