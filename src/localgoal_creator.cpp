#include "localgoal_creator/localgoal_creator.h"

LocalGoalCreator::LocalGoalCreator() :
    local_nh_("~"), checkpoint_received_(false), node_edge_map_received_(false), current_pose_updated_(false),
    is_end_of_path_(false), update_checkpoint_flag_(false), available_skip_mode_(false), local_goal_index_(0)
{
    local_nh_.param<int>("hz", hz_, 10);
    local_nh_.param<int>("start_node", start_node_, 0);
    local_nh_.param<double>("local_goal_interval", local_goal_interval_, 1.0);
    local_nh_.param<double>("local_goal_dist", local_goal_dist_, 5.0);
    local_nh_.param<double>("limit_for_skip", limit_for_skip_, 180.0);

    checkpoint_sub_ = nh_.subscribe("/checkpoint", 1, &LocalGoalCreator::checkpoint_callback, this);
    node_edge_sub_ = nh_.subscribe("/node_edge_map", 1, &LocalGoalCreator::node_edge_map_callback, this);
    current_pose_sub_ = nh_.subscribe("/current_pose", 1, &LocalGoalCreator::current_pose_callback, this);
    local_goal_dist_sub_ = nh_.subscribe("/local_goal_dist", 1, &LocalGoalCreator::local_goal_dist_callback, this);

    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
    edge_pub_ = local_nh_.advertise<amsl_navigation_msgs::Edge>("/edge", 1);
    path_pub_ = local_nh_.advertise<nav_msgs::Path>("path", 1);

    skip_mode_flag_server_ = local_nh_.advertiseService("skip_mode/avaliable", &LocalGoalCreator::skip_mode_flag_callback, this);
    update_flag_server_ = local_nh_.advertiseService("update", &LocalGoalCreator::update_flag_callback, this);
    task_stop_client_ = nh_.serviceClient<std_srvs::SetBool>("/task/stop");

    current_checkpoint_id_ = start_node_;
    next_checkpoint_id_ = start_node_;
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

bool LocalGoalCreator::skip_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    available_skip_mode_ = req.data;
    res.success = true;
    if (available_skip_mode_)
        res.message = "Skip mode is available";
    else
        res.message = "Skip mode is not available";
    return true;
}

bool LocalGoalCreator::update_flag_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    update_checkpoint_flag_ = true;
    res.success = true;
    res.message = "Checkpoint will be updated";
    return true;
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
    if (checkpoint_.data.size() > 1)
        next_checkpoint_id_ = *next(itr);
    else
        call_task_stop();
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
    geometry_msgs::Point reference_point = node_edge_map_.nodes[current_node_index].point;
    const geometry_msgs::Point target_point = node_edge_map_.nodes[next_node_index].point;
    const double distance_between_nodes = hypot(target_point.x - reference_point.x, target_point.y - reference_point.y);
    const double direction = atan2(target_point.y - reference_point.y, target_point.x - reference_point.x);
    path_.poses.clear();

    bool enable_get_path;
    double interval;
    if (distance_between_nodes < DBL_EPSILON)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = current_pose_.header.frame_id;
        pose.pose.position.x = target_point.x;
        pose.pose.position.y = target_point.y;
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
        pose.pose.position.x = reference_point.x + interval * cos(direction);
        pose.pose.position.y = reference_point.y + interval * sin(direction);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
        path_.poses.push_back(pose);

        const double distance_to_target = hypot(pose.pose.position.x - target_point.x, pose.pose.position.y - target_point.y);
        enable_get_path = (interval < distance_to_target);

        reference_point = pose.pose.position;
    }
}

void LocalGoalCreator::publish_local_goal(geometry_msgs::Point point)
{
    geometry_msgs::Point goal_point = path_.poses[local_goal_index_].pose.position;
    double distance_to_local_goal = hypot(goal_point.x - point.x, goal_point.y - point.y);

    while(distance_to_local_goal < local_goal_dist_)
    {
        local_goal_index_++;
        goal_point = path_.poses[local_goal_index_].pose.position;
        distance_to_local_goal = hypot(goal_point.x - point.x, goal_point.y - point.y);

        if(local_goal_index_ == path_.poses.size())
        {
            local_goal_index_ = path_.poses.size()-1;
            break;
        }
    }

    is_end_of_path_ = (local_goal_index_ == path_.poses.size()-1);

    geometry_msgs::PoseStamped local_goal_msg;
    local_goal_msg.header.frame_id = current_pose_.header.frame_id;
    local_goal_msg.header.stamp = ros::Time::now();
    local_goal_msg.pose.position = path_.poses[local_goal_index_].pose.position;
    local_goal_msg.pose.position.z = current_pose_.pose.position.z;
    local_goal_msg.pose.orientation = path_.poses[local_goal_index_].pose.orientation;

    local_goal_pub_.publish(local_goal_msg);
}

void LocalGoalCreator::publish_edge()
{
    amsl_navigation_msgs::Edge edge_msg;
    edge_msg.node0_id = current_checkpoint_id_;
    edge_msg.node1_id = next_checkpoint_id_;
    edge_pub_.publish(edge_msg);
}

void LocalGoalCreator::publish_path()
{
    nav_msgs::Path path_msg;
    path_msg = path_;
    for (auto &p: path_msg.poses)
    {
        p.pose.position.z = current_pose_.pose.position.z;
        p.header.stamp = ros::Time::now();
    }
    path_msg.header.frame_id = current_pose_.header.frame_id;
    path_msg.header.stamp = ros::Time::now();
    path_pub_.publish(path_msg);
}

void LocalGoalCreator::call_task_stop()
{
    std_srvs::SetBool srv;
    srv.request.data = true;
    while (ros::ok() && !task_stop_client_.call(srv))
    {
        ROS_WARN("Failed to call task stop service");
        ros::Duration(0.5).sleep();
    }
    ROS_WARN_STREAM(srv.response.message);
}

void LocalGoalCreator::process()
{
    ros::Rate rate(hz_);
    ros::Time begin_for_skip;
    bool begin_flag = false;

    while (ros::ok())
    {
        if (checkpoint_received_ && node_edge_map_received_ && current_pose_updated_)
        {
            if (begin_flag)
            {
                const int elapsed_time = (ros::Time::now() - begin_for_skip).sec;
                ROS_WARN_STREAM_THROTTLE(1.0, "Countdown for skip: " << limit_for_skip_ - elapsed_time << " s");
                if (limit_for_skip_ < elapsed_time)
                {
                    update_checkpoint_flag_ = true;
                    begin_flag = false;
                }
            }

            if (path_.poses.empty() && !checkpoint_.data.empty())
            {
                get_path_to_next_checkpoint();
                const int current_node_index = get_index_from_node_id(current_checkpoint_id_);
                geometry_msgs::Point reference_point = node_edge_map_.nodes[current_node_index].point;
                publish_local_goal(reference_point);
                begin_flag = false;
            }
            else if (update_checkpoint_flag_ && !checkpoint_.data.empty())
            {
                update_checkpoint();
                get_path_to_next_checkpoint();
                const int current_node_index = get_index_from_node_id(current_checkpoint_id_);
                geometry_msgs::Point reference_point = node_edge_map_.nodes[current_node_index].point;
                publish_local_goal(reference_point);
                begin_flag = false;
                update_checkpoint_flag_ = false;
            }

            publish_local_goal(current_pose_.pose.position);
            publish_edge();
            publish_path();
            current_pose_updated_ = false;

            const int next_node_index = get_index_from_node_id(next_checkpoint_id_);
            const geometry_msgs::Point next_node_point = node_edge_map_.nodes[next_node_index].point;
            const geometry_msgs::Point current_point = current_pose_.pose.position;
            const double dist_to_node = hypot(next_node_point.x - current_point.x, next_node_point.y - current_point.y);
            if (available_skip_mode_ && !begin_flag && dist_to_node < local_goal_dist_)
            {
                begin_for_skip = ros::Time::now();
                begin_flag = true;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

}
