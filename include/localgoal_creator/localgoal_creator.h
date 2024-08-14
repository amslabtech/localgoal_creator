#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        // callbacks
        void checkpoint_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr& msg);
        void current_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void local_goal_dist_callback(const std_msgs::Float64::ConstPtr& msg);
        bool skip_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool restore_mode_flag_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool update_flag_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // other functions
        int get_index_from_node_id(int target_id);
        void update_checkpoint();
        void get_path_to_next_checkpoint();
        void publish_local_goal(geometry_msgs::Point point);
        void publish_edge();
        void publish_path();
        void call_task_stop();

        // private params
        int hz_;
        int start_node_;
        double local_goal_interval_;
        double local_goal_dist_;
        double limit_for_skip_;

        // other params
        bool available_skip_mode_;
        bool restore_mode_flag_;
        bool checkpoint_received_;
        bool node_edge_map_received_;
        bool current_pose_updated_;
        bool is_end_of_path_;
        bool update_checkpoint_flag_;
        int current_checkpoint_id_;
        int next_checkpoint_id_;
        int local_goal_index_;
        nav_msgs::Path path_;
        std_msgs::Int32MultiArray checkpoint_;
        amsl_navigation_msgs::NodeEdgeMap node_edge_map_;
        geometry_msgs::PoseStamped current_pose_;

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Subscriber checkpoint_sub_;
        ros::Subscriber node_edge_sub_;
        ros::Subscriber current_pose_sub_;
        ros::Subscriber local_goal_dist_sub_;
        ros::Publisher local_goal_pub_;
        ros::Publisher edge_pub_;
        ros::Publisher path_pub_;
        ros::ServiceServer skip_mode_flag_server_;
        ros::ServiceServer restore_mode_flag_server_;
        ros::ServiceServer update_flag_server_;
        ros::ServiceClient task_stop_client_;
};

#endif // __LOCAL_GOAL_CREATOR_H__
