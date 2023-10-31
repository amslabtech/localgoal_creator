#ifndef __LOCAL_GOAL_CREATOR_H__
#define __LOCAL_GOAL_CREATOR_H__

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
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
        void reached_checkpoint_flag_callback(const std_msgs::Bool::ConstPtr& msg);
        void skip_mode_flag_callback(const std_msgs::Bool::ConstPtr& msg);

        // other functions
        int get_index_from_node_id(int target_id);
        void update_checkpoint();
        void get_path_to_next_checkpoint();
        void publish_local_goal(geometry_msgs::Point point);
        void publish_checkpoint_id();
        void publish_path();

        // private params
        int hz_;
        int start_node_;
        double local_goal_interval_;
        double local_goal_dist_;
        double limit_for_skip_;

        // other params
        bool enable_skip_mode_;
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
        ros::Subscriber reached_checkpoint_flag_sub_;
        ros::Subscriber local_goal_dist_sub_;
        ros::Subscriber skip_mode_flag_sub_;
        ros::Publisher local_goal_pub_;
        ros::Publisher current_checkpoint_id_pub_;
        ros::Publisher next_checkpoint_id_pub_;
        ros::Publisher path_pub_;
};

#endif // __LOCAL_GOAL_CREATOR_H__
