#pragma once

#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <atomic>

namespace haptiquad_nav {

class HaptiQuadNavPlanner : public nav_core::BaseLocalPlanner {
public:
    HaptiQuadNavPlanner();
    ~HaptiQuadNavPlanner() override = default;

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    bool isInitialized() { return initialized_; }

private:
    void speedLimitCb(const std_msgs::Float64::ConstPtr& msg);

    bool initialized_;
    bool goal_reached_;

    tf2_ros::Buffer*            tf_;
    costmap_2d::Costmap2DROS*   costmap_;
    std::string                 global_frame_;

    ros::NodeHandle nh_;
    ros::Subscriber speed_limit_sub_;

    std::vector<geometry_msgs::PoseStamped> plan_;

    double max_vel_x_;
    double max_vel_y_;
    double lookahead_dist_;
    double slow_down_dist_;
    double min_speed_;
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    double heading_kp_;
    double heading_max_omega_;

    std::atomic<double> speed_limit_;
};

}  // namespace haptiquad_nav
