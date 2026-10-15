#include "haptiquad_nav/haptiquad_nav_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(haptiquad_nav::HaptiQuadNavPlanner,
                       nav_core::BaseLocalPlanner)

namespace haptiquad_nav {

HaptiQuadNavPlanner::HaptiQuadNavPlanner()
    : initialized_(false), goal_reached_(false), tf_(nullptr), costmap_(nullptr),
      speed_limit_(std::numeric_limits<double>::infinity())
{}

void HaptiQuadNavPlanner::initialize(
    std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
{
    if (initialized_) return;

    tf_           = tf;
    costmap_      = costmap;
    global_frame_ = costmap->getGlobalFrameID();

    ros::NodeHandle pnh("~/" + name);
    pnh.param("max_vel_x",          max_vel_x_,          0.40);
    pnh.param("max_vel_y",          max_vel_y_,          0.20);
    pnh.param("lookahead_dist",     lookahead_dist_,     1.0);
    pnh.param("slow_down_dist",     slow_down_dist_,     1.5);
    pnh.param("min_speed",          min_speed_,          0.05);
    pnh.param("xy_goal_tolerance",  xy_goal_tolerance_,  0.15);
    pnh.param("yaw_goal_tolerance", yaw_goal_tolerance_,  0.10);
    pnh.param("heading_kp",         heading_kp_,          1.0);
    pnh.param("heading_max_omega",  heading_max_omega_,   0.5);

    speed_limit_sub_ = nh_.subscribe("/haptiquad_nav/speed_limit", 1,
                                     &HaptiQuadNavPlanner::speedLimitCb, this);

    initialized_ = true;
    ROS_INFO("[HaptiQuadNavPlanner] Ready (global_frame=%s, lookahead=%.2f m)",
             global_frame_.c_str(), lookahead_dist_);
}

void HaptiQuadNavPlanner::speedLimitCb(const std_msgs::Float64::ConstPtr& msg)
{
    // +inf means "no limit"; NaN or negative is a bad message -> stop.
    const double v = msg->data;
    if (std::isnan(v) || v < 0.0) {
        ROS_ERROR_THROTTLE(2.0, "[HaptiQuadNavPlanner] Invalid speed_limit %f -- using 0.", v);
        speed_limit_.store(0.0);
        return;
    }
    speed_limit_.store(v);
}

bool HaptiQuadNavPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("[HaptiQuadNavPlanner] Not initialized");
        return false;
    }
    plan_         = plan;
    goal_reached_ = false;
    return true;
}

bool HaptiQuadNavPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();  // safe default: stop

    if (!initialized_ || plan_.empty()) return false;

    // Robot pose in global frame (costmap handles TF internally)
    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_->getRobotPose(robot_pose)) {
        ROS_WARN_THROTTLE(1.0, "[HaptiQuadNavPlanner] Cannot get robot pose");
        return false;
    }

    const double rx   = robot_pose.pose.position.x;
    const double ry   = robot_pose.pose.position.y;
    const double ryaw = tf2::getYaw(robot_pose.pose.orientation);

    // Distance to goal — used for goal check and speed scaling
    const double gx        = plan_.back().pose.position.x;
    const double gy        = plan_.back().pose.position.y;
    const double goal_dist = std::hypot(gx - rx, gy - ry);
    const double goal_dyaw = std::abs(angles::shortest_angular_distance(
        ryaw, tf2::getYaw(plan_.back().pose.orientation)));

    if (goal_dist < xy_goal_tolerance_ && goal_dyaw < yaw_goal_tolerance_) {
        goal_reached_ = true;
        return true;
    }

    // Find closest plan point to current robot position
    int closest_idx = 0;
    double closest_d = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(plan_.size()); ++i) {
        const double d = std::hypot(plan_[i].pose.position.x - rx,
                                    plan_[i].pose.position.y - ry);
        if (d < closest_d) { closest_d = d; closest_idx = i; }
    }

    // Walk forward from closest point until we have accumulated lookahead_dist_
    int lookahead_idx = closest_idx;
    double accum = 0.0;
    for (int i = closest_idx; i < static_cast<int>(plan_.size()) - 1; ++i) {
        const double seg = std::hypot(
            plan_[i + 1].pose.position.x - plan_[i].pose.position.x,
            plan_[i + 1].pose.position.y - plan_[i].pose.position.y);
        accum += seg;
        lookahead_idx = i + 1;
        if (accum >= lookahead_dist_) break;
    }

    // Vector from robot to lookahead point in global frame
    const double dx = plan_[lookahead_idx].pose.position.x - rx;
    const double dy = plan_[lookahead_idx].pose.position.y - ry;

    // Rotate into robot (base) frame
    const double cos_y = std::cos(ryaw);
    const double sin_y = std::sin(ryaw);
    const double lx =  dx * cos_y + dy * sin_y;  // forward component
    const double ly = -dx * sin_y + dy * cos_y;  // lateral component

    const double target_dist = std::hypot(lx, ly);
    if (target_dist < 1e-3) return true;  // already at lookahead

    // Effective max speed is the lower of the configured max and the external limit.
    const double eff_max = std::min(max_vel_x_, speed_limit_.load());

    // Speed: full speed until slow_down_dist_, then linear ramp to min_speed_
    double speed = eff_max;
    if (goal_dist < slow_down_dist_)
        speed = std::max(min_speed_, eff_max * (goal_dist / slow_down_dist_));
    // min_speed_ must never exceed the external limit (speed_limit 0 = halt).
    speed = std::min(speed, eff_max);

    // Command velocity directly toward lookahead — no angular correction needed
    // because the robot strafes in any direction.
    double vx = speed * lx / target_dist;
    double vy = speed * ly / target_dist;

    // Per-axis clamp (preserves direction)
    if (std::abs(vy) > max_vel_y_) {
        const double scale = max_vel_y_ / std::abs(vy);
        vx *= scale;
        vy *= scale;
    }
    vx = std::max(-max_vel_x_, std::min(max_vel_x_, vx));

    // Yaw correction: P-controller toward final goal heading, always active.
    // The robot is holonomic, so this runs concurrently with linear motion.
    const double goal_yaw = tf2::getYaw(plan_.back().pose.orientation);
    const double dyaw     = angles::shortest_angular_distance(ryaw, goal_yaw);
    const double omega    = std::max(-heading_max_omega_,
                                     std::min(heading_max_omega_, heading_kp_ * dyaw));

    cmd_vel.linear.x  = vx;
    cmd_vel.linear.y  = vy;
    cmd_vel.angular.z = omega;

    return true;
}

bool HaptiQuadNavPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace haptiquad_nav
