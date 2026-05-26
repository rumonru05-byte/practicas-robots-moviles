#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

// Waypoints and parameters
struct Waypoint
{
    double x;
    double y;
};

class NavP2PNode : public rclcpp::Node
{
public:
    NavP2PNode() : Node("nav_p2p_node")
    {
        // Load parameters
        loadParameters();

        RCLCPP_INFO(this->get_logger(), "NavP2PNode started");
        // Publisher for cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
        // Subscriber for robot pose
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom", 10,
            std::bind(&NavP2PNode::odomCallback, this, std::placeholders::_1));

        // Timer to publish velocity commands at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&NavP2PNode::controlLoop, this));


        // Initialize waypoint index
        current_waypoint_index_ = 0;
        goal_reached_ = false;

        RCLCPP_INFO(this->get_logger(), "Point-to-Point Navigation Node Initialized");
        
        // Start navigation waypoints
        if (!waypoints_.empty()) {
            // Set the first waypoint as the goal
            setNextWaypoint();
        } else {
            RCLCPP_WARN(this->get_logger(), "No waypoints loaded! Check your YAML file.");
        }
    }

    void loadParameters()
    {
         // Declare parameters of this node (name, initial_value)
        this->declare_parameter("control_gain", 1.0);
        this->declare_parameter("tolerance", 0.1);
        this->declare_parameter("time_step", 25);  // in milliseconds
        this->declare_parameter("max_linear_speed", 1.2);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("wheel_base", 0.331);
        this->declare_parameter("wheel_radius", 0.097518);
        this->declare_parameter("loop_waypoints", false);
        this->declare_parameter("waypoints", std::vector<double>{});
        this->declare_parameter("show_markers", true);
        
        // Read parameters
        control_gain_ = this->get_parameter("control_gain").as_double();
        tolerance_ = this->get_parameter("tolerance").as_double();
        time_step_ = this->get_parameter("time_step").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        loop_waypoints_ = this->get_parameter("loop_waypoints").as_bool();
        show_markers_ = this->get_parameter("show_markers").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "control_gain: %.2f, tolerance: %.2f, max_linear_speed: %.2f, max_angular_speed: %.2f, wheel_base: %.2f, wheel_radius: %.2f, loop_waypoints: %s, show_markers: %s", 
                       control_gain_, tolerance_, max_linear_speed_, max_angular_speed_, wheel_base_, wheel_radius_, loop_waypoints_ ? "true" : "false", show_markers_ ? "true" : "false");

        // Load waypoints from parameters
        loadWaypoints();
    }

private:

    void loadWaypoints()
    {
        Waypoint wp;
        auto waypoint_param = this->get_parameter("waypoints").as_double_array();
        RCLCPP_INFO(this->get_logger(), "%zu waypoints params", waypoint_param.size());
        
        waypoints_.clear();
        for (size_t i = 0; i < waypoint_param.size(); i += 2)
        {
            wp.x = waypoint_param[i];
            wp.y = waypoint_param[i + 1];
            waypoints_.push_back(wp);
            
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f)", 
                       waypoints_.size(), wp.x, wp.y);
        }
    }

    void setNextWaypoint()
    {
        if (waypoints_.empty()) {
            return;
        }
        
        goal_x_ = waypoints_[current_waypoint_index_].x;
        goal_y_ = waypoints_[current_waypoint_index_].y;
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Navigating to waypoint %zu/%zu: (%.2f, %.2f)",
                   current_waypoint_index_ + 1, waypoints_.size(),
                   goal_x_, goal_y_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // Extract yaw from quaternion
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    void controlLoop()
    {
        if (waypoints_.empty()) {
            return;
        }
        /////////// PUT YOUR CONTROL CODE HERE ///////////

        // Compute distance and angle to the goal
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance = hypot(dx, dy);
        double angle_to_goal = atan2(dy, dx);
        double angle_error = angle_to_goal - current_theta_;
        
        // Normalize angle error to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        double angular_velocity = control_gain_ * angle_error;
        double linear_velocity = max_linear_speed_ * (1 - fabs(angle_error) / M_PI); // Reduce speed when turning


        if (angular_velocity > max_angular_speed_) angular_velocity = max_angular_speed_;
        if (angular_velocity < -max_angular_speed_) angular_velocity = -max_angular_speed_;
        
        geometry_msgs::msg::Twist cmd_vel_msg;

        // Simple proportional controller
        if (distance > tolerance_)
        {
            cmd_vel_msg.linear.x =  linear_velocity;
            cmd_vel_msg.angular.z = angular_velocity;
        }
        else
        {
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Waypoint %zu reached at (%.2f, %.2f)",
                       current_waypoint_index_ + 1,
                       goal_x_, goal_y_);
        }   

        cmd_vel_pub_->publish(cmd_vel_msg);
        // Check if goal is reached
        if (goal_reached_)
        {
            current_waypoint_index_++;
            if (current_waypoint_index_ >= waypoints_.size())
            {
                if (loop_waypoints_)
                {
                    current_waypoint_index_ = 0;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
                    timer_->cancel(); // Stop the control loop when finished
                    return;
                }
            }
            setNextWaypoint();
        }
    }

    // Publishers y Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Actual robot position
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;  
    
    // Goal position
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    bool goal_reached_;

    // Parameters
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool loop_waypoints_;
    bool show_markers_;
    // Controller gains
    double control_gain_;
    double tolerance_;
    // Vehicle parameters
    double wheel_base_;
    double wheel_radius_;
    int time_step_;
    double max_linear_speed_;
    double max_angular_speed_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavP2PNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}