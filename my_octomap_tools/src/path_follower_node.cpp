#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower_node")
    {
        declare_parameter<std::string>("path_topic", "/planned_path");
        declare_parameter<std::string>("odom_topic", "/Odometry");
        declare_parameter<std::string>("cmd_topic", "/cmd_vel");

        declare_parameter<double>("linear_speed", 0.25);
        declare_parameter<double>("angular_gain", 0.6);
        declare_parameter<double>("goal_tolerance", 0.25);
        declare_parameter<double>("yaw_tolerance", 0.2);

        path_topic_ = get_parameter("path_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        cmd_topic_ = get_parameter("cmd_topic").as_string();

        linear_speed_ = get_parameter("linear_speed").as_double();
        angular_gain_ = get_parameter("angular_gain").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10,
            std::bind(&PathFollower::path_cb, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&PathFollower::odom_cb, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathFollower::control_loop, this));

        RCLCPP_INFO(get_logger(), "Path follower started");
    }

private:
    double get_yaw(const geometry_msgs::msg::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double normalize_angle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void path_cb(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) return;

        path_ = *msg;
        target_index_ = 0;
        has_path_ = true;

        RCLCPP_INFO(get_logger(), "Received path with %zu poses", path_.poses.size());
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        has_odom_ = true;
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);
    }

    void control_loop()
    {
        if (!has_path_ || !has_odom_) {
            return;
        }

        if (target_index_ >= path_.poses.size()) {
            stop_robot();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal reached");
            return;
        }

        double rx = odom_.pose.pose.position.x;
        double ry = odom_.pose.pose.position.y;
        double yaw = get_yaw(odom_.pose.pose.orientation);

        auto target = path_.poses[target_index_].pose.position;

        double dx = target.x - rx;
        double dy = target.y - ry;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < goal_tolerance_) {
            target_index_++;
            return;
        }

        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - yaw);

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = linear_speed_;

        cmd.angular.z = angular_gain_ * yaw_error;

        if (cmd.angular.z > 0.4) cmd.angular.z = 0.4;
        if (cmd.angular.z < -0.4) cmd.angular.z = -0.4;

        cmd_pub_->publish(cmd);
    }

    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_topic_;

    double linear_speed_;
    double angular_gain_;
    double goal_tolerance_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;

    bool has_path_ = false;
    bool has_odom_ = false;

    size_t target_index_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
};
