#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <random>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

struct Node3D
{
    double x, y, z;
    int parent;
};

class RRT3DPlanner : public rclcpp::Node
{
public:
    RRT3DPlanner() : Node("rrt_3d_planner")
    {
        declare_parameter<std::string>("map_file", "/home/alice/ros2_ws/saved_map.ot");
        declare_parameter<std::string>("frame_id", "camera_init");

        declare_parameter<double>("start_x", 0.0);
        declare_parameter<double>("start_y", 0.0);
        declare_parameter<double>("start_z", 0.5);

        declare_parameter<double>("goal_x", 5.0);
        declare_parameter<double>("goal_y", 0.0);
        declare_parameter<double>("goal_z", 0.5);

        declare_parameter<double>("min_x", -10.0);
        declare_parameter<double>("max_x", 10.0);
        declare_parameter<double>("min_y", -10.0);
        declare_parameter<double>("max_y", 10.0);
        declare_parameter<double>("min_z", 0.0);
        declare_parameter<double>("max_z", 3.0);

        declare_parameter<double>("step_size", 0.4);
        declare_parameter<double>("goal_tolerance", 0.5);
        declare_parameter<int>("max_iterations", 5000);

        declare_parameter<double>("robot_radius", 0.35);

        map_file_ = get_parameter("map_file").as_string();
        frame_id_ = get_parameter("frame_id").as_string();

        start_ = {
            get_parameter("start_x").as_double(),
            get_parameter("start_y").as_double(),
            get_parameter("start_z").as_double(),
            -1
        };

        goal_ = {
            get_parameter("goal_x").as_double(),
            get_parameter("goal_y").as_double(),
            get_parameter("goal_z").as_double(),
            -1
        };

        min_x_ = get_parameter("min_x").as_double();
        max_x_ = get_parameter("max_x").as_double();
        min_y_ = get_parameter("min_y").as_double();
        max_y_ = get_parameter("max_y").as_double();
        min_z_ = get_parameter("min_z").as_double();
        max_z_ = get_parameter("max_z").as_double();

        step_size_ = get_parameter("step_size").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        max_iterations_ = get_parameter("max_iterations").as_int();
        robot_radius_ = get_parameter("robot_radius").as_double();

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        load_map();

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RRT3DPlanner::plan_once, this)
        );
    }

private:
    void load_map()
    {
        tree_.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(map_file_)));

        if (!tree_) {
            RCLCPP_ERROR(get_logger(), "Failed to load OctoMap: %s", map_file_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Loaded map: %s", map_file_.c_str());
        RCLCPP_INFO(get_logger(), "Resolution: %.3f", tree_->getResolution());
    }

    double dist(const Node3D& a, const Node3D& b)
    {
        return std::sqrt(
            (a.x - b.x) * (a.x - b.x) +
            (a.y - b.y) * (a.y - b.y) +
            (a.z - b.z) * (a.z - b.z)
        );
    }

    int nearest_node(const std::vector<Node3D>& nodes, const Node3D& sample)
    {
        int best = 0;
        double best_dist = dist(nodes[0], sample);

        for (int i = 1; i < static_cast<int>(nodes.size()); i++) {
            double d = dist(nodes[i], sample);
            if (d < best_dist) {
                best_dist = d;
                best = i;
            }
        }

        return best;
    }

    Node3D steer(const Node3D& from, const Node3D& to)
    {
        double d = dist(from, to);

        if (d < step_size_) {
            return {to.x, to.y, to.z, -1};
        }

        double ratio = step_size_ / d;

        return {
            from.x + (to.x - from.x) * ratio,
            from.y + (to.y - from.y) * ratio,
            from.z + (to.z - from.z) * ratio,
            -1
        };
    }

    bool is_occupied(double x, double y, double z)
    {
        if (!tree_) {
            return true;
        }

        octomap::point3d p(x, y, z);
        octomap::OcTreeNode* node = tree_->search(p);

        if (node && tree_->isNodeOccupied(node)) {
            return true;
        }

        return false;
    }

    bool is_state_free(const Node3D& p)
    {
        double res = tree_->getResolution();

        for (double dx = -robot_radius_; dx <= robot_radius_; dx += res) {
            for (double dy = -robot_radius_; dy <= robot_radius_; dy += res) {
                for (double dz = -robot_radius_; dz <= robot_radius_; dz += res) {

                    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (d > robot_radius_) {
                        continue;
                    }

                    if (is_occupied(p.x + dx, p.y + dy, p.z + dz)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool is_edge_free(const Node3D& a, const Node3D& b)
    {
        double d = dist(a, b);
        int steps = std::max(2, static_cast<int>(d / (tree_->getResolution() * 0.5)));

        for (int i = 0; i <= steps; i++) {
            double t = static_cast<double>(i) / static_cast<double>(steps);

            Node3D p{
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t,
                -1
            };

            if (!is_state_free(p)) {
                return false;
            }
        }

        return true;
    }

    Node3D random_sample()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());

        std::uniform_real_distribution<double> dx(min_x_, max_x_);
        std::uniform_real_distribution<double> dy(min_y_, max_y_);
        std::uniform_real_distribution<double> dz(min_z_, max_z_);
        std::uniform_real_distribution<double> bias(0.0, 1.0);

        // 10% samples are goal-biased
        if (bias(gen) < 0.10) {
            return goal_;
        }

        return {dx(gen), dy(gen), dz(gen), -1};
    }

    void publish_path(const std::vector<Node3D>& nodes, int goal_index)
    {
        std::vector<Node3D> reversed_path;

        int current = goal_index;
        while (current >= 0) {
            reversed_path.push_back(nodes[current]);
            current = nodes[current].parent;
        }

        std::reverse(reversed_path.begin(), reversed_path.end());

        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_;
        path.header.stamp = now();

        for (const auto& p : reversed_path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.position.z = p.z;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        last_path_ = path;
        has_path_ = true;
        path_pub_->publish(last_path_);

        RCLCPP_INFO(get_logger(), "Published path with %zu poses", path.poses.size());
    }

    void plan_once()
    {
        if (planned_ && has_path_) {
            last_path_.header.stamp = now();
            path_pub_->publish(last_path_);
            return;
        } 
          
        if (planned_) {
            return;
        }

        if (!tree_) {
            RCLCPP_ERROR(get_logger(), "No map loaded");
            return;
        }

        if (!is_state_free(start_)) {
            RCLCPP_ERROR(get_logger(), "Start is occupied or too close to obstacle");
            planned_ = true;
            return;
        }

        if (!is_state_free(goal_)) {
            RCLCPP_ERROR(get_logger(), "Goal is occupied or too close to obstacle");
            planned_ = true;
            return;
        }

        std::vector<Node3D> nodes;
        nodes.push_back(start_);

        RCLCPP_INFO(get_logger(), "Planning RRT from A to B...");

        for (int i = 0; i < max_iterations_; i++) {
            Node3D sample = random_sample();

            int nearest = nearest_node(nodes, sample);
            Node3D new_node = steer(nodes[
nearest], sample);
            new_node.parent = nearest;

            if (!is_state_free(new_node)) {
                continue;
            }

            if (!is_edge_free(nodes[nearest], new_node)) {
                continue;
            }

            nodes.push_back(new_node);

            if (dist(new_node, goal_) < goal_tolerance_) {
                Node3D final_goal = goal_;
                final_goal.parent = static_cast<int>(nodes.size()) - 1;

                if (is_edge_free(new_node, goal_)) {
                    nodes.push_back(final_goal);
                    publish_path(nodes, static_cast<int>(nodes.size()) - 1);
                    planned_ = true;
                    return;
                }
            }
        }

        RCLCPP_WARN(get_logger(), "RRT failed. Try increasing max_iterations or changing bounds.");
        planned_ = true;
    }

    std::string map_file_;
    std::string frame_id_;

    Node3D start_;
    Node3D goal_;

    double min_x_, max_x_;
    double min_y_, max_y_;
    double min_z_, max_z_;

    double step_size_;
    double goal_tolerance_;
    int max_iterations_;
    double robot_radius_;

    bool planned_ = false;
    
    nav_msgs::msg::Path last_path_;
    bool has_path_ = false;

    std::unique_ptr<octomap::OcTree> tree_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT3DPlanner>());
    rclcpp::shutdown();
    return 0;
} 
