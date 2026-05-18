#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LoadOctomapNode : public rclcpp::Node
{
public:
    LoadOctomapNode() : Node("load_octomap_node")
    {
        this->declare_parameter<std::string>("map_file", "/home/alice/ros2_ws/saved_map.ot");
        this->declare_parameter<std::string>("frame_id", "camera_init");

        map_file_ = this->get_parameter("map_file").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        pub_full_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            "/loaded_octomap_full", 1);

        pub_centers_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/loaded_octomap_point_cloud_centers", 1);

        load_map();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LoadOctomapNode::publish_map, this)
        );
    }

private:
    void load_map()
    {
        tree_.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(map_file_)));

        if (!tree_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load OctoMap: %s", map_file_.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded OctoMap: %s", map_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %.3f", tree_->getResolution());
    }

    void publish_map()
    {
        if (!tree_) {
            return;
        }

        auto now = this->now();

        octomap_msgs::msg::Octomap msg;
        if (!octomap_msgs::fullMapToMsg(*tree_, msg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert OctoMap to message");
            return;
        }

        msg.header.frame_id = frame_id_;
        msg.header.stamp = now;

        pub_full_->publish(msg);

        publish_centers(now);
    }

    void publish_centers(const rclcpp::Time& stamp)
    {
        std::vector<std::array<float, 3>> points;

        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
            if (tree_->isNodeOccupied(*it)) {
                points.push_back({
                    static_cast<float>(it.getX()),
                    static_cast<float>(it.getY()),
                    static_cast<float>(it.getZ())
                });
            }
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = frame_id_;
        cloud.header.stamp = stamp;
        cloud.height = 1;
        cloud.width = points.size();
        cloud.is_dense = true;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& p : points) {
            *iter_x = p[0];
            *iter_y = p[1];
            *iter_z = p[2];

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        pub_centers_->publish(cloud);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Published loaded map with %zu occupied cells",
            points.size()
        );
    }

    std::string map_file_;
    std::string frame_id_;

    std::unique_ptr<octomap::OcTree> tree_;

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_full_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_centers_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoadOctomapNode>());
    rclcpp::shutdown();
    return 0;
}
