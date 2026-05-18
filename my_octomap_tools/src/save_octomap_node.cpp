#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

using GetOctomap = octomap_msgs::srv::GetOctomap;

class SaveOctomapNode : public rclcpp::Node
{
public:
    SaveOctomapNode() : Node("save_octomap_node")
    {
        client_ = this->create_client<GetOctomap>("/octomap_full");
    }

    void save()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for /octomap_full service...");

        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service /octomap_full not available");
            return;
        }

        auto request = std::make_shared<GetOctomap::Request>();
        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /octomap_full");
            return;
        }

        auto response = future.get();

        octomap::AbstractOcTree* tree =
            octomap_msgs::fullMsgToMap(response->map);

        if (!tree) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message");
            return;
        }

        std::string filename = "saved_map.ot";

        if (tree->write(filename)) {
            RCLCPP_INFO(this->get_logger(), "Saved OctoMap to %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save OctoMap");
        }

        delete tree;
    }

private:
    rclcpp::Client<GetOctomap>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SaveOctomapNode>();
    node->save();

    rclcpp::shutdown();
    return 0;
}
