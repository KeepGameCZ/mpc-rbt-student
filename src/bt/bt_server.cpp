#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options)
        : TreeExecutionServer(options) {}

    void onTreeCreated(BT::Tree& tree) override {
        BT::StdCoutLogger logger_cout(tree);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto server = std::make_shared<BTServer>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
