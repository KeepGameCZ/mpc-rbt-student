#include <chrono>
using namespace std::chrono_literals;
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("MyNode")
    {
        Mypublisher = this->create_publisher<std_msgs::msg::String>("node_name", 10);
        MyTimer = this->create_wall_timer(500ms, std::bind(&MyNode::sent_callback, this));
    }

private:
    void sent_callback()
    {
        std_msgs::msg::String Mymessage;
        Mymessage.data = "MyNode";
        Mypublisher->publish(Mymessage);

    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Mypublisher;
    rclcpp::TimerBase::SharedPtr MyTimer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}