#include <chrono>
using namespace std::chrono_literals;
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("MyNode")
    {
        Mypublisher = this->create_publisher<std_msgs::msg::String>("node_name", 10);
        MyTimer = this->create_wall_timer(500ms, std::bind(&MyNode::sent_callback, this));

        batt_publisher = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
        batt_subscriber = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10, std::bind(&MyNode::batt_callback, this, std::placeholders::_1));

        this->declare_parameter<double>("battery_max", 42.0);
        this->declare_parameter<double>("battery_min", 32.0);
        
    }

private:
    void sent_callback()
    {
        std_msgs::msg::String Mymessage;
        Mymessage.data = "MyNode";
        Mypublisher->publish(Mymessage);
    }

    void batt_callback(const std_msgs::msg::Float32::SharedPtr battery)
    {
        std_msgs::msg::Float32 percent;
        double u_max = this->get_parameter("battery_max").as_double();
        double u_min = this->get_parameter("battery_min").as_double();

        percent.data = (battery->data - u_min)/(u_max - u_min)*100;
        batt_publisher->publish(percent);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Mypublisher;
    rclcpp::TimerBase::SharedPtr MyTimer;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr batt_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr batt_subscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}