#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    
    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        10,
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();

    //if(dt > 0.5) dt = 0.0;

    
    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    
    last_time_ = current_time;
   //RCLCPP_INFO(this->get_logger(), "Přijata data z kol!");
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here
    
    // ********
    // * Help *
    // ********
    double right_wheel_speed = robot_config::WHEEL_RADIUS * right_wheel_vel;
    double left_wheel_speed = robot_config::WHEEL_RADIUS * left_wheel_vel;
    
    double linear =  (right_wheel_speed + left_wheel_speed)/2;
    double angular = (left_wheel_speed - right_wheel_speed)/(2*robot_config::HALF_DISTANCE_BETWEEN_WHEELS);
    
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    double dx = linear * std::cos(theta) * dt;
    double dy = linear * std::sin(theta) * dt;
    double d_theta = angular * dt;
    theta += d_theta;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);
    odometry_.pose.pose.position.x += dx;
    odometry_.pose.pose.position.y += dy;
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.linear.y = 0.0;
    odometry_.twist.twist.linear.z = 0.0;

    odometry_.twist.twist.angular.x = 0.0;
    odometry_.twist.twist.angular.y = 0.0;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();

    odometry_.header.frame_id = "odom";
    odometry_.child_frame_id = "base_link";

    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
