#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1)
        );

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/tiago_base/Hokuyo_URG_04LX_UG01",
            rclcpp::SensorDataQoS(),
            std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1)            
        );
        
        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>( this, "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for planning service to be available...");
        }
    }

void MotionControlNode::checkCollision() { 
    if(laser_scan_.ranges.empty())
        return;
    
    for(int i = 0; i < (int)laser_scan_.ranges.size(); i++) {
        if(laser_scan_.ranges[i] < 0.3 || (i > (int)laser_scan_.ranges.size() && (laser_scan_.ranges.size() - laser_scan_.ranges.size()/6)))
            continue;

        if (laser_scan_.ranges[i] < thresh) {
            obstacle_detected_ = true;
            geometry_msgs::msg::Twist stop;
            stop.linear.x = 0.0;
            stop.angular.z = 0.0;
            twist_publisher_->publish(stop);
            RCLCPP_ERROR(this->get_logger(), "Collision in path!!!!");
            
            path_.poses.clear();
            return;
        }
    }
    return;
}

void MotionControlNode::updateTwist() {
    geometry_msgs::msg::Twist twist;
    float v_max = 0.2;
    double lookahead_dist = 0.4; // Díváme se 40 cm dopředu

    if (path_.poses.empty()) {
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;
        twist_publisher_->publish(twist);
        return;
    }
    
    // --- 1. SNÍMEK STAVU A TVŮJ POSUN SOUŘADNIC ---
    // Zde vracíme zpět tvůj posun mapy! Bez něj robot viděl trasu za sebou.
    double rx = current_pose_.pose.position.x - 0.5; 
    double ry = current_pose_.pose.position.y;

    tf2::Quaternion q(
        current_pose_.pose.orientation.x, 
        current_pose_.pose.orientation.y, 
        current_pose_.pose.orientation.z, 
        current_pose_.pose.orientation.w
    ); 
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // --- 2. CHYTRÝ PAC-MAN (Smazání projetých bodů) ---
    while (path_.poses.size() > 1) {
        double d1 = std::hypot(path_.poses[0].pose.position.x - rx, path_.poses[0].pose.position.y - ry);
        double d2 = std::hypot(path_.poses[1].pose.position.x - rx, path_.poses[1].pose.position.y - ry);
        
        if (d2 <= d1) {
            path_.poses.erase(path_.poses.begin()); 
        } else {
            break; 
        }
    }

    // --- KONTROLA DOSAŽENÍ CÍLE ---
    if (path_.poses.size() <= 2) {
        if (!path_.poses.empty()) {
            double dist_to_goal = std::hypot(path_.poses.back().pose.position.x - rx,
                                             path_.poses.back().pose.position.y - ry);
            if (dist_to_goal < 0.15) {
                path_.poses.clear();
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                twist_publisher_->publish(twist);
                RCLCPP_INFO(this->get_logger(), "--- !GOAL FINISHED! :3 ---");
                return;
            }
        }
    }

    if (path_.poses.empty()) return;

    // --- 3. VÝBĚR CÍLOVÉHO BODU (Mrkvička) ---
    int target_idx = 0;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double d = std::hypot(path_.poses[i].pose.position.x - rx, path_.poses[i].pose.position.y - ry);
        target_idx = i;
        if (d > lookahead_dist) {
            break;
        }
    }

    double tx = path_.poses[target_idx].pose.position.x;
    double ty = path_.poses[target_idx].pose.position.y;
    double dx = tx - rx;
    double dy = ty - ry;

    // --- 4. VÝPOČET ÚHLU K CÍLI ---
    double target_angle = std::atan2(dy, dx);
    double angular_diff = target_angle - yaw;

    if (angular_diff > M_PI) 
        angular_diff -= 2.0 * M_PI;         
    if (angular_diff < -M_PI) 
        angular_diff += 2.0 * M_PI;

    // --- 5. HYBRIDNÍ ŘÍZENÍ (ČISTÁ MATEMATIKA BEZ MÍNUSŮ!) ---
    if (std::abs(angular_diff) > 0.6) { 
        // Otočka na místě k cíli (Správná polarita!)
        twist.linear.x = 0.0; 
        twist.angular.z = 1.5 * angular_diff; 
    } else {
        // Hladký Pure Pursuit po trase (Správná polarita!)
        double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);
        twist.linear.x = v_max;
        twist.angular.z = (2.0 * v_max * local_y) / (lookahead_dist * lookahead_dist);
    }

    // --- 6. OCHRANA MOTORŮ ---
    if(twist.angular.z > 1.0) twist.angular.z = 1.0;
    if(twist.angular.z < -1.0) twist.angular.z = -1.0;
        
    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    (void)uuid;

    double go_x = goal->pose.pose.position.x;
    double go_y = goal->pose.pose.position.y;

    RCLCPP_INFO(this->get_logger(), "NEW GOAL: Going on [X: %.2f, Y: %.2f]", go_x, go_y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    path_.poses.clear();
    geometry_msgs::msg::Twist stop;
    twist_publisher_->publish(stop);
    obstacle_detected_ = false;

    
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_; // nebylo by potreba protoze aktualni pozici robotu jiz vkladam v Planning.cpp::plan_path
    request->goal = goal_handle->get_goal()->pose;
    goal_handle_ = goal_handle;
    
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(10.0); // 10 Hz

    while (rclcpp::ok() && goal_handle_->is_active()) {

        if (goal_handle_->is_canceling()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_ ->canceled(result);
            RCLCPP_INFO(this->get_logger(), "!!Action was canceled!!");
            return;
        }

        if (obstacle_detected_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            RCLCPP_ERROR(this->get_logger(), "!!! OBSTACKLE - Stoping !!!");
            return;
        }

        if (path_.poses.empty()) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_ ->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Action Completed :D");
                path_.poses.clear();
                return;   
        }

        auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        feedback->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    
    auto response = future.get();
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;

        // if (path_.poses.size() >= 3) {
        //     RCLCPP_INFO(this->get_logger(), "--- TRASA ÚSPĚŠNĚ PŘIJATA ---");
        //     RCLCPP_INFO(this->get_logger(), "Celkem bodu: %zu", path_.poses.size());
        //     RCLCPP_INFO(this->get_logger(), "Bod 0: [X: %.3f, Y: %.3f]", path_.poses[0].pose.position.x, path_.poses[0].pose.position.y);
        //     RCLCPP_INFO(this->get_logger(), "Bod 1: [X: %.3f, Y: %.3f]", path_.poses[1].pose.position.x, path_.poses[1].pose.position.y);
        //     RCLCPP_INFO(this->get_logger(), "Bod 2: [X: %.3f, Y: %.3f]", path_.poses[2].pose.position.x, path_.poses[2].pose.position.y);
        // }
        if (goal_handle_->is_active()) {
            goal_handle_->execute();
            std::thread(&MotionControlNode::execute, this).detach();
            RCLCPP_INFO(this->get_logger(), "--- PATH PLANNED---");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Planner didn't plan any path!");
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.pose = msg.pose.pose;
    checkCollision();
    updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    
    //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "--- LIDAR ZIJE! (Pocet paprsku: %zu) ---", msg.ranges.size());

    laser_scan_ = msg;
    checkCollision();
}
