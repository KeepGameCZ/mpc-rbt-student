#include "Planning.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "plan_path", 
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // Subscriber for RBT positon
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&PlanningNode::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "#RBT positon loaded#");
        

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        auto future = map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::odomCallback(const nav_msgs::msg::Odometry &odometry_subscriber_) { //callback pro cteni pozice RBT
    current_robot_pose_ = odometry_subscriber_.pose.pose;
    robot_pose_received_ = true; // pojistka ze pozice RBT prisla
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(this->get_logger(), "Map loaded! Width: %d, Height: %d", map_.info.width, map_.info.height);
    
        dilateMap();
    }
    
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "map";
    start_pose.header.stamp = this->get_clock()->now();

    if (robot_pose_received_) {
        start_pose.pose = current_robot_pose_;
        start_pose.pose.position.x += -0.5;
    } else {
        start_pose = request->start;
    }
    
    aStar(start_pose, request->goal);
    smoothPath();
    path_pub_->publish(path_);
    response->plan = path_;
}

void PlanningNode::dilateMap() {
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    int dilate_Size = 8 + (robot_config::HALF_DISTANCE_BETWEEN_WHEELS / map_.info.resolution);
    RCLCPP_INFO(get_logger(), "Dilatation radius %d", dilate_Size);
    int map_width = map_.info.width;
    int map_height = map_.info.height;

    for (int y = 0; y < map_height; y++) {
        for (int x = 0; x < map_width; x++) {
            int index = y * map_width + x;
            if(map_.data[index] > 50) {
                for (int dy = -dilate_Size; dy <= dilate_Size; dy++) {
                    for (int dx = -dilate_Size; dx <= dilate_Size; dx++) {
                        if (dx*dx + dy*dy <= dilate_Size*dilate_Size) { //osetreni ze bude "kruh"
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
                                int surr_index = ny * map_width + nx;
                                dilatedMap.data[surr_index] = 100;
                            }
                        }
                    }                    
                }
            }
        }        
    }
    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Map Dilated!");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    int start_x = (int)((start.pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
    int start_y = (int)((start.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);
    int goal_x = (int)((goal.pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
    int goal_y = (int)((goal.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);

    Cell cStart(start_x, start_y);
    Cell cGoal(goal_x, goal_y);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const float step_cost[8] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};


    while(!openList.empty() && rclcpp::ok()) {
        auto best_it = std::min_element(
            openList.begin(), 
            openList.end(),
            [](const std::shared_ptr<Cell>& a, const std::shared_ptr<Cell>& b) {
                return a->f < b->f;
            }   
        );

        std::shared_ptr<Cell> current = *best_it;
        openList.erase(best_it);
        int current_index = current->y * map_.info.width + current->x;
        closedList[current_index] = true;

        if (current->x == goal_x && current->y == goal_y) {
            RCLCPP_INFO(get_logger(), "Path Founded! :D");

            path_.poses.clear(); // cistka stare cesty
            path_.header.frame_id = "map";
            path_.header.stamp = this->get_clock()->now();

            std::shared_ptr<Cell> trace = current; // zacnu od konce

            while (trace != nullptr) { //jdu po rodicich az na zacatek a ukladam cestu a souradnice
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_.header;
            
                pose.pose.position.x = (trace->x * map_.info.resolution) + map_.info.origin.position.x;
                pose.pose.position.y = (trace->y * map_.info.resolution) + map_.info.origin.position.y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.w = 1.0;

                path_.poses.push_back(pose);

                trace = trace->parent;
            }
            std::reverse(path_.poses.begin(), path_.poses.end()); //otoceni cesty = start -> cil
        
            return;
        }

        for (int i = 0; i < 8; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];
            float cost = step_cost[i];
        
            if (nx < 0 || nx >= (int)map_.info.width || ny < 0 || ny >= (int)map_.info.height) { continue; }// mimo mapu!

            int neighbor_index = ny * map_.info.width + nx;

            if (map_.data[neighbor_index] > 50 || map_.data[neighbor_index] == -1) { continue; }// Zed nebo void
            if (closedList[neighbor_index] == true) { continue; }// Zde si uz byl

            float new_g = current->g + cost; // vypocet hodnot noveho bodu
            float new_h = std::hypot(goal_x - nx, goal_y - ny);
            float new_f = new_g + new_h;

            std::shared_ptr<Cell> neighbor = std::make_shared<Cell>(nx, ny); // ulozeni noveho bodu postupu
            neighbor->g = new_g;
            neighbor->h = new_h;
            neighbor->f = new_f;
            neighbor->parent = current;

            bool in_open_list = false;  // hledani optimalnejsi cesty
            for (auto& open_node : openList) { //neboli jestli uz tento bod jednu byl kontrolovan
                if (open_node->x == nx && open_node->y == ny) {
                    in_open_list = true;
                
                    if (new_g < open_node->g) {
                        open_node->g = new_g;
                        open_node->f = new_f;
                        open_node->parent = current;
                    }
                    break;
                }
            }

            if (!in_open_list) {
                openList.push_back(neighbor);
            }
        }  
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    
}

void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3)
        return;
    
    int iters = 50;
    for (int i = 0; i < iters; i++) {
        std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
        for (size_t p = 1; p < (path_.poses.size() - 1); p++) {
            newPath[p].pose.position.x = ((path_.poses[p-1].pose.position.x + path_.poses[p].pose.position.x + path_.poses[p+1].pose.position.x) / 3.0);
            newPath[p].pose.position.y = ((path_.poses[p-1].pose.position.y + path_.poses[p].pose.position.y + path_.poses[p+1].pose.position.y) / 3.0);
        }
        path_.poses = newPath;
    }
} 

Cell::Cell(int c, int r) {
    this->x = c;
    this->y = r;
    this->f = 0.0;
    this->g = 0.0;
    this->h = 0.0;
    this->parent = nullptr;  
}
