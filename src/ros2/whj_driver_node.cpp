/**
 * @file whj_driver_node.cpp
 * @brief ROS2 node for RealMan WHJ Joint Motors
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include "realman_whj/driver.hpp"

using namespace realman_whj;

class WHJDriverNode : public rclcpp::Node {
public:
    WHJDriverNode() : Node("whj_driver_node") {
        // Declare parameters
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<std::vector<int64_t>>("motor_ids", {1});
        this->declare_parameter<std::vector<std::string>>("joint_names", {"joint_1"});
        this->declare_parameter<double>("update_rate", 50.0);
        this->declare_parameter<double>("position_offset", 0.0);
        
        // Get parameters
        std::string can_interface = this->get_parameter("can_interface").as_string();
        auto motor_ids_param = this->get_parameter("motor_ids").as_integer_array();
        joint_names_ = this->get_parameter("joint_names").as_string_array();
        double update_rate = this->get_parameter("update_rate").as_double();
        
        // Convert motor IDs
        for (auto id : motor_ids_param) {
            motor_ids_.push_back(static_cast<uint8_t>(id));
        }
        
        // Validate parameters
        if (motor_ids_.size() != joint_names_.size()) {
            RCLCPP_ERROR(this->get_logger(), "motor_ids and joint_names must have same size");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize driver
        RCLCPP_INFO(this->get_logger(), "Initializing driver on %s", can_interface.c_str());
        
        DriverConfig config;
        config.timeout_ms = 100;
        
        if (!driver_.init(can_interface, config)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize driver: %s", 
                        driver_.getLastError().c_str());
            rclcpp::shutdown();
            return;
        }
        
        // Enable motors
        for (uint8_t id : motor_ids_) {
            RCLCPP_INFO(this->get_logger(), "Enabling motor %d", id);
            if (!driver_.enableMotor(id, true)) {
                RCLCPP_WARN(this->get_logger(), "Failed to enable motor %d", id);
            }
        }
        
        // Create publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        
        // Create subscribers
        position_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "position_command", 10,
            std::bind(&WHJDriverNode::positionCommandCallback, this, std::placeholders::_1));
        
        // Create services
        enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "enable_motors",
            std::bind(&WHJDriverNode::enableService, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        reset_zero_srv_ = this->create_service<std_srvs::srv::Empty>(
            "reset_zero_position",
            std::bind(&WHJDriverNode::resetZeroService, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create timer for periodic updates
        auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&WHJDriverNode::updateTimer, this));
        
        RCLCPP_INFO(this->get_logger(), "WHJ Driver Node started with %zu motors", 
                   motor_ids_.size());
    }
    
    ~WHJDriverNode() {
        // Disable motors on shutdown
        for (uint8_t id : motor_ids_) {
            driver_.enableMotor(id, false);
        }
        driver_.deinit();
    }

private:
    void updateTimer() {
        auto states = driver_.getMultipleStates(motor_ids_);
        
        if (states.empty()) {
            return;
        }
        
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        
        for (const auto& [id, state] : states) {
            // Find joint name for this motor
            auto it = std::find(motor_ids_.begin(), motor_ids_.end(), id);
            if (it != motor_ids_.end()) {
                size_t idx = std::distance(motor_ids_.begin(), it);
                msg.name.push_back(joint_names_[idx]);
                msg.position.push_back(state.position_deg * M_PI / 180.0);  // Convert to radians
                msg.velocity.push_back(state.speed_rpm * 2.0 * M_PI / 60.0);  // Convert to rad/s
                msg.effort.push_back(state.current_ma / 1000.0);  // Convert to A
            }
        }
        
        joint_state_pub_->publish(msg);
    }
    
    void positionCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != motor_ids_.size()) {
            RCLCPP_WARN(this->get_logger(), "Position command size mismatch");
            return;
        }
        
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            // Convert from radians to degrees
            float position_deg = msg->data[i] * 180.0 / M_PI;
            driver_.setTargetPosition(motor_ids_[i], position_deg);
        }
    }
    
    void enableService(const std_srvs::srv::SetBool::Request::SharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response) {
        bool success = true;
        for (uint8_t id : motor_ids_) {
            if (!driver_.enableMotor(id, request->data)) {
                success = false;
            }
        }
        response->success = success;
        response->message = request->data ? "Motors enabled" : "Motors disabled";
    }
    
    void resetZeroService(const std_srvs::srv::Empty::Request::SharedPtr,
                         std_srvs::srv::Empty::Response::SharedPtr) {
        for (uint8_t id : motor_ids_) {
            driver_.setZeroPosition(id);
        }
        RCLCPP_INFO(this->get_logger(), "Zero position reset");
    }
    
    WHJDriver driver_;
    std::vector<uint8_t> motor_ids_;
    std::vector<std::string> joint_names_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_cmd_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_zero_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WHJDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
