
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/position_target.hpp"

#include <memory>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;


class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control_mavros") {

        client1_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    
   RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");
           // Start timer for periodic setpoint publishing
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_takeoff_setpoint, this));
    std::this_thread::sleep_for(std::chrono::seconds(3));    setOffboard();
    arm();
        


}
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;

    void arm();
    void setOffboard();
    void publish_takeoff_setpoint();

}; 

void OffboardControl::arm(){
        auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arming_request->value = true; // true to arm, false to disarm
        auto arming_future = arming_client_->async_send_request(arming_request);
        while (!arming_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/cmd/arming service...");
}


    // Spin and wait for the arming result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arming_future) == rclcpp::FutureReturnCode::SUCCESS) {
        if (arming_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Drone ARMED successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm drone.");
    }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call arming service.");}}

void OffboardControl::setOffboard(){
        auto request1 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request1->custom_mode = "OFFBOARD" ;
        auto future_result1 = client1_->async_send_request(request1);
        while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
}

    // 6. Spin the Node until the future is complete. This is the key step.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result1) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future_result1.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Drone set OFFBOARD successfully!");
        }   else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode.");}}}

void OffboardControl::publish_takeoff_setpoint() {
        auto setpoint_msg = mavros_msgs::msg::PositionTarget();
        setpoint_msg.header.stamp = this->get_clock()->now();
        setpoint_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                 mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                 mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                                 mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        setpoint_msg.position.x = 0;
        setpoint_msg.position.y = 0;
        setpoint_msg.position.z = 10; 
        setpoint_msg.yaw = 0;
        setpoint_pub_->publish(setpoint_msg);}




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}


