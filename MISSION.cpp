
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
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
        client_clear = this->create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
        client_push = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");

   RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for MAVROS connection...");
    std::this_thread::sleep_for(std::chrono::seconds(2));   
    pushMission();

    setAUTO();
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    arm();
        


}
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client1_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr client_push;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr client_clear;


    void arm();
    void publish_takeoff_setpoint();
    void setAUTO();
    void pushMission();

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


void OffboardControl::setAUTO(){
        auto request1 = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request1->base_mode = 92 ;
        auto future_result1 = client1_->async_send_request(request1);
        while (!client1_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /mavros/set_mode service...");
}

    // 6. Spin the Node until the future is complete. This is the key step.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result1) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future_result1.get()->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Drone set AUTO successfully!");
        }   else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode.");}}}


void OffboardControl::pushMission(){
// std::vector<mavros_msgs::msg::Waypoint> wp_list;
    auto clear_req = std::make_shared<mavros_msgs::srv::WaypointClear::Request>();

        // Waypoint 1: Takeoff
        mavros_msgs::msg::Waypoint wp1;
        wp1.frame = 3;  // GLOBAL
        wp1.command = 22;  // NAV_TAKEOFF
        wp1.is_current = true;
        wp1.autocontinue = true;
        wp1.x_lat = 50;
        wp1.y_long = 10;
        wp1.z_alt = 10.0;

        // Waypoint 2: Move to point
        mavros_msgs::msg::Waypoint wp2;
        wp2.frame = 3;  // GLOBAL
        wp2.command = 16;  // NAV_WAYPOINT
        wp2.is_current = false;
        wp2.autocontinue = true;
        wp2.x_lat = 51;
        wp2.y_long = 11;
        wp2.z_alt = 10.0;


    auto request_wp = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
       request_wp->waypoints.push_back(wp1);
        request_wp->waypoints.push_back(wp2);

        auto future_push = client_push->async_send_request(request_wp);

}




int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}


