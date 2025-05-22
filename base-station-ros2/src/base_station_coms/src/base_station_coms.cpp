
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"




#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"


#include <iostream>
#include <chrono>
#include <memory>


using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;


using std::placeholders::_1;
using std::placeholders::_2;


class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("base_station_coms") {


        this->declare_parameter<int>("status_request_frequency_seconds", 15);
        this->status_request_frequency = this->get_parameter("status_request_frequency_seconds").as_int();


        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();


        // client for the base_station_radio node. Requests that the radio sends an emergency kill command to a specific coug
        radio_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_e_kill",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );
        
        // client for the base_station_modem node. Requests that the modem sends an emergency kill command to a specific coug
        modem_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_kill",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        // client for the base_station_modem node. Requests that the modem sends an emergency surface command to a specific coug
        modem_e_surface_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_surface",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        // client for the base_station_modem node. Requests that the modem gets the status of a coug
        modem_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_status_request",
            std::bind(&ComsNode::emergency_surface_callback, this, _1, _2)
        );

        // client for the base_station_radio node. Requests that the radio gets the status of a coug
        radio_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_status_request",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        // service for sending e_kill message. Decides whether to send over radio or modem
        emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "e_kill_service",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        // service for sending e_surface message. Decides whether to send over radio or modem
        emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "e_surface_service",
            std::bind(&ComsNode::emergency_surface_callback, this, _1, _2)
        );

        // service for sending e_kill message. Decides whether to send over radio or modem
        confirm_emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "confirm_e_kill_service",
            std::bind(&ComsNode::confirm_e_kill_callback, this, _1, _2)
        );

        // service for sending e_surface message. Decides whether to send over radio or modem
        confirm_emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "confirm_e_surface_service",
            std::bind(&ComsNode::confirm_e_surface_callback, this, _1, _2)
        );

        timer_ = this->create_wall_timer(
                    std::chrono::seconds(status_request_frequency), std::bind(&ComsNode::request_status_callback, this));


        std::ostringstream ss;
        ss << "Vehicle ids in mission: ";
        for(int64_t i: vehicles_in_mission_) ss << i << ", ";
        RCLCPP_INFO(this->get_logger(), "base station started");
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());




    }

    void request_status_callback(){
       
        coug_id_index += 1;
        if (coug_id_index+1>=vehicles_in_mission_.size())
            coug_id_index = 0;
        
        auto request = std::make_shared<base_station_interfaces::srv::BeaconId::Request>();
        request->beacon_id = vehicles_in_mission_[coug_id_index];

        if (radio_connection[request->beacon_id]){

            radio_status_request_client_->async_send_request(request,
                [this](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through radio", vehicle_turn_id);
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through radio failed", vehicle_turn_id);
                });

        } else if (modem_connection[request->beacon_id]) {
            
            modem_status_request_client_->async_send_request(request,
                [this](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through modem", vehicle_turn_id);
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through modem failed", vehicle_turn_id);
                });

        } else {
            // error message, no connection to coug
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot request status from Coug %i because there is no connection", vehicle_turn_id);
        }
    }


    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response){
        if (radio_connection[request->beacon_id]){

            radio_e_kill_client_->async_send_request(request,
                [this](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio", vehicle_turn_id);
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio failed", vehicle_turn_id);
                });

        } else if (modem_connection[request->beacon_id]) {
            
            modem_e_kill_client_->async_send_request(request,
                [this](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem", vehicle_turn_id);
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem failed", vehicle_turn_id);
                });

        } else {
            // error message, no connection to coug
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send e_kill command to Coug %i because there is no connection", vehicle_turn_id);
        }
    }

    void emergency_surface_callback(){
        if (modem_connection[request->beacon_id]){

            modem_e_surface_client_->async_send_request(request,
                [this](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    auto response = future.get();
                    if (response->success)
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through radio", vehicle_turn_id);
                    else
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through radio failed", vehicle_turn_id);
                });

        } else {
            // error message, no connection to coug
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command to Coug %i because there is no connection", vehicle_turn_id);
        }
    }



private:


    rclcpp::TimerBase::SharedPtr timer_;


    std::vector<int64_t> vehicles_in_mission_;


    size_t modem_coms_schedule_turn_index = 0;  


    int status_request_frequency;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}

