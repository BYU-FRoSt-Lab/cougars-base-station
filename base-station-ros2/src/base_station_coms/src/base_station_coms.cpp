
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"




#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"


#include <iostream>
#include <chrono>
#include <memory>
#include <unordered_map>


using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;


using std::placeholders::_1;
using std::placeholders::_2;


class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("base_station_coms") {


        this->declare_parameter<int>("status_request_frequency_seconds", 5);
        this->status_request_frequency = this->get_parameter("status_request_frequency_seconds").as_int();


        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();


        // client for the base_station_radio node. Requests that the radio sends an emergency kill command to a specific coug
        radio_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_e_kill"
        );
        
        // client for the base_station_modem node. Requests that the modem sends an emergency kill command to a specific coug
        modem_e_kill_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_kill"
        );

        // client for the base_station_modem node. Requests that the modem sends an emergency surface command to a specific coug
        modem_e_surface_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_e_surface"
        );

        // client for the base_station_modem node. Requests that the modem gets the status of a coug
        modem_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "modem_status_request"
        );

        // client for the base_station_radio node. Requests that the radio gets the status of a coug
        radio_status_request_client_ = this->create_client<base_station_interfaces::srv::BeaconId>(
            "radio_status_request"
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

        timer_ = this->create_wall_timer(
                    std::chrono::seconds(status_request_frequency), std::bind(&ComsNode::request_status_callback, this));

        connections_subscriber_ = this->create_subscription<base_station_interfaces::msg::Connections>(
            "connections",
            10,
            std::bind(&ComsNode::listen_to_connections, this, _1)
        );

        std::ostringstream ss;
        ss << "Vehicle ids in mission: ";
        for(int64_t i: vehicles_in_mission_) ss << i << ", ";
        RCLCPP_INFO(this->get_logger(), "base station started");
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        modem_connection[0] = true;
        radio_connection[0] = true; 

        for(int64_t i: vehicles_in_mission_){
            modem_connection[i] = true;
            radio_connection[i] = false; // radio connection is not established by default
        }




    }

    void listen_to_connections(const base_station_interfaces::msg::Connections::SharedPtr msg) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Updating connections for cougs");

        size_t n = std::min(vehicles_in_mission_.size(), msg->connections.size());
        for (size_t i = 0; i < n; ++i) {
            int beacon_id = vehicles_in_mission_[i];
            if (msg->connection_type == 1) {
                radio_connection[beacon_id] = msg->connections[i];
            } else if (msg->connection_type == 0) {
                modem_connection[beacon_id] = msg->connections[i];
            }
        }
    }

    void request_status_callback(){
        if (vehicles_in_mission_.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No vehicles in mission — skipping status request.");
            return;
}   
        vehicle_id_index += 1;
        if (vehicle_id_index+1>=vehicles_in_mission_.size())
            vehicle_id_index = 0;
        vehicle_id_index = 0;
        auto request = std::make_shared<base_station_interfaces::srv::BeaconId::Request>();
        int beacon_id = vehicles_in_mission_[vehicle_id_index];
        request->beacon_id = beacon_id;

        if (radio_connection[beacon_id]){

            if (!radio_status_request_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Radio status request service not available for Coug %i. Skipping request.", beacon_id);
                return;
            }

            auto result_future = radio_status_request_client_->async_send_request(request,
                [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through radio", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through radio failed", beacon_id);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception while requesting status from Coug %i: %s", beacon_id, e.what());
                    }
                });

        } else if (modem_connection[beacon_id]) {

            if (!modem_status_request_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Modem status request service not available for Coug %i. Skipping request.", beacon_id);
                return;
            }

            auto result_future = modem_status_request_client_->async_send_request(request,
                [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Status request from Coug %i through modem failed", beacon_id);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception while requesting status from Coug %i: %s", beacon_id, e.what());
                    }
                });

        } else {
            // error message, no connection to coug
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot request status from Coug %i because there is no connection", beacon_id);
        }
    }


    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response){
        int beacon_id = request->beacon_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), beacon_id) != vehicles_in_mission_.end() || beacon_id == BEACON_ALL){
            if (radio_connection[beacon_id]){

                radio_e_kill_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through radio failed", beacon_id);
                    });
                response->success = true;

            } else if (modem_connection[beacon_id]) {
                
                modem_e_kill_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kill command sent to Coug %i through modem failed", beacon_id);
                    });
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send e_kill command to Coug %i because there is no connection", beacon_id);
                response->success = false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send kill command. There is no Vehicle with ID %i", beacon_id);
            response->success = false;
        }
    }

    void emergency_surface_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response){
        int beacon_id = request->beacon_id;
        if (std::find(vehicles_in_mission_.begin(), vehicles_in_mission_.end(), beacon_id) != vehicles_in_mission_.end() || beacon_id == BEACON_ALL){
            if (modem_connection[beacon_id]){

                modem_e_surface_client_->async_send_request(request,
                    [this, beacon_id](rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedFuture future) {
                        auto response = future.get();
                        if (response->success)
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through modem", beacon_id);
                        else
                            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Surface command sent to Coug %i through modem", beacon_id);
                    });
                response->success = true;

            } else {
                // error message, no connection to coug
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command to Coug %i because there is no connection", beacon_id);
                response->success = false;
            }
        } else{
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Cannot send surface command. There is no Vehicle with ID %i", beacon_id);
            response->success = false;
        }
    }



private:


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<base_station_interfaces::msg::Connections>::SharedPtr connections_subscriber_;

    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr radio_e_kill_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_e_kill_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_e_surface_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr radio_status_request_client_;
    rclcpp::Client<base_station_interfaces::srv::BeaconId>::SharedPtr modem_status_request_client_;

    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;

    std::unordered_map<int,bool> radio_connection;
    std::unordered_map<int,bool> modem_connection;

    std::vector<int64_t> vehicles_in_mission_;


    size_t vehicle_id_index = 0;  


    int status_request_frequency;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}

