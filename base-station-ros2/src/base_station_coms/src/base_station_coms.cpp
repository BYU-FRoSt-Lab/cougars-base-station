
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


        this->declare_parameter<int>("base_station_beacon_id", 15);
        this->base_station_beacon_id_ = this->get_parameter("base_station_beacon_id").as_int();


        this->declare_parameter<int>("status_request_frequency_seconds", 15);
        this->status_request_frequency = this->get_parameter("status_request_frequency_seconds").as_int();


        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();


        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );
        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);




        emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "emergency_kill_service",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );


        emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "emergency_surface_service",
            std::bind(&ComsNode::emergency_surface_callback, this, _1, _2)
        );


        timer_ = this->create_wall_timer(
                    std::chrono::seconds(status_request_frequency), std::bind(&ComsNode::request_status_callback, this));


        std::ostringstream ss;
        ss << "Vehicle ids in mission: ";
        for(int64_t i: vehicles_in_mission_) ss << i << ", ";
        RCLCPP_INFO(this->get_logger(), "base station coms node started");
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());




    }


    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        switch(id) {
            default: break;
            case EMPTY: break;
            case VEHICLE_STATUS:{
                recieve_status(msg);
            } break;
            case CONFIRM_EMERGENCY_KILL: {
                emergency_kill_confirmed(msg);
            } break;
            case CONFIRM_EMERGENCY_SURFACE: {
                emergency_surface_confirmed(msg);
            } break;
        }
    }


    // Sends emergency kill signal to coug specified in request
    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)
    {
        EmergencyKill e_kill_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_kill_msg), (uint8_t*)&e_kill_msg, MSG_OWAY);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Kill Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }


    // Sends emergency surface signal to coug specified in request
    void emergency_surface_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)      
    {
        EmergencySurface e_surface_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_surface_msg), (uint8_t*)&e_surface_msg, MSG_OWAY);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Surface Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }


    // periodically requests status of cougs
    void request_status_callback() {
       
        modem_coms_schedule_turn_index += 1;
        if (modem_coms_schedule_turn_index+1>=vehicles_in_mission_.size())
            modem_coms_schedule_turn_index = 0;


        RequestStatus request;
        int vehicle_turn_id = vehicles_in_mission_[modem_coms_schedule_turn_index];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting status from Coug %i", vehicle_turn_id);
        send_acoustic_message(vehicle_turn_id, sizeof(request), (uint8_t*)&request, MSG_REQX);
       
    }


    void emergency_kill_confirmed(seatrac_interfaces::msg::ModemRec msg){
        bool success = msg.packet_data[0];
        if (success){


            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency kill command was successful.");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency kill command failed.");
        }


    }


    void emergency_surface_confirmed(seatrac_interfaces::msg::ModemRec msg){
        bool success = msg.packet_data[0];
        if (success){


            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency surface command was successful.");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency surface command failed.");
        }
    }


    void recieve_status(seatrac_interfaces::msg::ModemRec msg) {
       
        const VehicleStatus* status = reinterpret_cast<const VehicleStatus*>(msg.packet_data.data());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coug %i Status:", msg.src_id);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x pos: %i", status->x);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "y pos: %i", status->y);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth: %i", status->depth);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Heading: %i", status->heading);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moos_waypoint: %i", status->moos_waypoint);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moos_behavior_number: %i", status->moos_behavior_number);


    }
   
    //used by the service callback functions to publish messages to the cougs
    void send_acoustic_message(int target_id, int message_len, uint8_t* message, AMSGTYPE_E msg_type) {


        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = CID_DAT_SEND;
        request.dest_id = (uint8_t)target_id;
        request.msg_type = msg_type;
        request.packet_len = (uint8_t)std::min(message_len, 31);
        // request.insert_timestamp = true;
        std::memcpy(&request.packet_data, message, request.packet_len);
       
        this->modem_publisher_->publish(request);
    }




private:


    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;


    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;


    rclcpp::TimerBase::SharedPtr timer_;


    std::vector<int64_t> vehicles_in_mission_;


    size_t modem_coms_schedule_turn_index = 0;  


    int status_request_frequency;


    int base_station_beacon_id_;




};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}

