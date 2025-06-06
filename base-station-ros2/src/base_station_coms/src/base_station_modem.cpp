
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"
#include "std_msgs/msg/bool.hpp"



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


class ModemComs : public rclcpp::Node {
public:
    ModemComs() : Node("base_station_modem") {


        this->declare_parameter<int>("base_station_beacon_id", 15);
        this->base_station_beacon_id_ = this->get_parameter("base_station_beacon_id").as_int();

        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();


        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ModemComs::listen_to_modem, this, _1)
        );
        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);


        request_status_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_status_request",
            std::bind(&ModemComs::status_request_callback, this, _1, _2)
        );

        emergency_kill_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_e_kill",
            std::bind(&ModemComs::emergency_kill_callback, this, _1, _2)
        );


        emergency_surface_service_ = this->create_service<base_station_interfaces::srv::BeaconId>(
            "modem_e_surface",
            std::bind(&ModemComs::emergency_surface_callback, this, _1, _2)
        );

        this->status_publisher_ = this->create_publisher<base_station_interfaces::msg::Status>("status", 10);

        this->modem_connections_publisher_ = this->create_publisher<base_station_interfaces::msg::Connections>("connections", 10);

        this->confirm_e_kill_publisher_ = this->create_publisher<std_msgs::msg::Bool>("confirm_e_kill", 10);

        this->confirm_e_surface_ = this->create_publisher<std_msgs::msg::Bool>("confirm_e_surface", 10);

        RCLCPP_INFO(this->get_logger(), "base station coms node started");

        

    }

    // listens to ModemRec message and processes msg according to the msg id
    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        last_message_time_[id] = this->now();

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
                emergency_kill_confirmed(msg);
            } break;
        }
    }


    // Sends emergency kill signal to coug specified in request
    void emergency_kill_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)
    {
        EmergencyKill e_kill_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_kill_msg), (uint8_t*)&e_kill_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Kill Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }


    // Sends emergency surface signal to coug specified in request
    void emergency_surface_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)      
    {
        EmergencySurface e_surface_msg;
        send_acoustic_message(request->beacon_id, sizeof(e_surface_msg), (uint8_t*)&e_surface_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Surface Signal Sent to Coug %i", request->beacon_id);
        response->success = true;
    }

    // requests status of coug specified in request
    void status_request_callback(const std::shared_ptr<base_station_interfaces::srv::BeaconId::Request> request,
                                    std::shared_ptr<base_station_interfaces::srv::BeaconId::Response> response)
    {
        check_modem_connections();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting Status of Coug %i", request->beacon_id);

        RequestStatus request_status_msg;
        send_acoustic_message(request->beacon_id, sizeof(request_status_msg), (uint8_t*)&request_status_msg, MSG_OWAY);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Requesting Status of Coug %i", request->beacon_id);
        response->success = true;
    }

    void recieve_status(seatrac_interfaces::msg::ModemRec msg) {
        
        const VehicleStatus* status = reinterpret_cast<const VehicleStatus*>(msg.packet_data.data());
        auto status_msg = base_station_interfaces::msg::Status();
        status_msg.vehicle_id = msg.src_id;
        status_msg.x = status->x;
        status_msg.y = status->y;
        status_msg.depth = status->depth;
        status_msg.heading = status->heading;
        status_msg.dvl_vel = status->dvl_vel;
        status_msg.battery_voltage = status->battery_voltage;
        status_msg.dvl_running = status->dvl_running;
        status_msg.gps_connection = status->gps_connection;
        status_msg.leak_detection = status->leak_detection;
        status_msg.waypoint = status->waypoint;

        this->status_publisher_->publish(status_msg);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coug %i Status:", msg.src_id);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x pos: %i", status->x);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "y pos: %i", status->y);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth: %i", status->depth);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Heading: %i", status->heading);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint: %i", status->waypoint);

    }

    // 
    void emergency_kill_confirmed(seatrac_interfaces::msg::ModemRec msg){
        auto success_msg = std_msgs::msg::Bool();
        success_msg.data = msg.packet_data[0];
        this->confirm_e_kill_publisher_->publish(success_msg);
        if (success_msg.data){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency kill command was successful for Coug %i", msg.src_id);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency kill command failed for Coug %i", msg.src_id);
        }
    }

    void emergency_surface_confirmed(seatrac_interfaces::msg::ModemRec msg){
        auto success_msg = std_msgs::msg::Bool();
        success_msg.data = msg.packet_data[0];
        this->confirm_e_kill_publisher_->publish(success_msg);
        if (success_msg.data){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency surface command was successful for Coug %i", msg.src_id);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency surface command failed for Coug %i", msg.src_id);
        }
    }


    void check_modem_connections(double timeout_sec = 10.0) {
        rclcpp::Time now = this->now();
        std::vector<bool> connections;
        std::vector<uint32_t> last_ping;


        for (auto vehicle_id : vehicles_in_mission_) {
            auto it = last_message_time_.find(vehicle_id);
            if (it != last_message_time_.end()) {
                double dt = (now - it->second).seconds();
                connections.push_back(dt < timeout_sec);
                last_ping.push_back(static_cast<uint32_t>(dt));

            } else {
                modem_connection[vehicle_id] = false;
            }
        }
            base_station_interfaces::msg::Connections msg;
            msg.header.stamp = now;
            msg.connection_type = 0; // 0 for acoustic modem
            msg.connections = connections;
            msg.last_ping = last_ping;
            modem_connections_publisher_->publish(msg);

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

    rclcpp::Publisher<base_station_interfaces::msg::Status>::SharedPtr status_publisher_;
    rclcpp::Publisher<base_station_interfaces::msg::Connections>::SharedPtr modem_connections_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirm_e_kill_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirm_e_surface_;

    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_surface_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr emergency_kill_service_;
    rclcpp::Service<base_station_interfaces::srv::BeaconId>::SharedPtr request_status_service_;


    rclcpp::TimerBase::SharedPtr timer_;


    std::vector<int64_t> vehicles_in_mission_;

    std::unordered_map<int, rclcpp::Time> last_message_time_;



    size_t modem_coms_schedule_turn_index = 0;  


    int status_request_frequency;


    int base_station_beacon_id_;

    std::unordered_map<int,bool> modem_connection;




};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ModemComs>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}

