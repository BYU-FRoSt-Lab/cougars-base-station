

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"

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
    ComsNode() : Node("cougars_coms") {

        this->declare_parameter<int>("base_station_beacon_id", 15);

        this->base_station_beacon_id_ = this->get_parameter("base_station_beacon_id").as_int();

        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );
        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);


        emergency_kill_service_ = this->create_service<std_srvs::srv::SetBool>(
            "emergency_kill_service",
            std::bind(&ComsNode::emergency_kill_callback, this, _1, _2)
        );

        start_mission_service_ = this->create_service<std_srvs::srv::SetBool>(
            "start_mission_service",
            std::bind(&ComsNode::start_mission_callback, this, _1, _2)
        );



    }

    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        switch(id) {
            default: break;
            case EMPTY: break;
        }
    }


    void emergency_kill_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        EmergencyKill e_kill_msg;
        send_acoustic_message(BEACON_ALL, sizeof(e_kill_msg), (uint8_t*)&e_kill_msg);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency Kill Signal Sent");
    }


    void start_mission_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        
    }


    void send_acoustic_message(int target_id, int message_len, uint8_t* message) {
        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = CID_DAT_SEND;
        request.dest_id = (uint8_t)target_id;
        request.msg_type = MSG_OWAY;
        request.packet_len = (uint8_t)std::min(message_len, 31);
        std::memcpy(&request.packet_data, message, request.packet_len);
        
        this->modem_publisher_->publish(request);
    }


private:

    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;


    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_kill_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_mission_service_;

    int base_station_beacon_id_;

};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}