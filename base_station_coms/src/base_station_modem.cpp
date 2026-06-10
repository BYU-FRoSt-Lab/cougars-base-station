
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "base_station_interfaces/srv/beacon_id.hpp"
#include "base_station_interfaces/msg/status.hpp"
#include "base_station_interfaces/msg/connections.hpp"
#include "std_msgs/msg/bool.hpp"
#include "base_station_interfaces/msg/console_log.hpp"
#include "base_station_interfaces/srv/init.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geographic_msgs/msg/route_network.hpp"

#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"
#include "base_station_coms/vehicle_modem_connection.hpp"

#include <iostream>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>


using namespace std::literals::chrono_literals;
using namespace cougars_coms;
using namespace narval::seatrac;


using std::placeholders::_1;
using std::placeholders::_2;


class ModemComs : public rclcpp::Node {
public:
    ModemComs() : Node("base_station_modem") {

        this->declare_parameter<std::vector<int64_t>>("vehicles_in_mission", {1,2,5});
        this->vehicles_in_mission_ = this->get_parameter("vehicles_in_mission").as_integer_array();

        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ModemComs::listen_to_modem, this, _1)
        );

        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>(
            "modem_send", 10
        );

        RCLCPP_INFO(this->get_logger(), "base station coms node started");

        for (int64_t vehicle_id : vehicles_in_mission_) {
            this->vehicle_modems_[vehicle_id] = std::make_shared<VehicleModemConnection>(
                vehicle_id,
                this,
                this->get_logger(),
                modem_publisher_,
                max_missed_messages_
            );
        }

    }
    

    void listen_to_modem(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
        int vehicle_id = msg->src_id;

        auto vehicle_it = vehicle_modems_.find(vehicle_id);
        if (vehicle_it == vehicle_modems_.end()) {
            RCLCPP_WARN(this->get_logger(), "Received message from unknown vehicle ID: %d", vehicle_id);
            return;
        }

        vehicle_it->second->handle_modem_message(*msg, this->now());
    }





private:

    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;

    std::vector<int64_t> vehicles_in_mission_;
    std::unordered_map<int, std::shared_ptr<VehicleModemConnection>> vehicle_modems_;

    int max_missed_messages_ = 2;
};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ModemComs>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}
