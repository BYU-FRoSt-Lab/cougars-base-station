#ifndef VEHICLE_MODEM_CONNECTION_HPP
#define VEHICLE_MODEM_CONNECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "geographic_msgs/msg/route_network.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cougars_interfaces/msg/system_control.hpp"
#include "base_station_coms/coms_protocol.hpp"
#include "base_station_coms/seatrac_enums.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

class VehicleModemConnection {
public:
    VehicleModemConnection(
        int vehicle_id,
        rclcpp::Node* node,
        rclcpp::Logger logger,
        rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher,
        int max_missed_messages = 2,
        bool publish_link_status = true)
        : vehicle_id_(vehicle_id),
          modem_connection_status_(true),
          radio_connection_status_(false),
          wifi_connection_status_(false),
          messages_missed_(max_missed_messages - 1),
          max_missed_messages_(max_missed_messages),
          publish_link_status_(publish_link_status),
          last_message_time_(node->now()),
          logger_(logger),
          modem_publisher_(modem_publisher) {

        std::string namespace_name = "coug" + std::to_string(vehicle_id);

        this->load_mission_subscriber_ = node->create_subscription<geographic_msgs::msg::RouteNetwork>(
            namespace_name + "/load_mission", 10,
            [this](const geographic_msgs::msg::RouteNetwork::SharedPtr msg) {
                this->load_mission_callback(msg);
            }
        );
        
        this->start_mission_subscriber_ = node->create_subscription<cougars_interfaces::msg::SystemControl>(
            namespace_name + "/start_mission", 10,
            [this](const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
                this->start_mission_callback(msg);
            }
        );

        this->emergency_kill_subscriber_ = node->create_subscription<std_msgs::msg::Bool>(
            namespace_name + "/emergency_kill", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->emergency_kill_callback(msg);
            }
        );

        this->emergency_surface_subscriber_ = node->create_subscription<std_msgs::msg::Bool>(
            namespace_name + "/emergency_surface", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->emergency_surface_callback(msg);
            }
        );

        this->connections_subscriber_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            namespace_name + "/link_status", 10,
            [this](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
                this->connections_callback(msg);
            }
        );

        this->modem_connections_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            namespace_name + "/link_status", 10
        );
    }

    void handle_modem_message(const seatrac_interfaces::msg::ModemRec& msg, rclcpp::Time current_time) {
        last_message_time_ = current_time;
        reset_missed_messages();

        if (msg.packet_len == 0) {
            return;
        }

        using namespace cougars_coms;
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];

        switch(id) {
            default: break;
            case EMPTY: break;
            case VEHICLE_STATUS: {
                RCLCPP_DEBUG(logger_, "Vehicle %d: received status message", this->get_vehicle_id());
            } break;
            case CONFIRM_EMERGENCY_KILL: {
                RCLCPP_INFO(logger_, "Vehicle %d: confirmed emergency kill", this->get_vehicle_id());
            } break;
            case CONFIRM_EMERGENCY_SURFACE: {
                RCLCPP_INFO(logger_, "Vehicle %d: confirmed emergency surface", this->get_vehicle_id());
            } break;
            case TIMESTAMP: {
                const TimeStamp* timestamp_msg =
                    reinterpret_cast<const TimeStamp*>(msg.packet_data.data());
                RCLCPP_DEBUG(logger_, "Vehicle %d: received timestamp message - seconds: %u, nanoseconds: %u", this->get_vehicle_id(), timestamp_msg->seconds, timestamp_msg->nanoseconds);
            } break;
        }

        if (publish_link_status_) {
            this->check_modem_connection();
        }
    }

        //used by the service callback functions to publish messages to the cougs
    void send_acoustic_message(int message_len, uint8_t* message, narval::seatrac::AMSGTYPE_E msg_type) {
        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = narval::seatrac::CID_DAT_SEND;
        request.dest_id = (uint8_t)this->get_vehicle_id();
        request.msg_type = msg_type;
        request.packet_len = (uint8_t)std::min(message_len, 30);
        // request.insert_timestamp = true;
        std::memcpy(request.packet_data.data(), message, request.packet_len);
       
        this->modem_publisher_->publish(request);
        this->increment_missed_messages();
        if (publish_link_status_) {
            this->check_modem_connection();
        }
    }

    void check_modem_connection() {
        diagnostic_msgs::msg::DiagnosticStatus status_msg;
        status_msg.name = "Coug" + std::to_string(this->get_vehicle_id()) + " Modem Connection";
        status_msg.hardware_id = "modem";
        if (this->is_connected()) {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status_msg.message = "Connected";
        } else {
            status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status_msg.message = "Disconnected";
        }
        diagnostic_msgs::msg::KeyValue last_message_time;
        last_message_time.key = "last_message_time";
        last_message_time.value = std::to_string(this->get_last_message_time().seconds());
        status_msg.values.push_back(last_message_time);

        this->modem_connections_publisher_->publish(status_msg);
    }

    void mark_missed_message_and_publish() {
        increment_missed_messages();
        check_modem_connection();
    }

    void connections_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
        if (msg->hardware_id == "radio") {
            radio_connection_status_ = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
        } else if (msg->hardware_id == "wifi") {
            wifi_connection_status_ = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
        } else if (msg->hardware_id == "modem") {
            modem_connection_status_ = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
            if (modem_connection_status_) {
                messages_missed_ = 0;
            }
        }
    }

    void increment_missed_messages() {
        messages_missed_++;
        if (this->get_missed_messages() >= max_missed_messages_ && this->is_connected()) {
            RCLCPP_WARN(logger_, "Vehicle %d has missed %d or more messages, marking as disconnected",
                        this->get_vehicle_id(), max_missed_messages_);
            modem_connection_status_ = false;
        }
    }

    void reset_missed_messages() {
        if (this->get_missed_messages() > 0 || !this->is_connected()) {
            messages_missed_ = 0;
            if (!this->is_connected()) {
                modem_connection_status_ = true;
                RCLCPP_INFO(logger_, "Vehicle %d reconnected", this->get_vehicle_id());
            }
        }
    }

    bool is_connected() const {
        return modem_connection_status_;
    }

    int get_missed_messages() const {
        return messages_missed_;
    }

    rclcpp::Time get_last_message_time() const {
        return last_message_time_;
    }

    int get_vehicle_id() const {
        return vehicle_id_;
    }

    

private:
    void load_mission_callback(const geographic_msgs::msg::RouteNetwork::SharedPtr msg) {
        (void)msg;
        if (this->is_connected() && !this->radio_connection_status_ && !this->wifi_connection_status_) {
            RCLCPP_WARN(logger_, "Loading mission for vehicle %d over modem", this->get_vehicle_id());
        }
    }

    void start_mission_callback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
        if (!this->is_connected() || radio_connection_status_ || wifi_connection_status_) {
            return;
        }

        RCLCPP_WARN(logger_, "Starting mission for vehicle %d over modem", this->get_vehicle_id());
        cougars_coms::Init init_msg;

        // construct the bitmask based on the incoming message fields
        init_msg.init_bitmask = 0;
        init_msg.init_bitmask |= msg->start.data ? 0x01 : 0x00;
        init_msg.init_bitmask |= msg->rosbag_flag.data ? 0x02 : 0x00;
        init_msg.init_bitmask |= msg->thruster_arm.data ? 0x04 : 0x00;
        init_msg.init_bitmask |= msg->dvl_acoustics.data ? 0x08 : 0x00;

        // automatically sets the max size for the string copy to prevent overflow, ensuring null termination
        std::snprintf(
            init_msg.rosbag_prefix,
            sizeof(init_msg.rosbag_prefix),
            "%s",
            msg->rosbag_prefix.c_str()
        );

        this->send_acoustic_message(
            sizeof(init_msg),
            reinterpret_cast<uint8_t*>(&init_msg),
            narval::seatrac::MSG_OWAY
        );
    }

    void emergency_kill_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !this->is_connected() || radio_connection_status_ || wifi_connection_status_) {
            return;
        }

        RCLCPP_WARN(logger_, "Emergency kill for vehicle %d over modem", this->get_vehicle_id());
        cougars_coms::EmergencyKill e_kill_msg;
        send_acoustic_message(
            sizeof(e_kill_msg),
            reinterpret_cast<uint8_t*>(&e_kill_msg),
            narval::seatrac::MSG_OWAY
        );
    }

    void emergency_surface_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !this->is_connected() || radio_connection_status_ || wifi_connection_status_) {
            return;
        }

        RCLCPP_WARN(logger_, "Emergency surface for vehicle %d over modem", this->get_vehicle_id());
        cougars_coms::EmergencySurface e_surface_msg;
        send_acoustic_message(
            sizeof(e_surface_msg),
            reinterpret_cast<uint8_t*>(&e_surface_msg),
            narval::seatrac::MSG_OWAY
        );
    }

    int vehicle_id_;
    bool modem_connection_status_;
    bool radio_connection_status_;
    bool wifi_connection_status_;
    int messages_missed_;
    int max_missed_messages_;
    bool publish_link_status_;
    rclcpp::Time last_message_time_;
    rclcpp::Logger logger_;
    

    rclcpp::Subscription<geographic_msgs::msg::RouteNetwork>::SharedPtr load_mission_subscriber_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr start_mission_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_kill_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_surface_subscriber_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr modem_connections_publisher_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr connections_subscriber_;
};

#endif // VEHICLE_MODEM_CONNECTION_HPP
