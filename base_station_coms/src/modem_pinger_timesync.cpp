#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "base_station_coms/seatrac_enums.hpp"
#include <chrono>
#include <cstdint>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "base_station_coms/coms_protocol.hpp"

using namespace std::chrono_literals;
using seatrac_interfaces::msg::ModemSend;
using namespace narval::seatrac;
using namespace cougars_coms;

using std::placeholders::_1;


/**
 * @brief Sends acoustic pings on a schedule
 * @author Ben Washburn
 * @date July 2026
 * 
 * Sends acoustic pings over the Seatrac X150 beacon in a round-robin
 * sequence. Every ping_delay_seconds, the next beacon ID in
 * vehicles_in_mission is pinged. After the final ID, the sequence restarts
 * at the beginning.
 * 
 * Publishes:
 * - modem_send (seatrac_interfaces/msg/ModemSend)
 */
class SeatracPinger : public rclcpp::Node
{
public:
    SeatracPinger() : Node("modem_pinger")
    {
        /**
         * @param ping_delay_seconds
         * 
         * The time between a ping from any vehicle in the water. For example
         * if Coug 1, Coug 2, and Coug 3 are pinging each other in that order,
         * then this would be the time between pings from Coug 1 and Coug 2.
         * WARNING: ping_delay_seconds should be the same for all vehicles.
         */
        this->declare_parameter<int>("ping_delay_seconds", 5);

        /**
         * @param vehicles_in_mission
         *
         * Ordered list of vehicle beacon IDs to ping. One vehicle is pinged per
         * ping_delay_seconds interval, then the list repeats from the beginning.
         */
        this->declare_parameter<std::vector<int64_t>>(
            "vehicles_in_mission", std::vector<int64_t>{2});

        /**
         * @param request_response
         * 
         * A boolean indicating whether or not to request a response signal from
         * the vehicle you're addressing this ping to. Response signals can be used
         * to calculate range between vehicles using 2-way-time-of-flight. 
         * 
         * WARNING: If request_response is true, the target_id must not be zero 
         * (between 1 and 16), indicating that this acoustic message is addressed 
         * to a specific vehicle and not all vehicles.
         */
        this->declare_parameter<bool>("request_response", false);

        /**
         * @param modem_offset_x / modem_offset_y / modem_offset_z
         *
         * Lever arm from the GPS antenna to the Seatrac modem, expressed in
         * the vehicle body frame (x: forward, y: right/starboard, z: down),
         * in meters. This accounts for the modem not being co-located with
         * the GPS antenna: the reported USBL range/bearing is relative to
         * the modem, but the only absolute position we have is the GPS fix.
         * The offset is rotated into the world frame using the beacon's
         * reported attitude before being applied.
         */
        this->declare_parameter<double>("modem_offset_x", 0.0);
        this->declare_parameter<double>("modem_offset_y", 0.0);
        this->declare_parameter<double>("modem_offset_z", 0.0);


        /**
         * @param origin_latitude / origin_longitude / origin_altitude
         *
         * The geographic coordinates of the base station's origin, expressed in
         * degrees and meters, respectively. This is used as the reference point
         * for all position calculations.
         */
        this->declare_parameter<double>("origin_latitude", 0.0);
        this->declare_parameter<double>("origin_longitude", 0.0);
        this->declare_parameter<double>("origin_altitude", 0.0);

        this->ping_delay_ = this->get_parameter("ping_delay_seconds").as_int();
        this->vehicles_in_mission_ =
            this->get_parameter("vehicles_in_mission").as_integer_array();
        this->request_response_ = this->get_parameter("request_response").as_bool();
        this->modem_offset_x_ = this->get_parameter("modem_offset_x").as_double();
        this->modem_offset_y_ = this->get_parameter("modem_offset_y").as_double();
        this->modem_offset_z_ = this->get_parameter("modem_offset_z").as_double();
        this->latest_fix_ = nullptr;

        this->latest_origin_ = std::make_shared<geographic_msgs::msg::GeoPoint>();
        this->latest_origin_->latitude = this->get_parameter("origin_latitude").as_double();
        this->latest_origin_->longitude = this->get_parameter("origin_longitude").as_double();
        this->latest_origin_->altitude = this->get_parameter("origin_altitude").as_double();



        modem_publisher_ = this->create_publisher<ModemSend>("modem_send", 10);

        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&SeatracPinger::listen_to_modem, this, _1)
        );

        this->gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
            {
                latest_fix_ = msg;
            }
        );

        this->origin_subscriber_ = this->create_subscription<geographic_msgs::msg::GeoPoint>(
            "origin", 10,
            [this](const geographic_msgs::msg::GeoPoint::SharedPtr msg)
            {
                latest_origin_ = msg;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Updated origin: latitude=%.9f, longitude=%.9f, altitude=%.3f",
                    msg->latitude, msg->longitude, msg->altitude);
            }
        );

        for (const int64_t vehicle_id : vehicles_in_mission_)
        {
            const std::string topic =
                "/coug" + std::to_string(vehicle_id) + "/modem_gps_odometry";
            vehicle_odometry_publishers_.try_emplace(
                vehicle_id,
                this->create_publisher<nav_msgs::msg::Odometry>(topic, 10));
        }

        // wait 5 seconds before starting the ping scheduler to allow the modem to initialize
        rclcpp::sleep_for(std::chrono::seconds(5));

        if (ping_delay_ > 0 && !vehicles_in_mission_.empty())
        {
            this->ping_timer_ = this->create_wall_timer(
                std::chrono::seconds(ping_delay_),
                std::bind(&SeatracPinger::ping_next_vehicle, this)
            );
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "Ping scheduler disabled: ping_delay_seconds must be positive and vehicles_in_mission must not be empty.");
        }
        RCLCPP_INFO(this->get_logger(), "SeatracPinger node initialized with origin (%.9f, %.9f, %.3f).",
            latest_origin_->latitude, latest_origin_->longitude, latest_origin_->altitude);
    }

private:
    int ping_delay_;
    std::vector<int64_t> vehicles_in_mission_;
    std::size_t next_vehicle_index_ = 0;
    bool request_response_;
    double modem_offset_x_;
    double modem_offset_y_;
    double modem_offset_z_;
    rclcpp::Publisher<ModemSend>::SharedPtr modem_publisher_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr origin_subscriber_;
    std::unordered_map<
        int64_t, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>
        vehicle_odometry_publishers_;
    sensor_msgs::msg::NavSatFix::SharedPtr latest_fix_;
    geographic_msgs::msg::GeoPoint::SharedPtr latest_origin_;

    void ping_next_vehicle()
    {
        send_ping(vehicles_in_mission_[next_vehicle_index_]);
        next_vehicle_index_ = (next_vehicle_index_ + 1) % vehicles_in_mission_.size();
    }

    void send_ping(int64_t target_id)
    {
        RCLCPP_INFO(
            this->get_logger(), "Ping coug%lld",
            static_cast<long long>(target_id));

        auto data = RequestTimestamp();
        auto request = ModemSend();
        request.msg_id = CID_E::CID_DAT_SEND;
        request.dest_id = target_id;
        request.msg_type = AMSGTYPE_E::MSG_REQU;
        std::memcpy(request.packet_data.data(), &data, sizeof(data));
        modem_publisher_->publish(request);
    }

    void listen_to_modem(const seatrac_interfaces::msg::ModemRec::SharedPtr msg)
    {
        if (msg->packet_len == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Received empty packet from vehicle %d", msg->src_id);
            return;
        }

        COUG_MSG_ID id = (COUG_MSG_ID)msg->packet_data[0];
        switch (id)
        {
        case COUG_MSG_ID::TIMESTAMP:
            RCLCPP_INFO(this->get_logger(), "Received timestamp response from vehicle %d", msg->src_id);
            this->publish_vehicle_location(msg);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Received unknown message from vehicle %d with COUG_MSG_ID %d", msg->src_id, id);
            break;
        }
    }

    void publish_vehicle_location(const seatrac_interfaces::msg::ModemRec::SharedPtr msg)
    {
        const auto publisher = vehicle_odometry_publishers_.find(msg->src_id);
        if (publisher == vehicle_odometry_publishers_.end())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received location from vehicle %d, which is not in vehicles_in_mission.",
                msg->src_id);
            return;
        }

        if (latest_fix_ == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "No GPS fix available to publish vehicle location.");
            return;
        }
        if (latest_origin_ == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "No origin available to publish vehicle location.");
            return;
        }
        TimeStamp* timestamp_msg = reinterpret_cast<TimeStamp*>(msg->packet_data.data());


        float beacon_roll = msg->attitude_roll / 10;
        float beacon_pitch = msg->attitude_pitch / 10;
        float beacon_yaw = msg->attitude_yaw / 10;

        float range = msg->range_dist / 10;
        float azimuth = msg->usbl_azimuth / 10;
        float elevation = msg->usbl_elevation / 10;

        double x;
        double y;
        double z;
        beacon_spherical_to_cartesian(range, azimuth, elevation, x, y, z);
        rotate_from_beacon_frame(beacon_roll, beacon_pitch, beacon_yaw, x, y, z);

        // The USBL range/bearing is relative to the modem, not the GPS
        // antenna. Rotate the modem's body-frame lever arm (from GPS to
        // modem) into the world frame using the same attitude, and add it
        // so the fix-based translation below effectively originates from
        // the modem's true position rather than the GPS antenna's.
        double offset_x = modem_offset_x_;
        double offset_y = modem_offset_y_;
        double offset_z = modem_offset_z_;
        rotate_from_beacon_frame(beacon_roll, beacon_pitch, beacon_yaw, offset_x, offset_y, offset_z);
        x += offset_x;
        y += offset_y;
        z += offset_z;

        translate_to_world_frame(*latest_fix_, *latest_origin_, x, y, z);


        nav_msgs::msg::Odometry vehicle_odometry_msg;
        vehicle_odometry_msg.header.stamp.sec = timestamp_msg->seconds;
        vehicle_odometry_msg.header.stamp.nanosec = timestamp_msg->nanoseconds;
        vehicle_odometry_msg.header.frame_id = "map";
        vehicle_odometry_msg.child_frame_id = "coug" + std::to_string(msg->src_id);
        vehicle_odometry_msg.pose.pose.position.x = x;
        vehicle_odometry_msg.pose.pose.position.y = y;
        vehicle_odometry_msg.pose.pose.position.z = z;

        const std::string odometry_yaml = nav_msgs::msg::to_yaml(vehicle_odometry_msg);
        RCLCPP_INFO(
            this->get_logger(), "Publishing vehicle odometry:\n%s",
            odometry_yaml.c_str());

        publisher->second->publish(vehicle_odometry_msg);
    }

    static void beacon_spherical_to_cartesian(
        double range, double azimuth, double elevation,
        double &x, double &y, double &z)
    {
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const double azimuth_radians = azimuth * degrees_to_radians;
        const double elevation_radians = elevation * degrees_to_radians;
        const double horizontal_range = range * std::cos(elevation_radians);
        x = horizontal_range * std::cos(azimuth_radians);
        y = horizontal_range * std::sin(azimuth_radians);
        z = range * std::sin(elevation_radians);
    }

    static void rotate_from_beacon_frame(
        double roll, double pitch, double yaw,
        double &x, double &y, double &z)
    {
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        const double roll_radians = roll * degrees_to_radians;
        const double pitch_radians = pitch * degrees_to_radians;
        const double yaw_radians = yaw * degrees_to_radians;
        const double sin_roll = std::sin(roll_radians);
        const double cos_roll = std::cos(roll_radians);
        const double sin_pitch = std::sin(pitch_radians);
        const double cos_pitch = std::cos(pitch_radians);
        const double sin_yaw = std::sin(yaw_radians);
        const double cos_yaw = std::cos(yaw_radians);

        // The reported attitude rotates world-frame vectors into the beacon
        // frame. Apply its transpose to rotate this beacon-frame vector back
        // into the world frame: R_x(-roll) R_y(-pitch) R_z(-yaw).
        const double yawed_x = cos_yaw * x + sin_yaw * y;
        const double yawed_y = -sin_yaw * x + cos_yaw * y;
        const double pitched_x = cos_pitch * yawed_x - sin_pitch * z;
        const double pitched_z = sin_pitch * yawed_x + cos_pitch * z;
        x = pitched_x;
        y = cos_roll * yawed_y + sin_roll * pitched_z;
        z = -sin_roll * yawed_y + cos_roll * pitched_z;
    }

    static void translate_to_world_frame(
        const sensor_msgs::msg::NavSatFix &fix,
        const geographic_msgs::msg::GeoPoint &origin,
        double &x, double &y, double &z)
    {
        constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
        constexpr double earth_radius_meters = 6371000.0;
        const double origin_latitude_radians = origin.latitude * degrees_to_radians;
        const double latitude_delta_radians =
            (fix.latitude - origin.latitude) * degrees_to_radians;
        const double longitude_delta_radians =
            (fix.longitude - origin.longitude) * degrees_to_radians;
        x += earth_radius_meters * longitude_delta_radians * std::cos(origin_latitude_radians);
        y += earth_radius_meters * latitude_delta_radians;
        z += fix.altitude - origin.altitude;
    }
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeatracPinger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}