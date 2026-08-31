#include <rclcpp/rclcpp.hpp>
#include <base_station_interfaces/msg/u_command_base.hpp>
#include <base_station_interfaces/msg/console_log.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <sstream>
#include <iomanip>

#define KEYCODE_SPACE 0x20
#define KEYCODE_DOT 0x2E

class TeleopCommand : public rclcpp::Node
{
public:
  TeleopCommand();
  void keyLoop();

private:
  void switchVehicle();
  void keyPressCallback(const std_msgs::msg::String::SharedPtr msg);
  void processKey(char key);
  void publishCommand();
  void publishConsoleLog(const std::string& message, int vehicle_id = 0);
  void publishStatus();  // Publishes current vehicle/thruster/publishing/hard-turn state (latched) for the GUI tab
  bool tryApplyChange(const std::function<void()> &apply_fn);
  void applyHardTurn(double direction);  // direction: -1.0 for left, +1.0 for right
  void timerCallback();  // New timer callback for rate-limited publishing

  double fin1_, fin2_, fin3_, max_fin_value_;
  int thruster_value_, fin_change_speed_;
  std::string vehicle_name_;
  int vehicle_id_;
  std::vector<int64_t> vehicles_in_mission_;
  size_t current_vehicle_index_;
  base_station_interfaces::msg::UCommandBase last_command_msg_;
  rclcpp::Publisher<base_station_interfaces::msg::UCommandBase>::SharedPtr command_pub_;
  rclcpp::Publisher<base_station_interfaces::msg::ConsoleLog>::SharedPtr console_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keypress_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_vehicle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_thruster_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publishing_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_hard_turn_pub_;

  // Rate limiting variables
  rclcpp::TimerBase::SharedPtr publish_timer_;
  bool command_dirty_;  // Flag to track if we need to publish
  double publish_rate_hz_;  // Configurable publish rate
  bool thruster_enabled_;  // Flag to enable/disable thruster
  bool publishing_enabled_;  // Flag to enable/disable publishing and GUI printing
  // Invert controls (configurable)
  bool invert_vertical_controls_;
  bool invert_lateral_controls_;
  // Change-rate limiting
  double command_change_rate_hz_; // max number of changes per second
  std::chrono::steady_clock::time_point last_change_time_;
  std::chrono::milliseconds min_change_interval_ms_;
  // Hard turn mode: A/D (and Left/Right arrows) snap fin1 straight to +/- hard_turn_angle_
  // instead of incrementing, and spring back to 0 once the key stops repeating (there's no
  // real key-release event available from either the terminal or the GUI's key stream, so a
  // held key is inferred from repeats arriving faster than hard_turn_release_timeout_).
  bool hard_turn_mode_;
  bool turn_key_held_;
  double hard_turn_angle_;
  std::chrono::steady_clock::time_point last_turn_key_time_;
  std::chrono::milliseconds hard_turn_release_timeout_;
};

TeleopCommand::TeleopCommand() :
  Node("teleop_command"),
  fin1_(0.0), fin2_(0.0), fin3_(0.0), current_vehicle_index_(0), command_dirty_(false), thruster_enabled_(false), publishing_enabled_(false),
  hard_turn_mode_(false), turn_key_held_(false)
{
  declare_parameter("max_fin_value", 70.0);
  declare_parameter("thruster_value", 0);
  declare_parameter("vehicles_in_mission", std::vector<int64_t>{1, 2, 5});
  declare_parameter("publish_rate_hz", 5.0);  // Default 5 Hz publish rate
  // Max rate at which control values can change (to prevent overly-fast changes)
  declare_parameter("command_change_rate_hz", 10.0); // default 10 Hz
  // Invert control options
  declare_parameter("invert_vertical_controls", false);
  declare_parameter("invert_lateral_controls", false);
  declare_parameter("fin_change_speed", 2); // degrees per key press
  declare_parameter("hard_turn_angle", 45.0); // degrees, fin1 snap target in hard turn mode
  declare_parameter("hard_turn_release_timeout_ms", 400); // how long without a repeat before we consider the key released

  get_parameter("max_fin_value", max_fin_value_);
  get_parameter("thruster_value", thruster_value_);
  get_parameter("vehicles_in_mission", vehicles_in_mission_);
  get_parameter("publish_rate_hz", publish_rate_hz_);
  get_parameter("invert_vertical_controls", invert_vertical_controls_);
  get_parameter("invert_lateral_controls", invert_lateral_controls_);
  get_parameter("command_change_rate_hz", command_change_rate_hz_);
  get_parameter("fin_change_speed", fin_change_speed_); // just to use the param and avoid warning
  get_parameter("hard_turn_angle", hard_turn_angle_);
  int hard_turn_release_timeout_ms = 400;
  get_parameter("hard_turn_release_timeout_ms", hard_turn_release_timeout_ms);
  hard_turn_release_timeout_ = std::chrono::milliseconds(hard_turn_release_timeout_ms);

  if (command_change_rate_hz_ <= 0.0) command_change_rate_hz_ = 1.0;
  min_change_interval_ms_ = std::chrono::milliseconds(static_cast<int>(1000.0 / command_change_rate_hz_));
  // Allow immediate first change
  last_change_time_ = std::chrono::steady_clock::now() - min_change_interval_ms_;

  // Initialize with first vehicle in the list
  if (!vehicles_in_mission_.empty()) {
    vehicle_id_ = vehicles_in_mission_[current_vehicle_index_];
  } else {
    vehicle_id_ = 1; // Default fallback
    vehicles_in_mission_ = {1}; // Ensure we have at least one vehicle
  }

  command_pub_ = create_publisher<base_station_interfaces::msg::UCommandBase>("/keyboard_controls", 10);

  // Publisher for console log messages to GUI
  console_pub_ = create_publisher<base_station_interfaces::msg::ConsoleLog>("console_log", 10);

  // Status publishers for the GUI's Keyboard Controls tab. TRANSIENT_LOCAL so a GUI opened
  // after this node has already started still immediately gets the current state.
  auto status_qos = rclcpp::QoS(1).reliable().transient_local();
  status_vehicle_pub_ = create_publisher<std_msgs::msg::Int32>("/teleop_status/vehicle_id", status_qos);
  status_thruster_pub_ = create_publisher<std_msgs::msg::Bool>("/teleop_status/thruster_enabled", status_qos);
  status_publishing_pub_ = create_publisher<std_msgs::msg::Bool>("/teleop_status/publishing_enabled", status_qos);
  status_hard_turn_pub_ = create_publisher<std_msgs::msg::Bool>("/teleop_status/hard_turn_mode", status_qos);

  // Subscribe to GUI key press events
  keypress_sub_ = create_subscription<std_msgs::msg::String>(
    "/gui_keypress", 10,
    std::bind(&TeleopCommand::keyPressCallback, this, std::placeholders::_1)
  );

  // Initialize last command message
  last_command_msg_.vehicle_id = vehicle_id_;
  last_command_msg_.ucommand.header.stamp = get_clock()->now();
  last_command_msg_.ucommand.header.frame_id = vehicle_name_;
  last_command_msg_.vehicle_id = vehicle_id_;
  last_command_msg_.ucommand.fin = {
    -fin1_,  // Send degrees directly (negated for proper direction)
     fin2_,  // Send degrees directly
    -fin3_,  // Send degrees directly (negated for proper direction)
     0.0     // 4th fin (if needed)
  };
  last_command_msg_.ucommand.thruster = static_cast<double>(thruster_value_);
  last_command_msg_.thruster_enabled = thruster_enabled_;


  // Create timer for rate-limited publishing
  int timer_period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(timer_period_ms),
    std::bind(&TeleopCommand::timerCallback, this)
  );

  RCLCPP_INFO(get_logger(), "Initialized teleop for vehicle %d with publish rate %.1f Hz, thruster %s, invert_vertical=%s, invert_lateral=%s",
              vehicle_id_, publish_rate_hz_, thruster_enabled_ ? "ENABLED" : "DISABLED",
              invert_vertical_controls_ ? "true" : "false",
              invert_lateral_controls_ ? "true" : "false");

  // Publish initial status so a GUI that's already open picks up the starting state
  publishStatus();

  // Send initial console message to GUI
  publishConsoleLog("Teleop node initialized for vehicle " + std::to_string(vehicle_id_) + 
                   ", thruster " + (thruster_enabled_ ? "ENABLED" : "DISABLED"));
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto teleop_command = std::make_shared<TeleopCommand>();

  signal(SIGINT, quit);

  // Start keyboard loop in a separate thread
  std::thread keyboard_thread([teleop_command]() {
    teleop_command->keyLoop();
  });

  // Spin ROS to handle callbacks (including keypress subscriber)
  rclcpp::spin(teleop_command);
  
  // Clean up
  keyboard_thread.join();
  rclcpp::shutdown();
  return 0;
}

void TeleopCommand::keyLoop()
{
  char c;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys OR WASD to control fins");
  puts("W/Up: Fins up, S/Down: Fins down");
  puts("A/Left: Turn left, D/Right: Turn right");
  puts("Spacebar: +thruster (up to 100), R/Period/Comma: -thruster (down to -100, reverse)");
  puts("Q: Toggle thruster enable/disable");
  puts("Z: Toggle publishing and GUI printing on/off");
  puts("T: Toggle hard turn mode (A/D snap fin1 to +/- hard_turn_angle instead of incrementing)");
  puts("E: Switch vehicle");
  puts("Ctrl+C to quit");
  puts("NOTE: This node also accepts key presses from the GUI via ROS messages");
  
  RCLCPP_INFO(get_logger(), "Currently controlling vehicle %d", vehicle_id_);

  for(;;)
  {
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    // compute control steps (respecting inversion flags)
    double vertical_step = fin_change_speed_ * (invert_vertical_controls_ ? -1.0 : 1.0);
    double lateral_step = fin_change_speed_ * (invert_lateral_controls_ ? -1.0 : 1.0);

    // Handle escape sequences for arrow keys
    if (c == 0x1B) // ESC
    {
      char seq[2];
      if (read(kfd, &seq[0], 1) != 1) continue;
      if (read(kfd, &seq[1], 1) != 1) continue;
      
      if (seq[0] == '[')
      {
        switch(seq[1])
        {
          case 'A': // Up arrow
            tryApplyChange([this, vertical_step]() {
              fin2_ = std::clamp(fin2_ + vertical_step, -max_fin_value_, max_fin_value_);
              fin3_ = std::clamp(fin3_ + vertical_step, -max_fin_value_, max_fin_value_);
            });
            break;
          case 'B': // Down arrow
            tryApplyChange([this, vertical_step]() {
              fin2_ = std::clamp(fin2_ - vertical_step, -max_fin_value_, max_fin_value_);
              fin3_ = std::clamp(fin3_ - vertical_step, -max_fin_value_, max_fin_value_);
            });
            break;
          case 'C': // Right arrow
            if (hard_turn_mode_) {
              applyHardTurn(1.0);
            } else {
              tryApplyChange([this, lateral_step]() {
                fin1_ = std::clamp(fin1_ + lateral_step, -max_fin_value_, max_fin_value_);
              });
            }
            break;
          case 'D': // Left arrow
            if (hard_turn_mode_) {
              applyHardTurn(-1.0);
            } else {
              tryApplyChange([this, lateral_step]() {
                fin1_ = std::clamp(fin1_ - lateral_step, -max_fin_value_, max_fin_value_);
              });
            }
            break;
        }
      }
    }
    else
    {
      // Handle regular keys
      // Use the common key processing function
      processKey(c);
    }
  }
}

void TeleopCommand::switchVehicle()
{
  // Send zeros to the current vehicle before switching
  RCLCPP_INFO(get_logger(), "Sending zeros to vehicle %d before switching", vehicle_id_);
  
  // Create a zero command message for the current vehicle
  base_station_interfaces::msg::UCommandBase zero_command_msg;
  zero_command_msg.ucommand.header.stamp = get_clock()->now();
  zero_command_msg.ucommand.header.frame_id = vehicle_name_;
  zero_command_msg.vehicle_id = vehicle_id_;
  zero_command_msg.ucommand.fin = {0.0, 0.0, 0.0, 0.0};
  zero_command_msg.ucommand.thruster = 0.0;
  
  // Publish the zero command to the current vehicle
  command_pub_->publish(zero_command_msg);
  
  // Now switch to the next vehicle
  current_vehicle_index_ = (current_vehicle_index_ + 1) % vehicles_in_mission_.size();
  vehicle_id_ = vehicles_in_mission_[current_vehicle_index_];
  last_command_msg_.vehicle_id = vehicle_id_;

  // Reset local control values to zero for the new vehicle
  fin1_ = 0.0;
  fin2_ = 0.0;
  fin3_ = 0.0;
  thruster_value_ = 0; // Reset to default thruster value
  turn_key_held_ = false; // Don't carry a stale "held" hard-turn state to the new vehicle

  RCLCPP_INFO(get_logger(), "Switched to controlling vehicle %d (fins and thruster reset to defaults)", vehicle_id_);
  publishStatus();
}

void TeleopCommand::timerCallback()
{
  // No real key-release event is available, so a held A/D is inferred from repeats; once
  // they stop arriving for longer than the timeout, treat it as released and spring back to 0.
  if (hard_turn_mode_ && turn_key_held_) {
    auto elapsed = std::chrono::steady_clock::now() - last_turn_key_time_;
    if (elapsed > hard_turn_release_timeout_) {
      fin1_ = 0.0;
      turn_key_held_ = false;
      command_dirty_ = true;
      if (publishing_enabled_) {
        publishConsoleLog("Hard turn released for vehicle " + std::to_string(vehicle_id_));
      }
    }
  }

  // Publish at a steady rate regardless of changes (only if publishing is enabled)
  if (publishing_enabled_) {
    publishCommand();
  }
  command_dirty_ = false;  // Reset the flag (we published current state)
}

void TeleopCommand::publishCommand()
{
  last_command_msg_.ucommand.header.stamp = get_clock()->now();
  last_command_msg_.ucommand.header.frame_id = vehicle_name_;
  last_command_msg_.vehicle_id = vehicle_id_;
  // Publish four fin values (fourth unused/zero by default)
  last_command_msg_.ucommand.fin = {
    -fin1_,  // Send degrees directly (negated for proper direction)
     fin2_,  // Send degrees directly
    -fin3_,  // Send degrees directly (negated for proper direction)
     0.0     // 4th fin (if needed)
  };
  // Only send thruster value if thruster is enabled, otherwise send 0
  last_command_msg_.ucommand.thruster = thruster_enabled_ ? static_cast<double>(thruster_value_) : 0.0;
  last_command_msg_.thruster_enabled = thruster_enabled_;

  command_pub_->publish(last_command_msg_);
  
  int effective_thruster = thruster_enabled_ ? thruster_value_ : 0;
  RCLCPP_DEBUG(get_logger(), "Vehicle %d - Fins: [%.1f, %.1f, %.1f] deg, Thruster: %d %s", 
              vehicle_id_, fin1_, fin2_, fin3_, effective_thruster,
              thruster_enabled_ ? "(ENABLED)" : "(DISABLED)");
  
}

void TeleopCommand::keyPressCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Only process single character messages from ROS
  if (!msg->data.empty() && msg->data.length() == 1) {
    char key = msg->data[0];
    RCLCPP_DEBUG(get_logger(), "Received key press from GUI: '%c' (0x%02X)", key, static_cast<unsigned char>(key));
    processKey(key);
  } else if (!msg->data.empty()) {
    RCLCPP_WARN(get_logger(), "Received multi-character string from GUI: '%s' - ignoring", msg->data.c_str());
  }
}

void TeleopCommand::processKey(char key)
{
  // compute control steps (respecting inversion flags)
  double vertical_step = fin_change_speed_ * (invert_vertical_controls_ ? -1.0 : 1.0);
  double lateral_step = fin_change_speed_ * (invert_lateral_controls_ ? -1.0 : 1.0);

  switch(key) {
    case 'w':
    case 'W':
      tryApplyChange([this, vertical_step]() {
        fin2_ = std::clamp(fin2_ + vertical_step, -max_fin_value_, max_fin_value_);
        fin3_ = std::clamp(fin3_ + vertical_step, -max_fin_value_, max_fin_value_);
      });
      break;
    case 's':
    case 'S':
      tryApplyChange([this, vertical_step]() {
        fin2_ = std::clamp(fin2_ - vertical_step, -max_fin_value_, max_fin_value_);
        fin3_ = std::clamp(fin3_ - vertical_step, -max_fin_value_, max_fin_value_);
      });
      break;
    case 'a':
    case 'A':
      if (hard_turn_mode_) {
        applyHardTurn(-1.0);
      } else {
        tryApplyChange([this, lateral_step]() {
          fin1_ = std::clamp(fin1_ - lateral_step, -max_fin_value_, max_fin_value_);
        });
      }
      break;
    case 'd':
    case 'D':
      if (hard_turn_mode_) {
        applyHardTurn(1.0);
      } else {
        tryApplyChange([this, lateral_step]() {
          fin1_ = std::clamp(fin1_ + lateral_step, -max_fin_value_, max_fin_value_);
        });
      }
      break;
    case ' ': // Space key
      tryApplyChange([this]() {
        thruster_value_ = std::min(100, thruster_value_ + 5);
      });
      break;
    case 'r':
    case 'R':
      tryApplyChange([this]() {
        thruster_value_ = std::max(-100, thruster_value_ - 5);
      });
      break;
    case '.':
    case ',':
    case '-':
    case '_':
      tryApplyChange([this]() {
        thruster_value_ = std::max(-100, thruster_value_ - 5);
      });
      break;
    case 'e':
    case 'E':
      switchVehicle();
      publishConsoleLog("Switched to controlling vehicle " + std::to_string(vehicle_id_));
      break;
    case 'q':
    case 'Q':
      thruster_enabled_ = !thruster_enabled_;  // Toggle thruster enable
      RCLCPP_DEBUG(get_logger(), "Thruster %s", thruster_enabled_ ? "ENABLED" : "DISABLED");
      publishConsoleLog("Thruster " + std::string(thruster_enabled_ ? "ENABLED" : "DISABLED") +
                       " for vehicle " + std::to_string(vehicle_id_));
      publishStatus();
      command_dirty_ = true;  // Mark for publishing to update thruster state
      break;
    case 'z':
    case 'Z':
      publishing_enabled_ = !publishing_enabled_;  // Toggle publishing and GUI printing
      RCLCPP_INFO(get_logger(), "Keyboard control %s", publishing_enabled_ ? "ENABLED" : "DISABLED");
      publishConsoleLog("Keyboard control " + std::string(publishing_enabled_ ? "ENABLED" : "DISABLED") +
                       " for vehicle " + std::to_string(vehicle_id_));
      publishStatus();
      break;
    case 't':
    case 'T':
      hard_turn_mode_ = !hard_turn_mode_;
      // Reset fin1 so switching modes never leaves it stuck at an incremental or snapped value
      fin1_ = 0.0;
      turn_key_held_ = false;
      command_dirty_ = true;
      RCLCPP_INFO(get_logger(), "Hard turn mode %s", hard_turn_mode_ ? "ENABLED" : "DISABLED");
      publishConsoleLog("Hard turn mode " + std::string(hard_turn_mode_ ? "ENABLED" : "DISABLED") +
                       " for vehicle " + std::to_string(vehicle_id_));
      publishStatus();
      break;
    default:
      // Ignore unknown keys
      break;
  }
}

bool TeleopCommand::tryApplyChange(const std::function<void()> &apply_fn)
{
  auto now = std::chrono::steady_clock::now();
  if (now - last_change_time_ < min_change_interval_ms_) {
    RCLCPP_DEBUG(get_logger(), "Change rate-limited (%.1f Hz allowed)", command_change_rate_hz_);
    return false;
  }

  apply_fn();
  last_change_time_ = now;
  command_dirty_ = true;
  // Publish immediate console log about the change showing current ucommand values (only if publishing is enabled)
  if (publishing_enabled_) {
    int effective_thruster = thruster_enabled_ ? thruster_value_ : 0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "Vehicle " << vehicle_id_ << " - Fins: [" << fin1_ << ", " << fin2_ << ", " << fin3_ << "] deg, Thruster: "
        << effective_thruster << (thruster_enabled_ ? " (ENABLED)" : " (DISABLED)");
    publishConsoleLog(oss.str(), vehicle_id_);
  }
  return true;
}

void TeleopCommand::applyHardTurn(double direction)
{
  // Every repeat of A/D while the key is held refreshes this, regardless of
  // tryApplyChange's separate rate limit - setting fin1 to the same target again is harmless.
  last_turn_key_time_ = std::chrono::steady_clock::now();

  double target = direction * hard_turn_angle_;
  bool changed = (fin1_ != target);  // catches both the initial press and a direction reversal mid-hold
  turn_key_held_ = true;

  if (changed) {
    fin1_ = target;
    command_dirty_ = true;
    if (publishing_enabled_) {
      publishConsoleLog(
        "Hard turn " + std::string(direction < 0 ? "LEFT" : "RIGHT") +
        " (" + std::to_string(static_cast<int>(target)) + " deg) for vehicle " + std::to_string(vehicle_id_));
    }
  }
}

void TeleopCommand::publishConsoleLog(const std::string& message, int vehicle_id)
{
  auto console_msg = base_station_interfaces::msg::ConsoleLog();
  console_msg.message = message;
  console_msg.vehicle_number = vehicle_id;
  console_pub_->publish(console_msg);
}

void TeleopCommand::publishStatus()
{
  std_msgs::msg::Int32 vehicle_msg;
  vehicle_msg.data = vehicle_id_;
  status_vehicle_pub_->publish(vehicle_msg);

  std_msgs::msg::Bool thruster_msg;
  thruster_msg.data = thruster_enabled_;
  status_thruster_pub_->publish(thruster_msg);

  std_msgs::msg::Bool publishing_msg;
  publishing_msg.data = publishing_enabled_;
  status_publishing_pub_->publish(publishing_msg);

  std_msgs::msg::Bool hard_turn_msg;
  hard_turn_msg.data = hard_turn_mode_;
  status_hard_turn_pub_->publish(hard_turn_msg);
}
