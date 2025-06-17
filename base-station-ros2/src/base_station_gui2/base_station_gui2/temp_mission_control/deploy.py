import os
import json
import subprocess
from datetime import datetime
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node

global deployment_node

MISSION_DIR = os.path.expanduser("~/base_station/base-station-ros2/src/base_station_gui2/base_station_gui2/temp_mission_control/missions")
PARAM_DIR = os.path.expanduser("~/base_station/base-station-ros2/src/base_station_gui2/base_station_gui2/temp_mission_control/params")
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "deploy_config.json")

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

class DeploymentPublisher(Node):
    def __init__(self):
        super().__init__('deployment_publisher')
        self.publisher_ = self.create_publisher(ConsoleLog, 'console_log', 10)

    def publish_log(self, passed_msg):
        self.publisher_.publish(passed_msg)
        self.get_logger().info(f"Published: {passed_msg.message} to Coug#{passed_msg.coug_number}")

def publish_console_log(msg_text, msg_num):
    global deployment_node
    msg = ConsoleLog()
    msg.message = msg_text
    msg.coug_number = msg_num
    deployment_node.publish_log(msg)
    
def load_config(sel_vehicles):
    selected = []
    with open(CONFIG_FILE, "r") as f:
        template = json.load(f)["vehicles"][0]  # template vehicle
    for x in sel_vehicles:
        # Deep copy the template and replace 'X' with the current vehicle number
        vehicle = json.loads(json.dumps(template))  # Deep copy
        for key, value in vehicle.items():
            if isinstance(value, str):
                vehicle[key] = value.replace("X", str(x))
        selected.append(vehicle)
    return selected

def scp_file(file_path, remote_user, remote_host, remote_path, remote_filename, vehicle_num):
    """Deletes existing file then copies a new one via SCP."""
    delete_cmd = f"rm -f {os.path.join(remote_path, remote_filename)}"
    print(f"🗑️ Deleting {remote_filename} on {remote_host}...")
    publish_console_log(f"🗑️ Deleting {remote_filename} on {remote_host}...", vehicle_num)
    subprocess.run(["ssh", f"{remote_user}@{remote_host}", delete_cmd])

    destination = f"{remote_user}@{remote_host}:{os.path.join(remote_path, remote_filename)}"
    print(f"📤 Copying {file_path} to {destination}...")
    publish_console_log(f"📤 Copying {file_path} to {destination}...", vehicle_num)
    result = subprocess.run(["scp", file_path, destination])
    return result.returncode == 0

def log_deployment(vehicle, files_sent, vehicle_num):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
    with open(log_file, "w") as f:
        f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
        f.write(f"Host: {vehicle['remote_host']}\n")
        for label, path in files_sent:
            f.write(f"{label}: {path}\n")
    print(f"✅ Deployment logged: {log_file}\n")
    publish_console_log(f"✅ Deployment logged: {log_file}\n", vehicle_num)

def main(sel_vehicles): #selected vehicles
    global deployment_node
    deployment_node = DeploymentPublisher()

    vehicles = load_config(sel_vehicles)
    for vehicle in vehicles:
        load_success = True
        files_sent = []
        mission_path = os.path.join(MISSION_DIR, vehicle["mission_file"])
        param_path = os.path.join(PARAM_DIR, vehicle["param_file"])
        fleet_path = os.path.join(PARAM_DIR, vehicle["fleet_param_file"])
        vehicle_num = int(vehicle["remote_host"][-1])

        for label, file_path, remote_filename in [
            ("Mission File", mission_path, "mission_states.json"),
            ("Vehicle Params", param_path, os.path.basename(param_path)),
            ("Fleet Params", fleet_path, os.path.basename(fleet_path)),
        ]:
            if os.path.exists(file_path):
                success = scp_file(
                    file_path,
                    vehicle["remote_user"],
                    vehicle["remote_host"],
                    vehicle["remote_path"],
                    remote_filename,
                    vehicle_num
                )
                if success:
                    files_sent.append((label, file_path))
                else:
                    print(f"❌ Failed to deploy {label} to {vehicle['name']}")
                    publish_console_log(f"❌ Failed to deploy {label} to {vehicle['name']}", vehicle_num)
                    load_success = False
            else:
                print(f"⚠️ File not found: {file_path} (skipping)")
                publish_console_log(f"⚠️ File not found: {file_path} (skipping)", vehicle_num)
                load_success = False

        log_deployment(vehicle, files_sent, vehicle_num)
        message = f"Coug{vehicle_num} mission loading finished with no errors." if load_success else f"Coug{vehicle_num} mission loading failed."
        publish_console_log(message, vehicle_num)