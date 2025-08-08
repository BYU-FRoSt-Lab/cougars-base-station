import os
import json
from datetime import datetime
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
from pathlib import Path
import paramiko

global ros_node

# SSH configuration
SSH_KEY_PATH = str(Path.home()) + "/.ssh/id_ed25519_cougs"

PARAM_DIR = os.environ.get(
    "BASE_STATION_PARAM_DIR",
    os.path.expanduser("~/base_station/mission_control/params")
)
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = str(Path.home()) + "/base_station/mission_control/deploy_config.json"

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

def load_config(sel_vehicles):
    with open(CONFIG_FILE, "r") as f:
        config = json.load(f)
        vehicles = config["vehicles"]
    result = []
    for num in sel_vehicles:
        if str(num) in vehicles:
            result.append(vehicles[str(num)])
        else:
            ros_node.publish_console_log(f"❌ Vehicle {num} not found in config, consider adding (skipping)", num)
    return result

def sftp_file(file_path, remote_user, remote_host, remote_path, remote_filename, vehicle_num):
    """Deletes existing file then copies a new one via SFTP using SSH key authentication."""
    try:
        ros_node.publish_console_log(f"🗑️ Deleting {remote_filename} on {remote_host}...", vehicle_num)
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        # Load SSH private key
        try:
            private_key = paramiko.RSAKey.from_private_key_file(SSH_KEY_PATH)
        except paramiko.SSHException:
            # Try Ed25519 key if RSA fails
            try:
                private_key = paramiko.Ed25519Key.from_private_key_file(SSH_KEY_PATH)
            except paramiko.SSHException:
                ros_node.publish_console_log(f"❌ Failed to load SSH key from {SSH_KEY_PATH}", vehicle_num)
                return False
        
        ssh.connect(remote_host, username=remote_user, pkey=private_key, timeout=10)
        sftp = ssh.open_sftp()
        remote_full_path = os.path.join(remote_path, remote_filename)
        # Try to delete the remote file if it exists
        try:
            sftp.remove(remote_full_path)
        except FileNotFoundError:
            pass  # It's OK if the file doesn't exist
        ros_node.publish_console_log(f"📤 Copying {file_path} to {remote_user}@{remote_host}:{remote_full_path}...", vehicle_num)
        sftp.put(file_path, remote_full_path)
        sftp.close()
        ssh.close()
        return True
    except Exception as e:
        ros_node.publish_console_log(f"❌ SFTP error: {e}", vehicle_num)
        return False

def log_deployment(vehicle, files_sent, vehicle_num):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
    with open(log_file, "w") as f:
        f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
        f.write(f"Host: {vehicle['remote_host']}\n")
        for label, path in files_sent:
            f.write(f"{label}: {path}\n")
    ros_node.publish_console_log(f"✅ Deployment logged: {log_file}\n", vehicle_num)

def main(passed_ros_node, sel_vehicles, passed_file_paths=[]): #selected vehicles
    global ros_node
    ros_node = passed_ros_node

    vehicles = load_config(sel_vehicles)
    for i, vehicle in enumerate(vehicles):
        load_success = True
        files_sent = []
        mission_path = passed_file_paths[i]
        param_path = os.path.join(PARAM_DIR, vehicle["param_file"])
        fleet_path = os.path.join(PARAM_DIR, vehicle["fleet_param_file"])
        vehicle_num = int(vehicle["name"][-1])
        ros_node.publish_console_log(f"Mission path for coug{vehicle_num} was set as: {mission_path}", vehicle_num)

        for label, file_path, remote_filename in [
            ("Mission File", mission_path, "mission.yaml"),
            ("Vehicle Params", param_path, os.path.basename(param_path)),
            ("Fleet Params", fleet_path, os.path.basename(fleet_path)),
        ]:
            if os.path.exists(file_path):
                success = sftp_file(
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
                    ros_node.publish_console_log(f"❌ Failed to deploy {label} to {vehicle['name']}", vehicle_num)
                    load_success = False
            else:
                ros_node.publish_console_log(f"⚠️ File not found: {file_path} (skipping)", vehicle_num)
                load_success = False

        log_deployment(vehicle, files_sent, vehicle_num)
        message = f"Coug{vehicle_num} mission loading finished with no errors." if load_success else f"Coug{vehicle_num} mission loading failed."
        ros_node.publish_console_log(message, vehicle_num)