import os
import json
import subprocess
from datetime import datetime
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
import getpass

global ros_node
cached_password = None  # Cache password for Vehicle 0 during deployment

PARAM_DIR = os.path.expanduser("~/base_station/base-station-ros2/src/base_station_gui2/base_station_gui2/temp_mission_control/params")
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "deploy_config.json")

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

def scp_file(file_path, remote_user, remote_host, remote_port, remote_path, remote_filename, vehicle_num):
    """Deletes existing file then copies a new one via SCP."""
    delete_cmd = f"rm -f {os.path.join(remote_path, remote_filename)}"
    print(f"🗑️ Deleting {remote_filename} on {remote_host}...")
    ros_node.publish_console_log(f"🗑️ Deleting {remote_filename} on {remote_host}...", vehicle_num)
    
    # Check if this is Vehicle 0 (simulator) - use password authentication
    use_password_auth = (vehicle_num == 0)
    
    if use_password_auth:
        # For Vehicle 0, prompt for password and use sshpass
        global cached_password
        try:
            # Check if sshpass is available
            sshpass_check = subprocess.run(["which", "sshpass"], capture_output=True)
            if sshpass_check.returncode != 0:
                ros_node.publish_console_log("❌ sshpass not found. Install with: sudo apt-get install sshpass", vehicle_num)
                return False
            
            # Use cached password or prompt for new one
            if cached_password is None:
                print(f"\n🔐 PASSWORD REQUIRED for Vehicle 0 🔐")
                print(f"Please enter SSH password for {remote_user}@{remote_host} in the terminal where you started the GUI")
                print("=" * 60)
                ros_node.publish_console_log(f"🔐 Password prompt displayed in terminal for {remote_user}@{remote_host}", vehicle_num)
                
                # Prompt for password (this will appear in the terminal where the GUI was launched)
                try:
                    cached_password = getpass.getpass(f"Enter SSH password for {remote_user}@{remote_host}: ")
                    if not cached_password:
                        ros_node.publish_console_log("❌ No password entered", vehicle_num)
                        return False
                    print("✅ Password received, continuing deployment...")
                    ros_node.publish_console_log("✅ Password received, continuing deployment...", vehicle_num)
                except (KeyboardInterrupt, EOFError):
                    ros_node.publish_console_log("❌ Password entry cancelled by user", vehicle_num)
                    return False
            
            password = cached_password
            
            # SSH command with password
            ssh_cmd = ["sshpass", "-p", password, "ssh", "-p", str(remote_port), f"{remote_user}@{remote_host}", delete_cmd]
            ros_node.publish_console_log(f"DEBUG: Executing SSH command with password auth: sshpass -p [HIDDEN] ssh -p {remote_port} {remote_user}@{remote_host} {delete_cmd}", vehicle_num)
            
            try:
                ssh_result = subprocess.run(ssh_cmd, timeout=30, capture_output=True, text=True)
                if ssh_result.returncode != 0:
                    ros_node.publish_console_log(f"DEBUG: SSH delete command failed with return code {ssh_result.returncode}", vehicle_num)
                    ros_node.publish_console_log(f"DEBUG: SSH stderr: {ssh_result.stderr}", vehicle_num)
                    ros_node.publish_console_log(f"DEBUG: SSH stdout: {ssh_result.stdout}", vehicle_num)
                else:
                    ros_node.publish_console_log(f"DEBUG: SSH delete command succeeded", vehicle_num)
            except subprocess.TimeoutExpired:
                ros_node.publish_console_log(f"❌ SSH command timed out after 30 seconds", vehicle_num)
                return False
            except Exception as e:
                ros_node.publish_console_log(f"❌ SSH command failed with exception: {e}", vehicle_num)
                return False

            destination = f"{remote_user}@{remote_host}:{os.path.join(remote_path, remote_filename)}"
            print(f"📤 Copying {file_path} to {destination}...")
            ros_node.publish_console_log(f"📤 Copying {file_path} to {destination}...", vehicle_num)
            
            # SCP command with password
            scp_cmd = ["sshpass", "-p", password, "scp", "-P", str(remote_port), file_path, destination]
            ros_node.publish_console_log(f"DEBUG: Executing SCP command with password auth: sshpass -p [HIDDEN] scp -P {remote_port} {file_path} {destination}", vehicle_num)
            
            try:
                scp_result = subprocess.run(scp_cmd, timeout=60, capture_output=True, text=True)
                if scp_result.returncode != 0:
                    ros_node.publish_console_log(f"DEBUG: SCP command failed with return code {scp_result.returncode}", vehicle_num)
                    ros_node.publish_console_log(f"DEBUG: SCP stderr: {scp_result.stderr}", vehicle_num)
                    ros_node.publish_console_log(f"DEBUG: SCP stdout: {scp_result.stdout}", vehicle_num)
                    return False
                else:
                    ros_node.publish_console_log(f"DEBUG: SCP command succeeded", vehicle_num)
                    return True
            except subprocess.TimeoutExpired:
                ros_node.publish_console_log(f"❌ SCP command timed out after 60 seconds", vehicle_num)
                return False
            except Exception as e:
                ros_node.publish_console_log(f"❌ SCP command failed with exception: {e}", vehicle_num)
                return False
                
        except KeyboardInterrupt:
            ros_node.publish_console_log("❌ Password entry cancelled by user", vehicle_num)
            return False
        except Exception as e:
            ros_node.publish_console_log(f"❌ Password authentication failed: {e}", vehicle_num)
            return False
    
    else:
        # For other vehicles, use key-based authentication (existing logic)
        ssh_cmd = ["ssh", "-p", str(remote_port), f"{remote_user}@{remote_host}", delete_cmd]
        ros_node.publish_console_log(f"DEBUG: Executing SSH command: {' '.join(ssh_cmd)}", vehicle_num)
        
        try:
            ssh_result = subprocess.run(ssh_cmd, timeout=30, capture_output=True, text=True)
            if ssh_result.returncode != 0:
                ros_node.publish_console_log(f"DEBUG: SSH delete command failed with return code {ssh_result.returncode}", vehicle_num)
                ros_node.publish_console_log(f"DEBUG: SSH stderr: {ssh_result.stderr}", vehicle_num)
                ros_node.publish_console_log(f"DEBUG: SSH stdout: {ssh_result.stdout}", vehicle_num)
            else:
                ros_node.publish_console_log(f"DEBUG: SSH delete command succeeded", vehicle_num)
        except subprocess.TimeoutExpired:
            ros_node.publish_console_log(f"❌ SSH command timed out after 30 seconds", vehicle_num)
            return False
        except Exception as e:
            ros_node.publish_console_log(f"❌ SSH command failed with exception: {e}", vehicle_num)
            return False

        destination = f"{remote_user}@{remote_host}:{os.path.join(remote_path, remote_filename)}"
        print(f"📤 Copying {file_path} to {destination}...")
        ros_node.publish_console_log(f"📤 Copying {file_path} to {destination}...", vehicle_num)
        
        scp_cmd = ["scp", "-P", str(remote_port), file_path, destination]
        ros_node.publish_console_log(f"DEBUG: Executing SCP command: {' '.join(scp_cmd)}", vehicle_num)
        
        try:
            scp_result = subprocess.run(scp_cmd, timeout=60, capture_output=True, text=True)
            if scp_result.returncode != 0:
                ros_node.publish_console_log(f"DEBUG: SCP command failed with return code {scp_result.returncode}", vehicle_num)
                ros_node.publish_console_log(f"DEBUG: SCP stderr: {scp_result.stderr}", vehicle_num)
                ros_node.publish_console_log(f"DEBUG: SCP stdout: {scp_result.stdout}", vehicle_num)
                return False
            else:
                ros_node.publish_console_log(f"DEBUG: SCP command succeeded", vehicle_num)
                return True
        except subprocess.TimeoutExpired:
            ros_node.publish_console_log(f"❌ SCP command timed out after 60 seconds", vehicle_num)
            return False
        except Exception as e:
            ros_node.publish_console_log(f"❌ SCP command failed with exception: {e}", vehicle_num)
            return False

def log_deployment(vehicle, files_sent, vehicle_num):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
    with open(log_file, "w") as f:
        f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
        f.write(f"Host: {vehicle['remote_host']}\n")
        for label, path in files_sent:
            f.write(f"{label}: {path}\n")
    print(f"✅ Deployment logged: {log_file}\n")
    ros_node.publish_console_log(f"✅ Deployment logged: {log_file}\n", vehicle_num)

def main(passed_ros_node, sel_vehicles, passed_file_paths=[]): #selected vehicles
    global ros_node, cached_password
    ros_node = passed_ros_node
    
    # Reset cached password for new deployment
    cached_password = None

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
                success = scp_file(
                    file_path,
                    vehicle["remote_user"],
                    vehicle["remote_host"],
                    vehicle.get("remote_port", 22),  # Use port 22 as default
                    vehicle["remote_path"],
                    remote_filename,
                    vehicle_num
                )
                if success:
                    files_sent.append((label, file_path))
                else:
                    print(f"❌ Failed to deploy {label} to {vehicle['name']}")
                    ros_node.publish_console_log(f"❌ Failed to deploy {label} to {vehicle['name']}", vehicle_num)
                    load_success = False
            else:
                print(f"⚠️ File not found: {file_path} (skipping)")
                ros_node.publish_console_log(f"⚠️ File not found: {file_path} (skipping)", vehicle_num)
                load_success = False

        log_deployment(vehicle, files_sent, vehicle_num)
        message = f"Coug{vehicle_num} mission loading finished with no errors." if load_success else f"Coug{vehicle_num} mission loading failed."
        ros_node.publish_console_log(message, vehicle_num)