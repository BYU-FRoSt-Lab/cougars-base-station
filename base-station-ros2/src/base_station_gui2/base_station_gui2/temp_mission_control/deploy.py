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

# New: path to store SSH setup state
SSH_SETUP_FILE = os.path.expanduser("~/.config/base_station/ssh_setup.json")

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

# New helper functions for SSH key setup and persistence

def _load_ssh_setup():
    try:
        with open(SSH_SETUP_FILE, 'r') as f:
            return json.load(f)
    except Exception:
        return {}


def _save_ssh_setup(d):
    os.makedirs(os.path.dirname(SSH_SETUP_FILE), exist_ok=True)
    with open(SSH_SETUP_FILE, 'w') as f:
        json.dump(d, f)


def _ensure_ssh_key():
    """Ensure an SSH keypair exists locally; generate one with no passphrase if needed."""
    key_path = os.path.expanduser('~/.ssh/id_rsa')
    pub_path = key_path + '.pub'
    if not os.path.exists(pub_path):
        try:
            subprocess.run(['ssh-keygen', '-t', 'rsa', '-b', '4096', '-N', '', '-f', key_path], check=True)
            ros_node.publish_console_log('✅ Generated new SSH keypair at ~/.ssh/id_rsa', 0)
        except Exception as e:
            ros_node.publish_console_log(f'❌ Failed to generate SSH key: {e}', 0)
            return None
    return pub_path


def _copy_key_to_remote(remote_user, remote_host, remote_port, vehicle_num):
    """Try to copy the local public key to the remote authorized_keys. Returns True on success."""
    pub = _ensure_ssh_key()
    if not pub:
        return False

    # Prefer ssh-copy-id if available
    try:
        sshcopy_check = subprocess.run(['which', 'ssh-copy-id'], capture_output=True)
        if sshcopy_check.returncode == 0:
            try:
                cmd = ['ssh-copy-id', '-p', str(remote_port), '-i', pub, f'{remote_user}@{remote_host}']
                ros_node.publish_console_log(f'DEBUG: Running: {" ".join(cmd)}', vehicle_num)
                res = subprocess.run(cmd, timeout=60, capture_output=True, text=True)
                if res.returncode == 0:
                    return True
                else:
                    ros_node.publish_console_log(f'❌ ssh-copy-id failed: {res.stderr}', vehicle_num)
            except Exception as e:
                ros_node.publish_console_log(f'❌ ssh-copy-id exception: {e}', vehicle_num)

        # Fallback to sshpass+ssh-copy-id if ssh-copy-id exists but requires password non-interactively
        sshpass_check = subprocess.run(['which', 'sshpass'], capture_output=True)
        if sshpass_check.returncode == 0:
            # Prompt for password once to copy the key
            try:
                pwd = getpass.getpass(f"Enter SSH password for {remote_user}@{remote_host}: ")
            except Exception:
                ros_node.publish_console_log('❌ Password entry cancelled by user', vehicle_num)
                return False
            if not pwd:
                ros_node.publish_console_log('❌ No password entered', vehicle_num)
                return False
            try:
                cmd = ['sshpass', '-p', pwd, 'ssh-copy-id', '-p', str(remote_port), '-i', pub, f'{remote_user}@{remote_host}']
                ros_node.publish_console_log(f'DEBUG: Running: sshpass -p [HIDDEN] ssh-copy-id -p {remote_port} -i {pub} {remote_user}@{remote_host}', vehicle_num)
                res = subprocess.run(cmd, timeout=60, capture_output=True, text=True)
                if res.returncode == 0:
                    return True
                else:
                    ros_node.publish_console_log(f'❌ sshpass+ssh-copy-id failed: {res.stderr}', vehicle_num)
            except Exception as e:
                ros_node.publish_console_log(f'❌ sshpass+ssh-copy-id exception: {e}', vehicle_num)

        # If neither method worked, instruct user to run ssh-copy-id manually
        ros_node.publish_console_log(f"⚠️ Unable to automatically copy SSH key to {remote_user}@{remote_host}.\nPlease run on your workstation: ssh-copy-id -p {remote_port} -i ~/.ssh/id_rsa.pub {remote_user}@{remote_host}", vehicle_num)
        return False

    except Exception as e:
        ros_node.publish_console_log(f'❌ Unexpected error during key copy: {e}', vehicle_num)
        return False


def _ensure_key_on_remote(remote_user, remote_host, remote_port, vehicle_num):
    """Ensure the remote has our public key installed. Returns True if key-based auth should work."""
    setup = _load_ssh_setup()
    host_key = f"{remote_user}@{remote_host}:{remote_port}"
    if setup.get(host_key):
        return True

    success = _copy_key_to_remote(remote_user, remote_host, remote_port, vehicle_num)
    if success:
        setup[host_key] = True
        _save_ssh_setup(setup)
        ros_node.publish_console_log(f'✅ SSH key installed on {remote_host}', vehicle_num)
        return True
    return False

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

    # Decide authentication method. Prefer key-based auth by ensuring key is installed once.
    use_password_auth = False

    if vehicle_num == 0:
        # For vehicle 0, try to ensure key-based authentication is set up (one-time).
        try:
            key_ok = _ensure_key_on_remote(remote_user, remote_host, remote_port, vehicle_num)
            if not key_ok:
                # If key installation failed, fall back to password prompt for this session
                use_password_auth = True
        except Exception as e:
            ros_node.publish_console_log(f'❌ Error ensuring SSH key on remote: {e}', vehicle_num)
            use_password_auth = True
    else:
        # For other vehicles assume key auth is configured
        use_password_auth = False

    if use_password_auth:
        # For systems where key setup failed, prompt for password and use sshpass for this session
        global cached_password
        try:
            # Check if sshpass is available
            sshpass_check = subprocess.run(["which", "sshpass"], capture_output=True)
            if sshpass_check.returncode != 0:
                ros_node.publish_console_log("❌ sshpass not found. Install with: sudo apt-get install sshpass", vehicle_num)
                return False

            if cached_password is None:
                ros_node.publish_console_log(f"🔐 Password prompt displayed in terminal for {remote_user}@{remote_host}", vehicle_num)
                try:
                    cached_password = getpass.getpass(f"Enter SSH password for {remote_user}@{remote_host}: ")
                    if not cached_password:
                        ros_node.publish_console_log("❌ No password entered", vehicle_num)
                        return False
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
        # Use key-based SSH/SCP (no password)
        try:
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

        except Exception as e:
            ros_node.publish_console_log(f"❌ Unexpected error during key-based SCP: {e}", vehicle_num)
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