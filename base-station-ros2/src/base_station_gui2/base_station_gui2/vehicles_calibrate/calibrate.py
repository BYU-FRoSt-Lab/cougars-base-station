#!/usr/bin/env python3

import subprocess
import os
import re
import sys
import yaml
import threading
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node
global ros_node

#TODO: where is this config path???
CONFIG_PATH = os.path.expanduser("~/config/cougarsrc.sh")
#Confirmed: This exists
VEHICLE_PARAMS_PATH = os.path.expanduser("~/config/vehicle_params.yaml")
#Confirmed: This exists
PYTHON_SCRIPT = os.path.expanduser("~/ros2_ws/update_yaml.py")
#Confirmed: This exists
DVL_DIR = os.path.expanduser("~/ros2_ws/dvl_tools")
#TODO: change this to coug2, coug3 etc?
NAMESPACE = os.getenv("NAMESPACE", "")  # You can also hardcode this if needed
NODE_NAME = "depth_convertor"
ROS_PARAM_NAME = "fluid_pressure_atm"

def source_env():
    command = f"source {CONFIG_PATH} && env"
    proc = subprocess.Popen(['/bin/bash', '-c', command], stdout=subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.decode().partition("=")
        os.environ[key.strip()] = value.strip()
    proc.communicate()

def run_service_call(service_name, srv_type, args):
    try:
        result = subprocess.run(
            ['ros2', 'service', 'call', service_name, srv_type, args],
            capture_output=True,
            text=True,
            timeout=5
        )
        output = result.stdout + result.stderr
        if "success=True" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.publish_console_log(f"[INFO] Service call to {service_name} succeeded. {match.group(1) if match else ''}", 0)
        elif "success=False" in output:
            match = re.search(r'message=\s*(.*)', output)
            ros_node.publish_console_log(f"[WARNING] Service call to {service_name} failed with message: {match.group(1) if match else ''}", 0)
        else:
            ros_node.publish_console_log(f"[ERROR] Unexpected response from service {service_name}: {output}", 0)
    except subprocess.TimeoutExpired:
        ros_node.publish_console_log(f"[ERROR] Service call to {service_name} timed out.", 0)

def run_script(script_path, *args):
    # Use Popen for real-time output with timeout
    process = subprocess.Popen(
        ['bash', script_path, *args], 
        stdout=subprocess.PIPE, 
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        universal_newlines=True
    )
    
    # ANSI color code pattern
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    
    def read_output():
        try:
            # Read output line by line as it comes
            for line in process.stdout:
                line = line.rstrip()
                if line:
                    # Skip curl progress lines
                    if any(skip_text in line for skip_text in [
                        '% Total', '% Received', '% Xferd', 'Dload', 'Upload', 
                        '--:--:--', 'Current', 'Speed', 'Time'
                    ]):
                        continue  # Skip curl progress output
                    
                    print(line)
                    clean_line = ansi_escape.sub('', line)
                    ros_node.publish_console_log(clean_line, 0)
        except:
            pass  # Process was terminated
    
    # Start reading in a separate thread
    output_thread = threading.Thread(target=read_output, daemon=True)
    output_thread.start()
    
    try:
        # Wait for process to complete with timeout
        process.wait(timeout=10)  # 10 seconds timeout to try running the script
        
    except subprocess.TimeoutExpired:
        print("Script timed out, killing process...")
        process.kill()
        process.wait()
        error_msg = f"Script {script_path} timed out after 10 seconds"
        print(error_msg)
        ros_node.publish_console_log(error_msg, 0)
        return
    
    # Wait for output thread to finish (with a short timeout)
    output_thread.join(timeout=1)
    
    # Check return code
    if process.returncode != 0:
        error_msg = f"Script {script_path} failed with return code {process.returncode}"
        print(error_msg)
        ros_node.publish_console_log(error_msg, 0)
             
def get_ros_param():
    full_node = f"/{NAMESPACE}/{NODE_NAME}".replace("//", "/")
    try:
        result = subprocess.run(
            ["ros2", "param", "get", full_node, ROS_PARAM_NAME],
            capture_output=True,
            text=True
        )
        output = result.stdout.strip()
        match = re.search(r"[-+]?\d*\.\d+|\d+", output)
        if match:
            return match.group()
        elif "Parameter not set" in output:
            ros_node.publish_console_log(f"[ERROR] Parameter '{ROS_PARAM_NAME}' is not set on node '{NODE_NAME}'.", 0)
        else:
            ros_node.publish_console_log(f"[ERROR] Failed to retrieve parameter: {output}", 0)
    except Exception as e:
        ros_node.publish_console_log(f"[ERROR] Exception while getting ROS param: {e}", 0)
    return None

def update_yaml_param(file_path, param_name, value, node_name, namespace):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)

    if namespace not in data:
        data[namespace] = {}
    if node_name not in data[namespace]:
        data[namespace][node_name] = {}
    
    data[namespace][node_name][param_name] = float(value)

    with open(file_path, 'w') as f:
        yaml.dump(data, f)

    ros_node.publish_console_log(f"[INFO] Updated {file_path} with {param_name} = {value} under {namespace}/{node_name}", 0)

# ----------------------------
# MAIN SCRIPT LOGIC
# ----------------------------
def main(passed_ros_node, vehicle_numbers):
    global ros_node 
    ros_node = passed_ros_node

    ros_node.publish_console_log(f"Running source_env...", 0)
    source_env()

    ros_node.publish_console_log(f"Running run_service_call...", 0)
    run_service_call(f"{NAMESPACE}/calibrate_depth", "std_srvs/srv/Trigger", "{}")

    ros_node.publish_console_log(f"Changing to DVL directory...", 0)
    os.chdir(DVL_DIR)

    ros_node.publish_console_log(f"Running calibrate_gyro.sh...", 0)
    run_script(os.path.join(DVL_DIR, "calibrate_gyro.sh"))

    speed = 1500
    ros_node.publish_console_log(f"Running set_speed_sound...", 0)
    run_script(os.path.join(DVL_DIR, "set_speed_sound.sh"), str(speed))

    ros_node.publish_console_log(f"Running set_ntp...", 0)
    run_script(os.path.join(DVL_DIR, "set_ntp.sh"))

    ros_node.publish_console_log(f"Running get_ros_param...", 0)
    param_value = get_ros_param()
    if param_value:
        update_yaml_param(VEHICLE_PARAMS_PATH, ROS_PARAM_NAME, param_value, NODE_NAME, NAMESPACE)
    ros_node.publish_console_log(f"Calibrate.py finished", 0)

if __name__ == "__main__":
    main()
