#!/usr/bin/env python3

import subprocess
import os
import re
import sys
import yaml
from base_station_interfaces.msg import ConsoleLog
import rclpy
from rclpy.node import Node

global deployment_node

class DeploymentPublisher(Node):
    def __init__(self):
        super().__init__('deployment_publisher')
        self.publisher_ = self.create_publisher(ConsoleLog, 'console_log', 10)

    def publish_log(self, passed_msg):
        self.publisher_.publish(passed_msg)
        # self.get_logger().info(f"Published: {passed_msg.message} to Coug#{passed_msg.vehicle_number}")

def publish_console_log(msg_text, msg_num):
    global deployment_node
    msg = ConsoleLog()
    msg.message = msg_text
    msg.vehicle_number = msg_num
    deployment_node.publish_log(msg)

#TODO: where is this config path???
CONFIG_PATH = os.path.expanduser("~/config/cougarsrc.sh")
VEHICLE_PARAMS_PATH = os.path.expanduser("~/config/vehicle_params.yaml")
PYTHON_SCRIPT = os.path.expanduser("~/ros2_ws/update_yaml.py")
DVL_DIR = os.path.expanduser("~/ros2_ws/dvl_tools")
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
            publish_console_log(f"[INFO] Service call to {service_name} succeeded. {match.group(1) if match else ''}", 0)
        elif "success=False" in output:
            match = re.search(r'message=\s*(.*)', output)
            publish_console_log(f"[WARNING] Service call to {service_name} failed with message: {match.group(1) if match else ''}", 0)
        else:
            publish_console_log(f"[ERROR] Unexpected response from service {service_name}: {output}", 0)
    except subprocess.TimeoutExpired:
        publish_console_log(f"[ERROR] Service call to {service_name} timed out.", 0)

# def prompt_yes_no(prompt):
#     return input(f"{prompt} (y/n): ").strip().lower() == 'y'

def run_script(script_path, *args):
    subprocess.run(['bash', script_path, *args])

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
            publish_console_log(f"[ERROR] Parameter '{ROS_PARAM_NAME}' is not set on node '{NODE_NAME}'.", 0)
        else:
            publish_console_log(f"[ERROR] Failed to retrieve parameter: {output}", 0)
    except Exception as e:
        publish_console_log(f"[ERROR] Exception while getting ROS param: {e}", 0)
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

    publish_console_log(f"[INFO] Updated {file_path} with {param_name} = {value} under {namespace}/{node_name}", 0)

# ----------------------------
# MAIN SCRIPT LOGIC
# ----------------------------
def main():
    global deployment_node
    deployment_node = DeploymentPublisher()

    source_env()

    run_service_call(f"{NAMESPACE}/calibrate_depth", "std_srvs/srv/Trigger", "{}")

    os.chdir(DVL_DIR)

    run_script(os.path.join(DVL_DIR, "calibrate_gyro.sh"))

    speed = 1500
    run_script(os.path.join(DVL_DIR, "set_speed_sound.sh"), str(speed))

    print('a0')
    run_script(os.path.join(DVL_DIR, "set_ntp.sh"))
    print('a1')

    param_value = get_ros_param()
    if param_value:
        update_yaml_param(VEHICLE_PARAMS_PATH, ROS_PARAM_NAME, param_value, NODE_NAME, NAMESPACE)

if __name__ == "__main__":
    main()
