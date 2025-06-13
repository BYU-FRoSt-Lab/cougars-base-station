import os
import json
import subprocess
from datetime import datetime

MISSION_DIR = "missions"
PARAM_DIR = "params"
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = "deploy_config.json"

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

def deploy_missions():
    def load_config():
        with open(CONFIG_FILE, "r") as f:
            return json.load(f)["vehicles"]

    def scp_file(file_path, remote_user, remote_host, remote_path, remote_filename):
        delete_cmd = f"rm -f {os.path.join(remote_path, remote_filename)}"
        subprocess.run(["ssh", f"{remote_user}@{remote_host}", delete_cmd])
        destination = f"{remote_user}@{remote_host}:{os.path.join(remote_path, remote_filename)}"
        result = subprocess.run(["scp", file_path, destination])
        return result.returncode == 0

    def log_deployment(vehicle, files_sent):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
        with open(log_file, "w") as f:
            f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
            f.write(f"Host: {vehicle['remote_host']}\n")
            for label, path in files_sent:
                f.write(f"{label}: {path}\n")

    vehicles = load_config()
    for vehicle in vehicles:
        files_sent = []
        mission_path = os.path.join(MISSION_DIR, vehicle["mission_file"])
        param_path = os.path.join(PARAM_DIR, vehicle["param_file"])
        fleet_path = os.path.join(PARAM_DIR, vehicle["fleet_param_file"])

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
                    remote_filename
                )
                if success:
                    files_sent.append((label, file_path))
            # else: skip

        log_deployment(vehicle, files_sent)