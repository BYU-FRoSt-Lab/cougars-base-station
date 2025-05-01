import os
import json
import subprocess
from datetime import datetime

MISSION_DIR = "missions"
PARAM_DIR = "params"
DEPLOY_HISTORY_DIR = "/home/frostlab/bag/deployment_history"
CONFIG_FILE = "deploy_config.json"

os.makedirs(DEPLOY_HISTORY_DIR, exist_ok=True)

def load_config():
    with open(CONFIG_FILE, "r") as f:
        return json.load(f)["vehicles"]

def deploy_file(file_path, remote_user, remote_host, remote_path, label):
    remote = f"{remote_user}@{remote_host}:{remote_path}"
    cmd = ["rsync", "-avz", file_path, remote]
    print(f"📤 Deploying {label} ({os.path.basename(file_path)}) to {remote_host}...")
    result = subprocess.run(cmd)
    return result.returncode == 0

def log_deployment(vehicle, files_sent):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(DEPLOY_HISTORY_DIR, f"{vehicle['name']}_deployment_{timestamp}.txt")
    with open(log_file, "w") as f:
        f.write(f"Deployment for {vehicle['name']} at {timestamp}\n")
        f.write(f"Host: {vehicle['remote_host']}\n")
        for label, path in files_sent:
            f.write(f"{label}: {path}\n")
    print(f"✅ Deployment logged: {log_file}\n")

def main():
    vehicles = load_config()
    for vehicle in vehicles:
        files_sent = []
        mission_path = os.path.join(MISSION_DIR, vehicle["mission_file"])
        param_path = os.path.join(PARAM_DIR, vehicle["param_file"])
        fleet_path = os.path.join(PARAM_DIR, vehicle["fleet_param_file"])

        for label, file_path in [("Mission File", mission_path),
                                 ("Vehicle Params", param_path),
                                 ("Fleet Params", fleet_path)]:
            if os.path.exists(file_path):
                success = deploy_file(
                    file_path,
                    vehicle["remote_user"],
                    vehicle["remote_host"],
                    vehicle["remote_path"],
                    label
                )
                if success:
                    files_sent.append((label, file_path))
                else:
                    print(f"❌ Failed to deploy {label} to {vehicle['name']}")
            else:
                print(f"⚠️ File not found: {file_path} (skipping)")

        log_deployment(vehicle, files_sent)

if __name__ == "__main__":
    main()
