#!/usr/bin/env python3
"""Interactive WiFi + XBee radio range test, run from the base station.

Walk the base station away from a stationary submarine to a series of target
distances. At each stop this runs the same iperf3 + RSSI capture as the
hardware_tests wifi tool, then pauses for a manual XCTU radio capture.
Distance is tracked live from the base station's own sensor_msgs/NavSatFix
topic (default /fix, e.g. gpsd_client) against a manually-entered starting
GPS position for the submarine, since the sub isn't reachable by ROS here.
"""

import csv
import math
import re
import select
import shutil
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


DEFAULT_SERVER_IP = "192.168.0.102"
DEFAULT_DURATION_S = "30"
DEFAULT_DISTANCES = "5,10,20,30,40,50"
DEFAULT_TOLERANCE_M = "1.0"
DEFAULT_NAVSAT_TOPIC = "/fix"
DEFAULT_MAX_WIFI_DISTANCE_M = "30"
DEFAULT_XCTU_EXPORT_DIR = "~/scripts/hardware_tests/xctu_exports"

# iperf3's default TCP connect retry behavior can hang for ~2 minutes when
# the server is unreachable (e.g. out of WiFi range). Bound it explicitly.
CONNECT_TIMEOUT_MS = 5000

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def ask(prompt, default):
    value = input(f"{prompt} [{default}]: ").strip()
    return value if value else default


def ask_float(prompt, default):
    while True:
        raw = ask(prompt, default)
        try:
            return float(raw)
        except ValueError:
            print(f"'{raw}' is not a number, try again.")


def ask_int(prompt, default):
    while True:
        raw = ask(prompt, default)
        try:
            return int(raw)
        except ValueError:
            print(f"'{raw}' is not a whole number, try again.")


def find_wifi_interface():
    try:
        out = subprocess.check_output(["iw", "dev"], text=True)
        matches = re.findall(r"Interface\s+(\S+)", out)
        if matches:
            return matches[0]
    except Exception:
        pass

    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device"], text=True
        )
        for line in out.splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[1] == "wifi" and parts[2] == "connected":
                return parts[0]
    except Exception:
        pass

    return None


class BaseStationFix(Node):
    """Tracks the base station's latest valid GPS fix."""

    def __init__(self, topic):
        super().__init__("comm_range_test")
        self.lock = threading.Lock()
        self._fix = None
        # gpsd_client publishes /fix as BEST_EFFORT; a RELIABLE subscriber
        # (the plain-int-depth default) silently never connects to it even
        # though the topic is alive (`ros2 topic echo` works because it
        # auto-matches the publisher's actual QoS).
        self.create_subscription(NavSatFix, topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to {topic}")

    def _cb(self, msg):
        # status.status < 0 means NavSatStatus.STATUS_NO_FIX — gpsd isn't
        # confident in the fix, but it still publishes its best estimate
        # (position_covariance reflects the uncertainty). Use it anyway
        # rather than blocking the test entirely; just flag it as low
        # confidence so the operator can judge whether to trust it.
        with self.lock:
            self._fix = {
                "lat": msg.latitude,
                "lon": msg.longitude,
                "has_fix": msg.status.status >= 0,
            }

    def latest_fix(self):
        with self.lock:
            return dict(self._fix) if self._fix else None


def wait_until_at_distance(fix_node, sub_lat, sub_lon, target_m, tolerance_m, poll_hz=2.0):
    print(f"\nMove to {target_m:g} m from the submarine's start position.")
    print("(Press Enter at any time to lock in your current position early.)")

    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            fix = fix_node.latest_fix()
            if fix is None:
                print("\nManual override: proceeding with unknown distance (no GPS fix yet).")
                return None
            dist = haversine_m(sub_lat, sub_lon, fix["lat"], fix["lon"])
            print(f"\nManual override: proceeding at {dist:.1f} m.")
            return dist

        fix = fix_node.latest_fix()
        if fix is None:
            print("\rWaiting for GPS fix...                              ", end="", flush=True)
        else:
            dist = haversine_m(sub_lat, sub_lon, fix["lat"], fix["lon"])
            quality = "" if fix["has_fix"] else "  [NO_FIX, low confidence]"
            print(
                f"\rCurrent distance: {dist:6.1f} m   target: {target_m:g} m   "
                f"delta: {dist - target_m:+.1f} m{quality}   ",
                end="", flush=True
            )
            if abs(dist - target_m) <= tolerance_m:
                print(f"\nReached target distance ({dist:.1f} m).")
                return dist

        time.sleep(1.0 / poll_hz)


def run_wifi_stop(server_ip, interface, duration_s, output_csv):
    pattern = re.compile(
        r"\]\s+(\d+\.\d+)-(\d+\.\d+)\s+sec\s+[\d.]+\s+\w+Bytes\s+([\d.]+)\s+Mbits/sec"
    )

    iperf = subprocess.Popen(
        [
            "iperf3", "-c", server_ip, "-i", "1", "-t", str(duration_s),
            "--connect-timeout", str(CONNECT_TIMEOUT_MS), "--forceflush",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )

    # Belt-and-suspenders on top of --connect-timeout: if iperf3 still
    # hasn't finished well past its expected runtime (e.g. it hangs for
    # some other reason when out of range), kill it rather than block the
    # whole test session forever. select() lets us poll for that even
    # while waiting on iperf3's stdout.
    watchdog_s = duration_s + (CONNECT_TIMEOUT_MS / 1000.0) + 15
    start = time.monotonic()

    samples = []
    try:
        with open(output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "iperf_mbits_sec", "rssi_dbm"])

            while True:
                remaining = watchdog_s - (time.monotonic() - start)
                if remaining <= 0:
                    print(
                        f"\niperf3 exceeded {watchdog_s:.0f}s without finishing "
                        "(likely out of range) — killing it."
                    )
                    break

                ready, _, _ = select.select([iperf.stdout], [], [], min(remaining, 1.0))
                if not ready:
                    if iperf.poll() is not None:
                        break
                    continue

                line = iperf.stdout.readline()
                if not line:
                    if iperf.poll() is not None:
                        break
                    continue

                print(line, end="")

                match = pattern.search(line)
                if not match:
                    continue

                mbits = float(match.group(3))

                try:
                    iw_out = subprocess.check_output(["iw", "dev", interface, "link"], text=True)
                    rssi_match = re.search(r"signal:\s+(-?\d+)", iw_out)
                    rssi = int(rssi_match.group(1)) if rssi_match else None
                except Exception:
                    rssi = None

                writer.writerow([time.time(), mbits, rssi if rssi is not None else ""])
                f.flush()
                samples.append((mbits, rssi))

    except KeyboardInterrupt:
        print("\nWiFi capture stopped early by user.")

    finally:
        try:
            iperf.terminate()
            try:
                iperf.wait(timeout=3)
            except subprocess.TimeoutExpired:
                iperf.kill()
        except Exception:
            pass

    return samples


def plot_wifi_stop(csv_path, output_dir, label_prefix):
    df = pd.read_csv(csv_path)
    if df.empty:
        return

    df["time_s"] = df["timestamp"] - df["timestamp"].iloc[0]
    df["iperf_mbits_sec"] = pd.to_numeric(df["iperf_mbits_sec"], errors="coerce")
    df["rssi_dbm"] = pd.to_numeric(df["rssi_dbm"], errors="coerce")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 6.5), sharex=True)
    ax1.plot(df["time_s"].to_numpy(), df["iperf_mbits_sec"].to_numpy(), linewidth=1.5)
    ax1.set_ylabel("Throughput [Mbits/sec]")
    ax1.set_title(f"{label_prefix}: WiFi Throughput and RSSI vs Time")
    ax1.grid(True, alpha=0.3)
    ax2.plot(df["time_s"].to_numpy(), df["rssi_dbm"].to_numpy(), linewidth=1.5)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("RSSI [dBm]")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()

    path = output_dir / f"{label_prefix}_wifi.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"Saved {path}")


def run_radio_stop(output_dir, label_prefix, xctu_dir):
    print("\nNow perform the XCTU radio range-test capture.")
    print(f"Export/save the capture into: {xctu_dir}")

    while True:
        response = input(
            "Press Enter once exported (or type 'skip' to skip this stop): "
        ).strip().lower()

        if response == "skip":
            print("Skipped — no radio log for this stop.")
            return None

        files = [p for p in xctu_dir.glob("*") if p.is_file()]
        if not files:
            print(f"No file found in {xctu_dir}. Export the capture there, then press Enter again.")
            continue

        files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        chosen = files[0]
        if len(files) > 1:
            print(f"Found {len(files)} files in {xctu_dir}; using the newest: {chosen.name}")

        dest = output_dir / f"{label_prefix}_radio{chosen.suffix}"
        shutil.copy2(chosen, dest)
        print(f"Copied {chosen.name} -> {dest}")

        for stale in files:
            try:
                stale.unlink()
            except Exception as e:
                print(f"Warning: could not remove {stale} from {xctu_dir}: {e}")

        return dest


def plot_summary(summary_rows, output_dir, test_name):
    df = pd.DataFrame(summary_rows)
    df = df.dropna(subset=["target_distance_m"])
    if df.empty:
        return

    if df["mean_rssi"].notna().any():
        plt.figure(figsize=(7, 4.5))
        plt.errorbar(
            df["target_distance_m"].to_numpy(), df["mean_rssi"].to_numpy(),
            yerr=df["std_rssi"].to_numpy(), marker="o", linewidth=1.5, capsize=4
        )
        plt.xlabel("Distance from start [m]")
        plt.ylabel("Mean RSSI [dBm]")
        plt.title(f"{test_name}: WiFi RSSI vs Distance")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        path = output_dir / f"{test_name}_summary_rssi.png"
        plt.savefig(path, dpi=200)
        plt.close()
        print(f"Saved {path}")

    if df["mean_mbits"].notna().any():
        plt.figure(figsize=(7, 4.5))
        plt.errorbar(
            df["target_distance_m"].to_numpy(), df["mean_mbits"].to_numpy(),
            yerr=df["std_mbits"].to_numpy(), marker="o", linewidth=1.5, capsize=4
        )
        plt.xlabel("Distance from start [m]")
        plt.ylabel("Mean Throughput [Mbits/sec]")
        plt.title(f"{test_name}: WiFi Throughput vs Distance")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        path = output_dir / f"{test_name}_summary_throughput.png"
        plt.savefig(path, dpi=200)
        plt.close()
        print(f"Saved {path}")


def main():
    print()
    print("=== Base Station Comm Range Test (WiFi + Radio) ===")
    print()

    test_name = ask("Test name", "range_test1")
    sub_lat = ask_float("Submarine starting latitude [deg]", "0.0")
    sub_lon = ask_float("Submarine starting longitude [deg]", "0.0")
    server_ip = ask("iperf3 server IP (submarine)", DEFAULT_SERVER_IP)
    detected_interface = find_wifi_interface()
    interface = ask("WiFi interface", detected_interface or "wlan0")
    distances_str = ask("Target distances in meters, comma-separated", DEFAULT_DISTANCES)
    distances = sorted({float(d.strip()) for d in distances_str.split(",") if d.strip()})
    duration_s = ask_int("iperf3 duration per stop (seconds)", DEFAULT_DURATION_S)
    tolerance_m = ask_float("Distance tolerance (meters)", DEFAULT_TOLERANCE_M)
    max_wifi_distance_m = ask_float(
        "Max WiFi test distance in meters (radio is tested at every distance)",
        DEFAULT_MAX_WIFI_DISTANCE_M
    )
    navsat_topic = ask("Base station NavSatFix topic", DEFAULT_NAVSAT_TOPIC)
    # Default lives under ~/scripts (the same bind mount used for output_dir
    # below) since XCTU typically runs on the host, outside the container —
    # a path outside that mount would never be visible to this script.
    xctu_dir = Path(ask("Directory to export XCTU captures into", DEFAULT_XCTU_EXPORT_DIR)).expanduser()
    xctu_dir.mkdir(parents=True, exist_ok=True)

    # comm_range_test.py runs from the colcon install space, not the source
    # checkout, so __file__ can't reach the repo's scripts/ dir. Anchor on
    # $HOME instead — both the vehicle and base station dev containers bind
    # mount the repo's scripts/ directory at ~/scripts.
    output_dir = (
        Path.home() / "scripts" / "hardware_tests" / "hardware_test_logs" / test_name
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    print()
    print(f"Output folder:    {output_dir}")
    print(f"Sub start (lat,lon): {sub_lat}, {sub_lon}")
    print(f"Server IP:        {server_ip}")
    print(f"Interface:        {interface}")
    print(f"Distances:        {distances}")
    print(f"Duration/stop:    {duration_s} s")
    print(f"Tolerance:        {tolerance_m} m")
    print(f"Max WiFi distance: {max_wifi_distance_m} m (radio tested at all distances)")
    print(f"NavSatFix topic:  {navsat_topic}")
    print(f"XCTU export dir:  {xctu_dir}")
    print()

    rclpy.init()
    fix_node = BaseStationFix(navsat_topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(fix_node,), daemon=True)
    spin_thread.start()

    summary = {target: {"target_distance_m": target} for target in distances}
    wifi_targets = [d for d in reversed(distances) if d <= max_wifi_distance_m]

    try:
        print("\n=== Outbound leg: radio range test (XCTU) ===")
        print("Walk away from the submarine, stopping at each target distance below.")
        for target in distances:
            label = f"{target:g}m"
            label_prefix = f"{test_name}_{label}"

            measured = wait_until_at_distance(fix_node, sub_lat, sub_lon, target, tolerance_m)
            radio_path = run_radio_stop(output_dir, label_prefix, xctu_dir)

            summary[target]["measured_distance_radio_m"] = measured
            summary[target]["radio_log"] = str(radio_path) if radio_path else ""

            print(f"\nDone with {label}.")

        if wifi_targets:
            print("\n=== Inbound leg: WiFi throughput/RSSI test ===")
            print(
                f"Turn around and walk back toward the submarine, stopping at each "
                f"target distance at or below {max_wifi_distance_m:g} m."
            )
        else:
            print(
                f"\nNo target distances are within the {max_wifi_distance_m:g} m WiFi "
                "cutoff — skipping the WiFi leg."
            )

        for target in wifi_targets:
            label = f"{target:g}m"
            label_prefix = f"{test_name}_{label}"

            measured = wait_until_at_distance(fix_node, sub_lat, sub_lon, target, tolerance_m)

            print(f"\nRunning WiFi test at {label}...")
            wifi_csv = output_dir / f"{label_prefix}_wifi.csv"
            samples = run_wifi_stop(server_ip, interface, duration_s, wifi_csv)
            if samples:
                plot_wifi_stop(wifi_csv, output_dir, label_prefix)

            mbits_vals = [m for m, _ in samples if m is not None]
            rssi_vals = [r for _, r in samples if r is not None]

            summary[target]["measured_distance_wifi_m"] = measured
            summary[target]["mean_mbits"] = statistics.mean(mbits_vals) if mbits_vals else None
            summary[target]["std_mbits"] = statistics.pstdev(mbits_vals) if len(mbits_vals) > 1 else 0.0
            summary[target]["mean_rssi"] = statistics.mean(rssi_vals) if rssi_vals else None
            summary[target]["std_rssi"] = statistics.pstdev(rssi_vals) if len(rssi_vals) > 1 else 0.0
            summary[target]["n_samples"] = len(samples)

            print(f"\nDone with {label}.")

    except KeyboardInterrupt:
        print("\nTest sequence stopped early by user.")

    finally:
        fix_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)

        summary_rows = [summary[target] for target in distances]
        if summary_rows:
            fieldnames = [
                "target_distance_m", "measured_distance_radio_m", "radio_log",
                "measured_distance_wifi_m", "mean_mbits", "std_mbits",
                "mean_rssi", "std_rssi", "n_samples",
            ]
            summary_csv = output_dir / f"{test_name}_summary.csv"
            with open(summary_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(summary_rows)
            print(f"\nSaved {summary_csv}")
            plot_summary(summary_rows, output_dir, test_name)

        print()
        print(f"All files saved in: {output_dir}")


if __name__ == "__main__":
    main()
