#!/usr/bin/env python3

"""
ros2_bridge_node.py

A type-agnostic ROS 2 bridge that forwards any standard message type,
with optional field filtering, configured entirely via YAML.

Usage:
    ros2 run <your_package> ros2_bridge_node --ros-args -p config_file:=bridge.yaml

Dependencies:
    - rclpy
    - rosidl_runtime_py
    - pyyaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rosidl_runtime_py.utilities import get_message

import yaml
import os
from typing import Any, Optional

# Thoughts:

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_config(path: str) -> dict:
    """Load and validate the YAML bridge configuration."""
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)

    if "topics" not in cfg or not isinstance(cfg["topics"], list):
        raise ValueError("Config must have a top-level 'topics' list.")

    for entry in cfg["topics"]:
        for required in ("input", "output", "type"):
            if required not in entry:
                raise ValueError(
                    f"Each topic entry must have '{required}'. Got: {entry}"
                )

    return cfg


def build_field_tree(allowed_fields: list) -> dict:
    """
    Convert a flat list of dot-notation field specs into a nested dict tree.

    None as a value means "copy this whole field, no further filtering".

    Examples:
        ["linear_acceleration", "header.stamp.sec", "header.frame_id"]
        ->  {
              "linear_acceleration": None,
              "header": {
                  "stamp": {"sec": None},
                  "frame_id": None,
              }
            }
    """
    tree = {}
    for spec in allowed_fields:
        parts = spec.split(".")
        node = tree
        for i, part in enumerate(parts):
            if part not in node:
                node[part] = None  # leaf by default
            if i < len(parts) - 1:
                # Need to go deeper; upgrade None leaf to a sub-tree dict
                if node[part] is None:
                    node[part] = {}
                node = node[part]
            # else: last part stays None (whole field)
    return tree


def _apply_field_tree(src_msg: Any, dst_msg: Any, tree: dict) -> None:
    """
    Recursively copy fields from src_msg into dst_msg according to tree.
    Works to arbitrary depth.
    """
    try:
        valid_fields = set(src_msg.get_fields_and_field_types().keys())
    except AttributeError:
        # Primitive value – nothing to recurse into
        return

    for field_name, subtree in tree.items():
        if field_name not in valid_fields:
            continue  # silently skip unknown fields

        src_value = getattr(src_msg, field_name)

        if subtree is None:
            # Leaf: copy the whole field
            setattr(dst_msg, field_name, src_value)
        else:
            # Internal node: recurse into the nested message
            dst_value = getattr(dst_msg, field_name)
            _apply_field_tree(src_value, dst_value, subtree)
            setattr(dst_msg, field_name, dst_value)


def filter_message(src_msg: Any, allowed_fields: Optional[list]) -> Any:
    """
    Return a new message of the same type, copying only allowed_fields.

    If allowed_fields is None or empty, the entire message is forwarded as-is.
    Supports arbitrary-depth dot-notation, e.g. "header.stamp.sec".
    """
    if not allowed_fields:
        return src_msg  # pass-through, no copy needed
    
    # We are building the tree every time here and it is not changing mid run ever
    # Should just need to apply the tree every time. 

    tree = build_field_tree(allowed_fields)
    dst_msg = type(src_msg)()
    _apply_field_tree(src_msg, dst_msg, tree)
    return dst_msg


def build_qos(qos_cfg: Optional[dict]) -> QoSProfile:
    """Build a QoSProfile from an optional dict, falling back to sensible defaults."""
    if not qos_cfg:
        return QoSProfile(depth=10)

    reliability = (
        ReliabilityPolicy.BEST_EFFORT
        if qos_cfg.get("reliability", "reliable").lower() == "best_effort"
        else ReliabilityPolicy.RELIABLE
    )
    durability = (
        DurabilityPolicy.TRANSIENT_LOCAL
        if qos_cfg.get("durability", "volatile").lower() == "transient_local"
        else DurabilityPolicy.VOLATILE
    )
    depth = int(qos_cfg.get("depth", 10))

    return QoSProfile(
        reliability=reliability,
        durability=durability,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Bridge Node
# ---------------------------------------------------------------------------

class BridgeNode(Node):
    """
    Dynamically creates subscriber/publisher pairs for every topic
    defined in the YAML config.
    """

    def __init__(self):
        super().__init__("ros2_bridge_node")

        # Declare the config_file parameter so it can be set via CLI
        self.declare_parameter("config_file", "")
        config_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_path:
            self.get_logger().fatal(
                "No config_file parameter provided. "
                "Pass it with: --ros-args -p config_file:=<path>"
            )
            raise RuntimeError("Missing config_file parameter.")

        if not os.path.isfile(config_path):
            self.get_logger().fatal(f"Config file not found: {config_path}")
            raise FileNotFoundError(config_path)

        cfg = load_config(config_path)
        self.get_logger().info(f"Loaded bridge config: {config_path}")

        # Keep references so they aren't garbage-collected
        self._subscribers = []
        self._publishers = []

        for entry in cfg["topics"]:
            self._setup_bridge(entry)

    # ------------------------------------------------------------------

    def _setup_bridge(self, entry: dict) -> None:
        """Wire up one sub→pub pair from a config entry."""
        input_topic: str = entry["input"]
        output_topic: str = entry["output"]
        type_string: str = entry["type"]
        id_string: str = entry["id"]
        allowed_fields: Optional[list] = entry.get("fields")
        qos_cfg: Optional[dict] = entry.get("qos")

        try:
            msg_type = get_message(type_string)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().error(
                f"Could not resolve message type '{type_string}': {exc}. "
                f"Skipping bridge {input_topic} → {output_topic}."
            )
            return

        qos = build_qos(qos_cfg)

        pub = self.create_publisher(msg_type, output_topic, qos)
        self._publishers.append(pub) # Maybe instead of a list of publisher you do a dict of publishers based on id string

        # I want this to expand to support more than just publishing the message back out to ros2
        # this is a good proof of concept but the end goal is to create a minimal data packet with the id as the header that
        # Can be sent over the radio and then unpacked on the other side and republished as the full message.
        
        # Close over the variables for this specific bridge
        def make_callback(_pub, _allowed_fields):
            def callback(msg):
                out_msg = filter_message(msg, _allowed_fields)
                _pub.publish(out_msg)
            return callback

        sub = self.create_subscription(
            msg_type,
            input_topic,
            make_callback(pub, allowed_fields),
            qos,
        )
        self._subscribers.append(sub)

        field_info = (
            f" (fields: {allowed_fields})" if allowed_fields else " (all fields)"
        )
        self.get_logger().info(
            f"Bridge {id_string}: {input_topic} → {output_topic} [{type_string}]{field_info}"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except (RuntimeError, FileNotFoundError) as exc:
        # Already logged inside __init__; exit cleanly
        print(f"[ros2_bridge_node] Fatal: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()