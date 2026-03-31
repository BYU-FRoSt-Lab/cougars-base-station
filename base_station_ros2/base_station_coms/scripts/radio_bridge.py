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
import struct
import json
from typing import Any, Optional


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
        for required in ("input", "output", "type", "id"):
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
        # Primitive value  nothing to recurse into
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


def filter_message(src_msg: Any, field_tree: Optional[dict]) -> Any:
    """
    Return a new message of the same type, copying only fields in field_tree.

    If field_tree is None, the entire message is forwarded as-is.
    Accepts a pre-built tree (from build_field_tree) rather than rebuilding
    it on every call.
    """
    if not field_tree:
        return src_msg  # pass-through, no copy needed

    dst_msg = type(src_msg)()
    _apply_field_tree(src_msg, dst_msg, field_tree)
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
# Radio packet layer (stub  replace internals without changing the interface)
# ---------------------------------------------------------------------------

def extract_fields_to_dict(msg: Any, field_tree: Optional[dict]) -> dict:
    """
    Walk a (possibly filtered) message and return a plain Python dict of
    only the selected leaf values.  This is what gets packed into the
    radio frame.

    field_tree=None means extract everything.
    """
    def _recurse(src, tree):
        try:
            valid = set(src.get_fields_and_field_types().keys())
        except AttributeError:
            return src  # primitive leaf

        keys = valid if tree is None else {k for k in tree if k in valid}
        out = {}
        for k in keys:
            subtree = None if tree is None else tree[k]
            val = getattr(src, k)
            out[k] = _recurse(val, subtree)
        return out

    return _recurse(msg, field_tree)


def pack_radio_packet(bridge_id: str, payload: dict) -> bytes:
    """
    Serialize a bridge packet for transmission over the radio link.

    Packet layout (little-endian):
        [2 bytes] id length
        [N bytes] id string (UTF-8)  # Does this need to be a string how bout a numeric id?
        [4 bytes] payload length
        [M bytes] payload (JSON-encoded UTF-8)

        # Do i want a check some here. I want a the tighter encodig. probably a custom struct for each topic. in the list.

    TODO: swap JSON for a tighter encoding (msgpack, CDR, custom struct)
          once field schema is locked down.
    """
    id_bytes      = bridge_id.encode("utf-8")
    payload_bytes = json.dumps(payload, separators=(",", ":")).encode("utf-8")

    return (
        struct.pack("<H", len(id_bytes))
        + id_bytes
        + struct.pack("<I", len(payload_bytes))
        + payload_bytes
    )


def unpack_radio_packet(raw: bytes) -> tuple[str, dict]:
    """
    Deserialize a packet produced by pack_radio_packet.

    Returns (bridge_id, payload_dict).
    Mirror image of pack_radio_packet  keep them in sync.
    """
    offset = 0
    id_len    = struct.unpack_from("<H", raw, offset)[0];  offset += 2
    bridge_id = raw[offset : offset + id_len].decode("utf-8"); offset += id_len
    pay_len   = struct.unpack_from("<I", raw, offset)[0];  offset += 4
    payload   = json.loads(raw[offset : offset + pay_len].decode("utf-8"))
    return bridge_id, payload


def apply_dict_to_message(data: dict, dst_msg: Any) -> None:
    """
    Recursively write a plain dict (from unpack_radio_packet) back into a
    ROS 2 message object.  Used on the receiving end to reconstruct the
    message before republishing.
    """
    try:
        valid = set(dst_msg.get_fields_and_field_types().keys())
    except AttributeError:
        return

    for k, v in data.items():
        if k not in valid:
            continue
        if isinstance(v, dict):
            nested = getattr(dst_msg, k)
            apply_dict_to_message(v, nested)
            setattr(dst_msg, k, nested)
        else:
            setattr(dst_msg, k, v)


# ---------------------------------------------------------------------------
# Bridge Node
# ---------------------------------------------------------------------------

class BridgeNode(Node):
    """
    Dynamically creates subscriber/publisher pairs for every topic
    defined in the YAML config.

    Publishers are stored in a dict keyed by bridge id for fast O(1) lookup,
    which also supports the radio receive path (id → publisher → republish).
    """

    def __init__(self):
        super().__init__("ros2_bridge_node")

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

        # Keyed by bridge id → publisher.  Fast O(1) lookup on radio receive.
        self.publish_dict: dict[str, Any] = {}

        # Keep subscriber refs so they aren't garbage-collected.
        self.subscribers: list = []

        for entry in cfg["topics"]:
            self._setup_bridge(entry)

    # ------------------------------------------------------------------

    def _setup_bridge(self, entry: dict) -> None:
        """Wire up one sub→pub pair from a config entry."""
        input_topic:  str = entry["input"]
        output_topic: str = entry["output"]
        type_string:  str = entry["type"]
        bridge_id:    str = entry["id"]
        allowed_fields    = entry.get("fields")
        qos_cfg           = entry.get("qos")
        # mode: "local" | "radio_tx"
        # "radio_rx" is handled by receive_radio_packet(), not a subscription.
        mode: str         = entry.get("mode", "local")

        try:
            msg_type = get_message(type_string)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().error(
                f"Could not resolve message type '{type_string}': {exc}. "
                f"Skipping bridge {input_topic} → {output_topic}."
            )
            return

        if bridge_id in self.publish_dict:
            self.get_logger().warn(
                f"Duplicate bridge id '{bridge_id}'  overwriting previous entry."
            )

        qos = build_qos(qos_cfg)

        pub = self.create_publisher(msg_type, output_topic, qos)
        self.publish_dict[bridge_id] = pub

        # Pre-build the field tree once so the hot callback path never rebuilds it.
        field_tree = build_field_tree(allowed_fields) if allowed_fields else None

        sub = self.create_subscription(
            msg_type,
            input_topic,
            self._make_callback(bridge_id, pub, field_tree, mode),
            qos,
        )
        self.subscribers.append(sub)

        field_info = f" (fields: {allowed_fields})" if allowed_fields else " (all fields)"
        self.get_logger().info(
            f"Bridge [{bridge_id}] mode={mode}: {input_topic} → {output_topic} "
            f"[{type_string}]{field_info}"
        )

    def _make_callback(
        self,
        bridge_id: str,
        pub,
        field_tree: Optional[dict],
        mode: str,
    ):
        """
        Return the appropriate callback for the given mode.

        Modes
        -----
        local      filter and republish directly on this ROS graph (default)
        radio_tx   filter, serialize to a minimal packet, hand off to the
                    radio transmit layer.  The receiver calls
                    receive_radio_packet() to reconstruct and republish.
        """
        if mode == "local":
            def callback(msg):
                pub.publish(filter_message(msg, field_tree))
            return callback

        elif mode == "radio_tx":
            def callback(msg):
                payload = extract_fields_to_dict(msg, field_tree)
                packet  = pack_radio_packet(bridge_id, payload)
                # TODO: hand `packet` to the radio transmit interface, e.g.:
                # Tell the radio the bridge the id and the quality. 
                #   self._radio.send(packet)
                self.get_logger().debug(
                    f"[{bridge_id}] radio_tx {len(packet)} bytes"
                )
            return callback

        else:
            self.get_logger().warn(
                f"Unknown mode '{mode}' for bridge '{bridge_id}', defaulting to local."
            )
            def callback(msg):
                pub.publish(filter_message(msg, field_tree))
            return callback

    # ------------------------------------------------------------------
    # Radio receive path
    # ------------------------------------------------------------------

    def receive_radio_packet(self, raw: bytes) -> None:
        """
        Entry point for data arriving from the radio on the receiving robot.

        1. Unpack frame → bridge_id + payload dict
        2. Look up bridge_id → publisher
        3. Reconstruct the ROS message from the dict
        4. Publish on the output topic

        Wire this to whatever delivers bytes from your radio driver
        a serial read callback, UDP socket listener, etc.
        """
        try:
            bridge_id, payload = unpack_radio_packet(raw)
        except Exception as exc:
            self.get_logger().error(f"Failed to unpack radio packet: {exc}")
            return

        pub = self.publish_dict.get(bridge_id)
        if pub is None:
            self.get_logger().warn(
                f"Radio packet for unknown bridge id '{bridge_id}'. "
                "Is this entry missing from the receiver's config?"
            )
            return

        msg = pub.msg_type()
        apply_dict_to_message(payload, msg)
        pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"[ros2_bridge_node] Fatal: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()