#!/usr/bin/env python3

"""
radio_bridge.py

ROS 2 node that wires the bridge_core helpers to a live ROS graph and
XBee radio hardware.  All serialization logic lives in bridge_core.py;
all radio hardware logic lives in radio_manager.py.

Usage:
    ros2 run base_station_coms radio_bridge --ros-args \
        -p config_file:=/path/to/bridge.yaml \
        -p xbee_port:=/dev/ttyUSB0 \
        -p xbee_baud:=9600
"""

from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

from bridge_core import BridgeCore, TxManager
from radio_manager import XBeeRadioDevice


# ---------------------------------------------------------------------------
# QoS helper (ROS 2 specific)
# ---------------------------------------------------------------------------

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

    Publishers are stored in a dict keyed by bridge id for fast O(1) lookup,
    which also supports the radio receive path (id → publisher → republish).
    """

    def __init__(self):
        super().__init__("ros2_bridge_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("xbee_port", "/dev/ttyUSB0")
        self.declare_parameter("xbee_baud", 9600)

        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        xbee_port   = self.get_parameter("xbee_port").get_parameter_value().string_value
        xbee_baud   = self.get_parameter("xbee_baud").get_parameter_value().integer_value

        if not config_path:
            self.get_logger().fatal(
                "No config_file parameter provided. "
                "Pass it with: --ros-args -p config_file:=<path>"
            )
            raise RuntimeError("Missing config_file parameter.")

        cfg = BridgeCore.load_config(config_path)
        self.get_logger().info(f"Loaded bridge config: {config_path}")

        self._radio_device  = XBeeRadioDevice(xbee_port, xbee_baud, logger=self.get_logger())
        self._radio_device.open()
        # TODO do I want to pass the device to the TX manager? I need to specify
        # What methods it needs to have in it like a parent class or something
        self._radio_manager = TxManager(device=self._radio_device, logger=self.get_logger())

        # Keyed by bridge id → publisher.  Fast O(1) lookup on radio receive.
        self.publish_dict: dict[int, Any] = {}

        # Keep subscriber refs so they aren't garbage-collected.
        self.subs: list = []

        for entry in cfg["topics"]:
            self._setup_bridge(entry)

        # Wire the radio receive path into this node (after manager so we override its callback)
        self._radio_device.set_receive_callback(self._on_radio_receive)

        self._radio_manager.start()

    # ------------------------------------------------------------------

    def _setup_bridge(self, entry: dict) -> None:
        """Wire up one sub→pub pair from a config entry."""
        input_topic:  str = entry["input"]
        output_topic: str = entry["output"]
        type_string:  str = entry["type"]
        bridge_id:    int = int(entry["id"])
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
                f"Duplicate bridge id '{bridge_id}' - overwriting previous entry."
            )

        qos = build_qos(qos_cfg)

        pub = self.create_publisher(msg_type, output_topic, qos)
        self.publish_dict[bridge_id] = pub

        # Pre-build the field tree once so the hot callback path never rebuilds it.
        field_tree = BridgeCore.build_field_tree(allowed_fields) if allowed_fields else None

        if mode == "radio_tx":
            self._radio_manager.register_bridge(
                bridge_id   = bridge_id,
                address     = entry.get("address"),  # TODO get this from the device address same for all topics in this node.
                priority    = entry.get("priority", 5),
                reliability = entry.get("reliability", "best_effort"),
                queue_depth = entry.get("queue_depth", 10),
            )

        sub = self.create_subscription(
            msg_type,
            input_topic,
            self._make_callback(bridge_id, pub, field_tree, mode),
            qos,
        )
        self.subs.append(sub)

        field_info = f" (fields: {allowed_fields})" if allowed_fields else " (all fields)"
        self.get_logger().info(
            f"Bridge [{bridge_id}] mode={mode}: {input_topic} → {output_topic} "
            f"[{type_string}]{field_info}"
        )

    def _make_callback(
        self,
        bridge_id: int,
        pub,
        field_tree: Optional[dict],
        mode: str,
    ):
        """
        Return the appropriate callback for the given mode.

        Modes
        -----
        local      filter and republish directly on this ROS graph (default)
        radio_tx   filter, serialize to a packet, hand off to the radio
                   transmit layer.  The receiver calls receive_radio_packet()
                   to reconstruct and republish.
        """
        if mode == "local":
            def callback(msg):
                pub.publish(BridgeCore.filter_message(msg, field_tree))
            return callback

        elif mode == "radio_tx":
            def callback(msg):
                payload = BridgeCore.extract_fields_to_dict(msg, field_tree)
                packet  = BridgeCore.pack_packet(bridge_id, payload)
                self._radio_manager.enqueue(bridge_id, packet)
            return callback

        else:
            self.get_logger().warn(
                f"Unknown mode '{mode}' for bridge '{bridge_id}', defaulting to local."
            )
            def callback(msg):
                pub.publish(BridgeCore.filter_message(msg, field_tree))
            return callback

    # ------------------------------------------------------------------
    # Radio receive path
    # ------------------------------------------------------------------

    def _on_radio_receive(self, bridge_id: int, seq: int, payload: bytes) -> None:
        """Callback wired to XBeeRadioDevice for every inbound packet."""
        # TODO: keep per-device ACK seq numbers separate for multi-device setups
        self._radio_manager.ack_received(bridge_id, seq)
        self.receive_radio_packet(payload)

    def destroy_node(self) -> None:
        self._radio_manager.stop()
        self._radio_device.close()
        super().destroy_node()

    def receive_radio_packet(self, raw: bytes) -> None:
        """
        Entry point for data arriving from the radio.

        1. Unpack frame → bridge_id + payload dict
        2. Look up bridge_id → publisher
        3. Reconstruct the ROS message from the dict
        4. Publish on the output topic
        """
        try:
            bridge_id, payload = BridgeCore.unpack_packet(raw)
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
        BridgeCore.apply_dict_to_message(payload, msg)
        pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except (RuntimeError, FileNotFoundError, ValueError) as exc:
        print(f"[radio_bridge] Fatal: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
