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

from typing import Any, Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

from bridge_core import BridgeCore, TxManager
from comms_device import CommsDevice
from radio_manager import XBeeRadioDevice
from ros_topic_device import RosTopicDevice


# QoS helper (ROS 2 specific)

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
    return QoSProfile(
        reliability=reliability,
        durability=durability,
        depth=int(qos_cfg.get("depth", 10)),
    )


# Bridge Node

class BridgeNode(Node):
    """
    Dynamically creates subscriber/publisher pairs for every topic defined
    in the YAML config.

    Modes
    -----
    local      subscribe → filter → republish on the local ROS graph
    radio_tx   subscribe → encode → enqueue for transmission over XBee
    radio_rx   receive bytes from XBee → decode → publish on local ROS graph
               (no subscription; driven by _on_radio_receive)
    """

    def __init__(self):
        super().__init__("ros2_bridge_node")

        self.declare_parameter("config_file", "")
        self.declare_parameter("xbee_port", "/dev/ttyUSB0")
        self.declare_parameter("xbee_baud", 9600)
        self.declare_parameter("device_id", 1)
        self.declare_parameter("sim_mode", False)
        self.declare_parameter("sim_tx_topic", "/radio_sim/tx")
        self.declare_parameter("sim_rx_topic", "/radio_sim/rx")

        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        xbee_port   = self.get_parameter("xbee_port").get_parameter_value().string_value
        xbee_baud   = self.get_parameter("xbee_baud").get_parameter_value().integer_value
        device_id   = self.get_parameter("device_id").get_parameter_value().integer_value
        sim_mode    = self.get_parameter("sim_mode").get_parameter_value().bool_value
        sim_tx      = self.get_parameter("sim_tx_topic").get_parameter_value().string_value
        sim_rx      = self.get_parameter("sim_rx_topic").get_parameter_value().string_value

        if not (1 <= device_id <= 255):
            raise ValueError(f"device_id {device_id} out of range (must be 1-255).")

        if not config_path:
            self.get_logger().fatal(
                "No config_file parameter provided. "
                "Pass it with: --ros-args -p config_file:=<path>"
            )
            raise RuntimeError("Missing config_file parameter.")

        cfg = BridgeCore.load_config(config_path)
        self.get_logger().info(f"Loaded bridge config: {config_path}")

        if sim_mode:
            self._radio_device: CommsDevice = RosTopicDevice(
                self, sim_tx, sim_rx, device_id=device_id, logger=self.get_logger()
            )
        else:
            self._radio_device: CommsDevice = XBeeRadioDevice(
                xbee_port, xbee_baud, logger=self.get_logger(), device_id=device_id
            )
        self._radio_device.open()
        self._radio_manager = TxManager(device=self._radio_device, logger=self.get_logger())

        # bridge_id → publisher  (used by radio_rx receive path)
        self.publish_dict: dict[int, Any] = {}

        # bridge_id → decode_fn  (built at setup, used by receive_radio_packet)
        self._decoders: dict[int, Callable] = {}

        # Keep subscriber refs so they aren't garbage-collected.
        self.subs: list = []

        for entry in cfg["topics"]:
            self._setup_bridge(entry)

        # Wire receive path after manager so we own the callbacks
        self._radio_device.set_receive_callback(self._on_radio_receive)
        self._radio_device.set_ack_callback(self._on_ack_receive)
        self._radio_manager.start()

    # ------------------------------------------------------------------

    def _setup_bridge(self, entry: dict) -> None:
        """Wire up one bridge from a config entry."""
        input_topic:  str = entry["input"]
        output_topic: str = entry["output"]
        type_string:  str = entry["type"]
        bridge_id:    int = int(entry["id"])
        allowed_fields    = entry.get("fields")
        qos_cfg           = entry.get("qos")
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
            self.get_logger().warn(f"Duplicate bridge id '{bridge_id}' — overwriting.")

        qos        = build_qos(qos_cfg)
        field_tree = BridgeCore.build_field_tree(allowed_fields) if allowed_fields else None

        # Build binary codec for radio modes
        if mode in ("radio_tx", "radio_rx"):
            encode_fn, decode_fn = BridgeCore.build_codec(msg_type, field_tree, get_message)
            self._decoders[bridge_id] = decode_fn
        else:
            encode_fn = None

        pub = self.create_publisher(msg_type, output_topic, qos)
        self.publish_dict[bridge_id] = pub

        if mode == "radio_tx":
            self._radio_manager.register_bridge(
                bridge_id   = bridge_id,
                address     = entry.get("address"),         # Destination addres or none for broadcast
                priority    = entry.get("priority", 5),     # Lower the higher priority
                reliability = entry.get("reliability", "best_effort"),    # Whether to retry failed transmissions
                queue_depth = entry.get("queue_depth", 10),   # How many packets to hold on to and keep trying.
            )
            sub = self.create_subscription(
                msg_type, input_topic,
                self._make_radio_tx_callback(bridge_id, encode_fn),
                qos,
            )
            self.subs.append(sub)

        elif mode == "local":
            sub = self.create_subscription(
                msg_type, input_topic,
                self._make_local_callback(pub, field_tree),
                qos,
            )
            self.subs.append(sub)

        elif mode == "radio_rx":
            pass  # driven entirely by _on_radio_receive; no subscription needed

        else:
            self.get_logger().warn(f"Unknown mode '{mode}' for bridge {bridge_id}, defaulting to local.")
            sub = self.create_subscription(
                msg_type, input_topic,
                self._make_local_callback(pub, field_tree),
                qos,
            )
            self.subs.append(sub)

        field_info = f" fields={allowed_fields}" if allowed_fields else ""
        self.get_logger().info(
            f"Bridge [{bridge_id}] mode={mode}: {input_topic} → {output_topic} "
            f"[{type_string}]{field_info}"
        )

    @staticmethod
    def _make_local_callback(pub, field_tree: Optional[dict]):
        def callback(msg):
            pub.publish(BridgeCore.filter_message(msg, field_tree))
        return callback

    def _make_radio_tx_callback(self, bridge_id: int, encode_fn: Callable):
        def callback(msg):
            payload = encode_fn(msg)
            packet  = BridgeCore.pack_packet(bridge_id, payload)
            self._radio_manager.enqueue(bridge_id, packet)
        return callback

    # ------------------------------------------------------------------
    # Radio receive path
    # ------------------------------------------------------------------

    def _on_radio_receive(
        self, bridge_id: int, seq: int, payload: bytes, src_id: int, src_hw_addr: Optional[str] = None
    ) -> None:
        """Callback wired to the CommsDevice for every inbound DATA packet."""
        self.get_logger().debug(f"RX from device {src_id}: bridge={bridge_id} seq={seq}")
        self._radio_manager.on_receive(bridge_id, seq, src_hw_addr)
        self.receive_radio_packet(payload)

    def _on_ack_receive(self, bridge_id: int, seq: int, src_id: int) -> None:
        """Callback wired to the CommsDevice for every inbound ACK packet."""
        self.get_logger().debug(f"ACK from device {src_id}: bridge={bridge_id} seq={seq}")
        self._radio_manager.ack_received(bridge_id, seq)

    def destroy_node(self) -> None:
        self._radio_manager.stop()
        self._radio_device.close()
        super().destroy_node()

    def receive_radio_packet(self, raw: bytes) -> None:
        """
        Decode a raw radio packet and republish on the output topic.

        1. Unpack envelope → bridge_id + payload bytes
        2. Look up bridge_id → publisher + decoder
        3. Decode payload into a dict, reconstruct the ROS message
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
                f"Radio packet for unknown bridge id {bridge_id}. "
                "Is this entry missing from the receiver's config?"
            )
            return

        decode = self._decoders.get(bridge_id)
        if decode is None:
            self.get_logger().warn(f"No decoder for bridge id {bridge_id}.")
            return

        try:
            data = decode(payload)
        except Exception as exc:
            self.get_logger().error(f"Decode failed for bridge {bridge_id}: {exc}")
            return

        msg = pub.msg_type()
        BridgeCore.apply_dict_to_message(data, msg)
        pub.publish(msg)


# Entry point

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
