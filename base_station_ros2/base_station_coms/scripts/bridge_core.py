#!/usr/bin/env python3

"""
bridge_core.py

Transport-agnostic bridge logic: config loading, field filtering, packet
packing/unpacking, and the transmit queue/manager.  No ROS 2 imports and
no hardware imports — usable standalone or in tests without a device.

Packet layout (little-endian):
    [2 bytes] sequence number
    [1 byte]  bridge_id (unsigned int, 0-255)
    # TODO:Do i want a checksum here? Make it optional
    [4 bytes] payload length
    [M bytes] payload (JSON-encoded UTF-8)

TODO: swap JSON for a tighter encoding (msgpack, CDR, custom struct)
      once field schema is locked down.
"""

import json
import logging
import os
import struct
import threading
import time
import yaml
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Optional


# ---------------------------------------------------------------------------
# Sequence number helpers
# ---------------------------------------------------------------------------

SEQ_MAX = 0xFFFF


def seq_next(current: int) -> int:
    return (current + 1) & SEQ_MAX


def seq_distance(a: int, b: int) -> int:
    """Signed distance from sequence number a to b, accounting for wrap."""
    d = (b - a) & SEQ_MAX
    return d if d <= SEQ_MAX // 2 else d - (SEQ_MAX + 1)


def wrap_with_seq(seq: int, raw_packet: bytes) -> bytes:
    """Prepend a 2-byte sequence number to a packet."""
    return struct.pack("<H", seq) + raw_packet


def strip_seq(framed: bytes) -> tuple[int, bytes]:
    """Split a framed packet back into (seq, raw_packet)."""
    seq = struct.unpack_from("<H", framed, 0)[0]
    return seq, framed[2:]


# ---------------------------------------------------------------------------
# Per-bridge transmit queue
# ---------------------------------------------------------------------------

@dataclass
class _QueuedPacket:
    seq: int
    data: bytes          # already framed with seq number
    attempts: int = 0
    enqueue_time: float = field(default_factory=time.monotonic)


class TxQueue:
    """
    A single bridge's transmit queue.

    Policy
    ------
    - Fixed-depth deque: when full, the *oldest* packet is dropped (keep latest).
    - Reliability mode:
        best_effort  - fire and forget; packet is removed after one send attempt.
        reliable     - packet stays in a pending-ack slot until ACK received or
                       max_retries exceeded; retried after ack_timeout seconds.
    - Thread-safe: all methods may be called from the ROS callback thread
      while the drain loop runs in the manager thread.
    """

    def __init__(
        self,
        bridge_id: int,
        address: Optional[str],        # XBee 64-bit address string, or None = broadcast
        priority: int,                 # lower = higher priority
        reliability: str,              # "best_effort" | "reliable"
        queue_depth: int,
        max_retries: int = 3,
        ack_timeout: float = 0.5,
    ):
        self.bridge_id: int = bridge_id
        self.address     = address
        self.priority    = priority
        self.reliability = reliability
        self.queue_depth = queue_depth
        self.max_retries = max_retries
        self.ack_timeout = ack_timeout

        self._lock   = threading.Lock()
        self._queue: deque[_QueuedPacket] = deque()
        self._seq    = 0

        # For reliable mode: the one packet waiting for an ACK.
        self._pending_ack: Optional[_QueuedPacket] = None
        self._pending_ack_time: float = 0.0

        # Stats
        self.stat_enqueued  = 0
        self.stat_dropped   = 0
        self.stat_sent      = 0
        self.stat_retried   = 0
        self.stat_failed    = 0

    def enqueue(self, raw_packet: bytes) -> None:
        """Add a packet to the queue, dropping the oldest if full."""
        with self._lock:
            self._seq = seq_next(self._seq)
            pkt = _QueuedPacket(
                seq=self._seq,
                data=wrap_with_seq(self._seq, raw_packet),
            )
            if len(self._queue) >= self.queue_depth:
                self._queue.popleft()
                self.stat_dropped += 1
            self._queue.append(pkt)
            self.stat_enqueued += 1

    def acknowledge(self, seq: int) -> None:
        """
        Called when the receiver sends back an ACK for sequence number seq.
        Clears the pending-ack slot so the next packet can be sent.
        """
        with self._lock:
            if self._pending_ack and self._pending_ack.seq == seq:
                self._pending_ack = None

    def peek_next(self) -> Optional[_QueuedPacket]:
        """
        Return the next packet to transmit without removing it.

        For reliable mode: if a packet is pending ACK and hasn't timed out,
        returns None (nothing new to send yet).  If it has timed out,
        returns it again for retry.

        For best_effort: returns the front of the queue (or None).
        """
        with self._lock:
            if self.reliability == "reliable":
                if self._pending_ack is not None:
                    if time.monotonic() - self._pending_ack_time < self.ack_timeout:
                        return None  # not time to retry yet
                    # Timed out - retry or give up
                    if self._pending_ack.attempts >= self.max_retries:
                        self._pending_ack = None
                        self.stat_failed += 1
                        return self._queue[0] if self._queue else None
                    self._pending_ack.attempts += 1
                    self._pending_ack_time = time.monotonic()
                    self.stat_retried += 1
                    return self._pending_ack
                return self._queue[0] if self._queue else None
            else:
                return self._queue[0] if self._queue else None

    def pop_sent(self, pkt: _QueuedPacket) -> None:
        """
        Mark a packet as sent.  For best_effort, removes it.
        For reliable, moves it to the pending-ack slot.
        """
        with self._lock:
            if self._queue and self._queue[0].seq == pkt.seq:
                self._queue.popleft()

            if self.reliability == "reliable" and self._pending_ack is None:
                self._pending_ack = pkt
                self._pending_ack.attempts = 1
                self._pending_ack_time = time.monotonic()

            self.stat_sent += 1

    def has_pending(self) -> bool:
        with self._lock:
            return bool(self._queue) or self._pending_ack is not None


# ---------------------------------------------------------------------------
# Transmit manager
# ---------------------------------------------------------------------------

class TxManager:
    """
    Manages per-bridge queues and drives the transmit loop.

    Priority scheduling
    -------------------
    Bridges are grouped into priority tiers (lower number = higher priority).
    On each drain cycle the manager:

      1. Processes *all* packets in the highest-priority non-empty tier.
      2. Only then moves to the next tier.

    Within a tier (same priority number) queues are served round-robin.

    Reliability
    -----------
    Bridges marked reliable=True get sequence numbers and the manager waits
    for an ACK (via ack_received()) before sending the next packet.  Packets
    that aren't ACKed within ack_timeout are retried up to max_retries times
    before being dropped with a warning.

    The device passed in must implement:
        device.send(address: Optional[str], framed_data: bytes) -> bool
        device.set_receive_callback(fn: Callable[[int, int, bytes], None]) -> None
    """

    def __init__(
        self,
        device,
        logger=None,
        drain_interval: float = 0.01,
    ):
        self._device         = device
        self._log            = logger or logging.getLogger(__name__)
        self._drain_interval = drain_interval

        self._queues: dict[int, TxQueue] = {}
        self._lock   = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def register_bridge(
        self,
        bridge_id: int,
        address: Optional[str]  = None,
        priority: int           = 5,
        reliability: str        = "best_effort",
        queue_depth: int        = 10,
        max_retries: int        = 3,
        ack_timeout: float      = 0.5,
    ) -> None:
        """
        Register a bridge with the manager.

        bridge_id   - unique unsigned int identifier (0-255).
        address     - XBee 64-bit hex string for unicast, or None for broadcast.
        priority    - integer tier; lower = higher priority.  Default 5.
        reliability - "best_effort" or "reliable".
        queue_depth - max queued packets; oldest dropped when exceeded.
        """
        if not (0 <= bridge_id <= 255):
            raise ValueError(f"bridge_id {bridge_id} out of range (must be 0-255).")
        if bridge_id in self._queues:
            self._log.warning(f"TxManager: re-registering bridge '{bridge_id}'.")

        self._queues[bridge_id] = TxQueue(
            bridge_id   = bridge_id,
            address     = address,
            priority    = priority,
            reliability = reliability,
            queue_depth = queue_depth,
            max_retries = max_retries,
            ack_timeout = ack_timeout,
        )
        self._log.info(
            f"Registered bridge {bridge_id} priority={priority} "
            f"reliability={reliability} depth={queue_depth}"
        )

    def start(self) -> None:
        """Start the background drain thread."""
        self._running = True
        self._thread  = threading.Thread(
            target=self._drain_loop, name="radio_tx_drain", daemon=True
        )
        self._thread.start()
        self._log.info("TxManager drain thread started.")

    def stop(self) -> None:
        """Signal the drain thread to stop and wait for it."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._log.info("TxManager stopped.")

    def enqueue(self, bridge_id: int, raw_packet: bytes) -> None:
        """
        Add raw_packet to the named bridge's queue.

        raw_packet should be the bytes produced by BridgeCore.pack_packet().
        The manager prepends the sequence number before transmitting.
        """
        q = self._queues.get(bridge_id)
        if q is None:
            self._log.warning(
                f"enqueue: unknown bridge_id '{bridge_id}'. "
                "Was register_bridge() called?"
            )
            return
        q.enqueue(raw_packet)

    def ack_received(self, bridge_id: int, seq: int) -> None:
        """
        Notify the manager that the receiver ACKed sequence number seq
        for the given bridge.  Call this from the radio receive callback.
        """
        q = self._queues.get(bridge_id)
        if q:
            q.acknowledge(seq)

    def on_receive(self, bridge_id: int, seq: int, payload: bytes) -> None:
        """
        Called by the device for every inbound packet.

        For reliable bridges, automatically handles ACK logic.
        The caller is responsible for reconstructing the ROS message from payload.

        TODO: transmit an ACK packet back to the sender.
        """
        q = self._queues.get(bridge_id)
        if q and q.reliability == "reliable":
            self._log.debug(f"[{bridge_id}] would ACK seq={seq}")

    def log_stats(self) -> None:
        """Dump per-bridge queue statistics to the logger."""
        for bridge_id, q in self._queues.items():
            self._log.info(
                f"  [{bridge_id}] priority={q.priority} reliability={q.reliability} "
                f"enqueued={q.stat_enqueued} dropped={q.stat_dropped} "
                f"sent={q.stat_sent} retried={q.stat_retried} failed={q.stat_failed}"
            )

    def _drain_loop(self) -> None:
        while self._running:
            self._drain_once()
            time.sleep(self._drain_interval)

    def _drain_once(self) -> None:
        """One full scheduling pass across all bridges."""
        # TODO understand this logic. This is a bit complex. 

        tiers: dict[int, list[TxQueue]] = {}
        for q in self._queues.values():
            tiers.setdefault(q.priority, []).append(q)

        for priority in sorted(tiers.keys()):
            tier_queues = tiers[priority]
            drained = True
            while drained:
                drained = False
                for q in tier_queues:
                    pkt = q.peek_next()
                    if pkt is None:
                        continue
                    success = self._device.send(q.address, pkt.data)
                    if success:
                        q.pop_sent(pkt)
                        drained = True
                        self._log.debug(
                            f"[{q.bridge_id}] sent seq={pkt.seq} "
                            f"({len(pkt.data)} bytes) priority={q.priority}"
                        )
                    else:
                        self._log.warning(
                            f"[{q.bridge_id}] hardware send failed for seq={pkt.seq}"
                        )

            if any(q.has_pending() for q in tier_queues):
                break


# ---------------------------------------------------------------------------
# BridgeCore — config, field filtering, packet packing/unpacking
# ---------------------------------------------------------------------------

class BridgeCore:
    """
    Stateless utilities for config loading, ROS message field filtering,
    and radio packet serialization.  All methods are static.
    """

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    @staticmethod
    def load_config(path: str) -> dict:
        """Load and validate the YAML bridge configuration."""
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Config file not found: {path}")

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
            bridge_id = int(entry["id"])
            if not (0 <= bridge_id <= 255):
                raise ValueError(
                    f"bridge_id {bridge_id} out of range (must be 0-255)."
                )

        return cfg

    # ------------------------------------------------------------------
    # Field filtering
    # ------------------------------------------------------------------

    @staticmethod
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
                    node[part] = None
                if i < len(parts) - 1:
                    if node[part] is None:
                        node[part] = {}
                    node = node[part]
        return tree

    @staticmethod
    def _apply_field_tree(src_msg: Any, dst_msg: Any, tree: dict) -> None:
        """Recursively copy fields from src_msg into dst_msg according to tree."""
        try:
            valid_fields = set(src_msg.get_fields_and_field_types().keys())
        except AttributeError:
            return

        for field_name, subtree in tree.items():
            if field_name not in valid_fields:
                continue
            src_value = getattr(src_msg, field_name)
            if subtree is None:
                setattr(dst_msg, field_name, src_value)
            else:
                dst_value = getattr(dst_msg, field_name)
                BridgeCore._apply_field_tree(src_value, dst_value, subtree)
                setattr(dst_msg, field_name, dst_value)

    @staticmethod
    def filter_message(src_msg: Any, field_tree: Optional[dict]) -> Any:
        """
        Return a new message of the same type, copying only fields in field_tree.
        If field_tree is None, the entire message is forwarded as-is.
        """
        if not field_tree:
            return src_msg

        dst_msg = type(src_msg)()
        BridgeCore._apply_field_tree(src_msg, dst_msg, field_tree)
        return dst_msg

    @staticmethod
    def extract_fields_to_dict(msg: Any, field_tree: Optional[dict]) -> dict:
        """
        Walk a message and return a plain Python dict of only the selected
        leaf values.  This is what gets packed into the radio frame.
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
                out[k] = _recurse(getattr(src, k), subtree)
            return out

        return _recurse(msg, field_tree)

    # ------------------------------------------------------------------
    # Packet packing / unpacking
    # ------------------------------------------------------------------

    @staticmethod
    def pack_packet(bridge_id: int, payload: dict) -> bytes:
        """
        Serialize a bridge packet for transmission over the radio link.

        Packet layout (little-endian):
            [1 byte]  bridge_id (unsigned int, 0-255)
            [4 bytes] payload length
            [M bytes] payload (JSON-encoded UTF-8)
        """
        payload_bytes = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        return (
            struct.pack("<B", bridge_id)
            + struct.pack("<I", len(payload_bytes))
            + payload_bytes
        )

    @staticmethod
    def unpack_packet(raw: bytes) -> tuple[int, dict]:
        """
        Deserialize a packet produced by pack_packet.
        Returns (bridge_id, payload_dict).
        """
        offset = 0
        bridge_id = struct.unpack_from("<B", raw, offset)[0];  offset += 1
        pay_len   = struct.unpack_from("<I", raw, offset)[0];  offset += 4
        payload   = json.loads(raw[offset : offset + pay_len].decode("utf-8"))
        return bridge_id, payload

    @staticmethod
    def apply_dict_to_message(data: dict, dst_msg: Any) -> None:
        """
        Recursively write a plain dict (from unpack_packet) back into a
        message object.  Used on the receiving end to reconstruct the message.
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
                BridgeCore.apply_dict_to_message(v, nested)
                setattr(dst_msg, k, nested)
            else:
                setattr(dst_msg, k, v)
