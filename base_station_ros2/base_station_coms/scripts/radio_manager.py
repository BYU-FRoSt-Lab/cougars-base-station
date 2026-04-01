#!/usr/bin/env python3

"""
radio_manager.py

Two-class radio transmit layer for the ROS 2 bridge:

    XBeeRadioDevice   - thin wrapper around digi.xbee.  Owns the serial
                        port.  Everything else is policy-free.

    RadioTxManager    - owns per-bridge queues, priority scheduling, sequence
                        numbers, and RELIABLE retry logic.  Calls
                        XBeeRadioDevice.send() when it decides to transmit.

Usage (from BridgeNode):

    device  = XBeeRadioDevice(port="/dev/ttyUSB0", baud=9600, logger=...)
    manager = RadioTxManager(device=device, logger=...)
    manager.register_bridge(
        bridge_id="imu_radio",
        address="0013A20041234567",   # 64-bit XBee address string, or None = broadcast
        priority=1,                   # lower number = higher priority
        reliability="best_effort",    # or "reliable"
        queue_depth=5,                # max packets before oldest is dropped
    )
    manager.start()

    # In a ROS callback:
    manager.enqueue("imu_radio", packet_bytes)

    # On shutdown:
    manager.stop()
    device.close()
"""

import threading
import time
import struct
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

# digi.xbee imports - guarded so the module can be imported in unit tests
# without hardware present.
try:
    from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
    from digi.xbee.exception import TransmitException
    _XBEE_AVAILABLE = True
except ImportError:
    _XBEE_AVAILABLE = False


# Sequence number helpers
# 
# Sequence numbers are 2-byte unsigned ints (0-65535) that wrap around.
# They are prepended to every packet so the receiver can detect drops and
# acknowledge frames.
#
# Packet wire format (little-endian):
#   [2 bytes] sequence number
#   [2 bytes] bridge_id length
#   [N bytes] bridge_id (UTF-8)
#   [4 bytes] payload length
#   [M bytes] payload bytes  (as produced by pack_radio_packet in bridge node) 
# TODO fix this to my desired header format. 
#
# The sequence number lives outside the existing pack_radio_packet envelope
# so that the packet layer doesn't need to know about message contents.

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

# Per-bridge queue

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
        bridge_id: str,
        address: Optional[str],        # XBee 64-bit address string, or None = broadcast
        priority: int,                 # lower = higher priority
        reliability: str,              # "best_effort" | "reliable"
        queue_depth: int,
        max_retries: int = 3,          # Put these as ros params
        ack_timeout: float = 0.5,      # Put as ros params
    ):
        self.bridge_id   = bridge_id
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

    # Public API (called from ROS callback thread)

    def enqueue(self, raw_packet: bytes) -> None:
        """Add a packet to the queue, dropping the oldest if full."""
        with self._lock:
            self._seq = seq_next(self._seq)
            pkt = _QueuedPacket(
                seq=self._seq,
                data=wrap_with_seq(self._seq, raw_packet),
            )
            if len(self._queue) >= self.queue_depth:
                self._queue.popleft()   # drop oldest # TODO check this
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

    # Internal API (called from manager drain loop - same lock required)

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
                    # Still waiting for ACK
                    if time.monotonic() - self._pending_ack_time < self.ack_timeout:
                        return None  # not time to retry yet
                    # Timed out - retry or give up
                    if self._pending_ack.attempts >= self.max_retries:
                        self._pending_ack = None  # give up
                        self.stat_failed += 1
                        return self._queue[0] if self._queue else None
                    self._pending_ack.attempts += 1
                    self._pending_ack_time = time.monotonic()
                    self.stat_retried += 1
                    return self._pending_ack
                # No pending ack - grab next from queue
                return self._queue[0] if self._queue else None
            else:
                return self._queue[0] if self._queue else None

    def pop_sent(self, pkt: _QueuedPacket) -> None:
        """
        Mark a packet as sent.  For best_effort, removes it.
        For reliable, moves it to the pending-ack slot.
        """
        with self._lock:
            # Only pop if it's the front (guard against races)
            # Can test this method for sure but it would be def 

            # TODO: Faster if you could transmit a few and wait for acks before popping. 
            # Will look at doing this later.
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


# XBee device wrapper

class XBeeRadioDevice:
    """
    Thin, policy-free wrapper around digi.xbee.

    Responsibilities
    ----------------
    - Open / close the serial port.
    - send(address, data) - unicast if address given, broadcast otherwise. 
    - set_receive_callback(fn) - fn(bridge_id: str, seq: int, payload: bytes)

    Everything else (queuing, priority, retries) lives in RadioTxManager.
    """

    def __init__(self, port: str, baud: int, logger=None):
        self._port   = port
        self._baud   = baud
        self._log    = logger or logging.getLogger(__name__)
        self._device = None
        self._rx_callback: Optional[Callable] = None

        if not _XBEE_AVAILABLE:
            self._log.warning(
                "digi.xbee not installed - XBeeRadioDevice running in stub mode."
            )


    def open(self) -> bool:
        if not _XBEE_AVAILABLE:
            self._log.warning("XBee stub: open() called, no hardware.")
            return False
        try:
            self._device = XBeeDevice(self._port, self._baud)
            self._device.open()
            self._device.add_data_received_callback(self._raw_rx_callback)
            self._log.info(f"XBee opened on {self._port} @ {self._baud} baud.")
            return True
        except Exception as exc:
            self._log.error(f"XBee open failed: {exc}")
            return False

    def close(self) -> None:
        if self._device and self._device.is_open():
            self._device.close()
            self._log.info("XBee device closed.")

    def set_receive_callback(self, fn: Callable[[str, int, bytes], None]) -> None:
        """
        Register a callback invoked on every received packet.

        fn(bridge_id: str, seq: int, payload: bytes)
        """
        self._rx_callback = fn


    def send(self, address: Optional[str], framed_data: bytes) -> bool:
        """
        Transmit framed_data to address (64-bit hex string) or broadcast.

        Returns True on success.
        """
        if not _XBEE_AVAILABLE or self._device is None:
            # Stub: pretend it worked
            self._log.debug(
                f"XBee stub send {len(framed_data)} bytes to {address or 'broadcast'}"
            )
            return True

        try:
            if address:
                # TODO what is this doing
                remote = RemoteXBeeDevice(
                    self._device, XBee64BitAddress.from_hex_string(address)
                )
                self._device.send_data(remote, framed_data)
            else:
                self._device.send_data_broadcast(framed_data)
            return True
        except TransmitException as exc:
            self._log.error(f"XBee TransmitException: {exc}")
            return False
        except Exception as exc:
            self._log.error(f"XBee send error: {exc}")
            return False


    def _raw_rx_callback(self, xbee_message) -> None:
        """Internal digi.xbee callback - strip seq and forward."""
        try:
            raw = xbee_message.data
            seq, payload = strip_seq(raw)

            # The payload still has the bridge_id envelope from pack_radio_packet.
            # Decode bridge_id so we can route it.
            id_len   = struct.unpack_from("<H", payload, 0)[0]
            bridge_id = payload[2 : 2 + id_len].decode("utf-8")

            if self._rx_callback:
                self._rx_callback(bridge_id, seq, payload)
        except Exception as exc:
            self._log.error(f"XBee RX parse error: {exc}")


# Transmit manager

class TxManager:
    """
    Manages per-bridge queues and drives the transmit loop.

    Priority scheduling
    -------------------
    Bridges are grouped into priority tiers (lower number = higher priority).
    On each drain cycle the manager:

      1. Processes *all* packets in the highest-priority non-empty tier.
      2. Only then moves to the next tier.

    Within a tier (same priority number) queues are served round-robin so
    no single bridge starves the others.

    Reliability
    -----------
    Bridges marked reliable=True get sequence numbers and the manager waits
    for an ACK (via ack_received()) before sending the next packet.  Packets
    that aren't ACKed within ack_timeout are retried up to max_retries times
    before being dropped with a warning.

    best_effort bridges send and immediately forget.
    """

    def __init__(
        self,
        device: XBeeRadioDevice,
        logger=None,
        drain_interval: float = 0.01,   # seconds between drain loop iterations, manages CPU load. 
    ):
        self._device         = device
        self._log            = logger or logging.getLogger(__name__) # TODO what is this?
        self._drain_interval = drain_interval

        self._queues: dict[str, TxQueue] = {}   # bridge_id → queue
        self._lock   = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

        # Wire up receive path
        self._device.set_receive_callback(self._on_receive)

    # Registration (call before start())

    def register_bridge(
        self,
        bridge_id: str,
        address: Optional[str]  = None,  
        priority: int           = 5,
        reliability: str        = "best_effort",
        queue_depth: int        = 10,
        max_retries: int        = 3,
        ack_timeout: float      = 0.5,
    ) -> None:
        """
        Register a bridge with the manager.

        Parameters mirror the YAML config plus XBee address.

        bridge_id   - unique string identifier for this bridge (e.g. "imu_radio").
        address     - XBee 64-bit hex string for unicast, or None for broadcast

        priority    - integer tier; lower = higher priority.  Default 5.
        reliability - "best_effort" or "reliable".
        queue_depth - max queued packets; oldest dropped when exceeded.
        """
        if bridge_id in self._queues:
            self._log.warning(f"RadioTxManager: re-registering bridge '{bridge_id}'.")

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
            f"Registered radio bridge '{bridge_id}' priority={priority} "
            f"reliability={reliability} depth={queue_depth}"
        )

    # Lifecycle

    def start(self) -> None:
        """Start the background drain thread."""
        self._running = True
        self._thread  = threading.Thread(
            target=self._drain_loop, name="radio_tx_drain", daemon=True
        )
        self._thread.start()
        self._log.info("RadioTxManager drain thread started.")

    def stop(self) -> None:
        """Signal the drain thread to stop and wait for it."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._log.info("RadioTxManager stopped.")

    # Public transmit API (called from ROS callbacks / BridgeNode)

    def enqueue(self, bridge_id: str, raw_packet: bytes) -> None:
        """
        Add raw_packet to the named bridge's queue.

        raw_packet should be the bytes produced by pack_radio_packet().
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

    def ack_received(self, bridge_id: str, seq: int) -> None:
        """
        Notify the manager that the receiver ACKed sequence number seq
        for the given bridge.  Call this from the radio receive callback.
        """
        q = self._queues.get(bridge_id)
        if q:
            q.acknowledge(seq)

    # Drain loop

    def _drain_loop(self) -> None:
        """
        Background thread: iterate over priority tiers and send packets.

        High-priority tiers are fully drained before lower tiers are touched.
        Within a tier, queues are round-robined.
        """
        while self._running:
            self._drain_once()
            time.sleep(self._drain_interval)

    def _drain_once(self) -> None:
        """One full scheduling pass across all bridges."""
        # TODO understand this logic. This is a bit complex. 
        # Group queues into tiers (sorted ascending = highest priority first)
        tiers: dict[int, list[TxQueue]] = {}
        for q in self._queues.values():
            tiers.setdefault(q.priority, []).append(q)

        for priority in sorted(tiers.keys()):
            tier_queues = tiers[priority]
            # Drain this tier fully before moving to the next.
            # "Fully" means: keep going until no queue in the tier has anything ready.
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
                        # Send failed at hardware level; leave in queue to retry
                        self._log.warning(
                            f"[{q.bridge_id}] hardware send failed for seq={pkt.seq}"
                        )

            # If the current tier still has anything pending (reliable awaiting ACK),
            # do NOT move to lower-priority tiers yet.
            if any(q.has_pending() for q in tier_queues):
                break

    # Receive path

    def _on_receive(self, bridge_id: str, seq: int, payload: bytes) -> None:
        """
        Called by XBeeRadioDevice for every inbound packet.

        For reliable bridges, automatically sends an ACK back.
        The BridgeNode's receive_radio_packet() is responsible for
        reconstructing the ROS message; this layer only handles ACKs.

        # TODO change bridge id to 1 byte unsigned int
        """
        q = self._queues.get(bridge_id)
        if q and q.reliability == "reliable":

            # TODO: transmit an ACK packet back to the sender.
            # ACK frame format can be minimal: just seq + bridge_id.
            # For now log it so the path is traceable.
            self._log.debug(f"[{bridge_id}] would ACK seq={seq}")

        # TODO do the unpacking of the data!


    # Diagnostics

    def log_stats(self) -> None:
        """Dump per-bridge queue statistics to the logger."""
        for bridge_id, q in self._queues.items():
            self._log.info(
                f"  [{bridge_id}] priority={q.priority} reliability={q.reliability} "
                f"enqueued={q.stat_enqueued} dropped={q.stat_dropped} "
                f"sent={q.stat_sent} retried={q.stat_retried} failed={q.stat_failed}"
            )