#!/usr/bin/env python3

"""
comms_device.py

Abstract base class for communication device wrappers.

Any transport (XBee radio, serial, UDP, WiFi, etc.) can be dropped into
TxManager by subclassing CommsDevice and implementing three abstract methods:
    open()      — open the hardware connection
    close()     — close the hardware connection
    _send_raw() — write framed bytes to the hardware

The base class owns the receive-callback mechanism and runs all inbound bytes
through a filtering pipeline before forwarding to the registered callback:

    _verify_checksum(raw)         → discard on mismatch
    _strip_framing(raw)           → extract (dest_addr, inner_packet)
    _check_destination(dest_addr) → discard if not addressed to this device
    strip_seq(inner_packet)       → extract (seq, payload)
    extract bridge_id from payload[0]
    _rx_callback(bridge_id, seq, payload)

Default implementations of all four filter hooks return True / pass-through,
which is correct for devices (like XBee) where the hardware already handles
checksums and destination routing.  Override only the hooks your transport
actually needs.

TX pipeline:

    send(address, packet)
        _add_framing(address, packet) → framed   [optional, default pass-through]
        _send_raw(address, framed)   → bool
"""

import logging
import struct
from abc import ABC, abstractmethod
from typing import Callable, Optional

from bridge_core import strip_seq


class CommsDevice(ABC):
    """
    Abstract base class for all communication device wrappers.

    Subclass and implement open(), close(), and _send_raw().
    Override the filtering hooks only when the hardware does not already
    provide checksum verification and destination-address routing.
    """

    def __init__(
        self,
        logger=None,
        my_address: Optional[str] = None,
    ):
        """
        Parameters
        ----------
        logger      : ROS 2 logger or standard logging.Logger.  Falls back to
                      the module logger if not provided.
        my_address  : This device's address on the medium.  Used by
                      _check_destination() in subclasses that share a broadcast
                      medium and must filter packets in software.
                      Leave None for devices where the hardware handles routing.
        """
        self._log        = logger or logging.getLogger(__name__)
        self._rx_callback: Optional[Callable[[int, int, bytes], None]] = None
        self.my_address  = my_address

    # ------------------------------------------------------------------
    # Abstract — must implement
    # ------------------------------------------------------------------

    @abstractmethod
    def open(self) -> bool:
        """Open the hardware connection.  Return True on success."""

    @abstractmethod
    def close(self) -> None:
        """Close the hardware connection."""

    @abstractmethod
    def _send_raw(self, address: Optional[str], data: bytes) -> bool:
        """
        Write data to the hardware, addressed to address (or broadcast if None).
        Return True on success.
        """

    # ------------------------------------------------------------------
    # Filtering hooks — override when hardware doesn't handle these
    # ------------------------------------------------------------------

    def _add_framing(self, address: Optional[str], packet: bytes) -> bytes:
        """
        Optionally wrap packet with device-specific framing before TX
        (e.g. destination address header, CRC).

        Default: pass-through (hardware handles it).
        """
        return packet

    def _strip_framing(self, raw: bytes) -> tuple[Optional[str], bytes]:
        """
        Remove device-specific framing from an inbound byte string.
        Return (dest_addr, inner_packet).

        dest_addr is forwarded to _check_destination().  Return None when
        the hardware already filtered by destination and no check is needed.

        Default: pass-through (hardware handles it).
        """
        return None, raw

    def _verify_checksum(self, raw: bytes) -> bool:
        """
        Return False to discard a packet whose checksum does not match.

        Default: True (hardware-verified, no software check needed).
        """
        return True

    def _check_destination(self, dest_addr: Optional[str]) -> bool:
        """
        Return False to discard a packet not addressed to this device.

        Compare dest_addr (extracted by _strip_framing) against self.my_address.

        Default: True (hardware already routed correctly, no filter needed).
        """
        return True
    
    # TODO method to add the checksum

    # ------------------------------------------------------------------
    # Concrete pipeline — not intended to be overridden
    # ------------------------------------------------------------------

    def send(self, address: Optional[str], packet: bytes) -> bool:
        """
        Full TX pipeline:
            1. _add_framing(address, packet) → framed bytes
            2. _send_raw(address, framed)    → bool
        """
        framed = self._add_framing(address, packet)
        return self._send_raw(address, framed)

    def set_receive_callback(self, fn: Callable[[int, int, bytes], None]) -> None:
        """
        Register the callback invoked for every valid inbound packet.

            fn(bridge_id: int, seq: int, payload: bytes)
        """
        self._rx_callback = fn

    def _process_received(self, raw: bytes) -> None:
        """
        Entry point for bytes arriving from the hardware.  Runs the full
        receive pipeline and, if the packet passes all filters, invokes the
        registered callback.

        Call this from your hardware receive callback, e.g.:
            def _hw_callback(self, message):
                self._process_received(message.data)
        """
        if not self._verify_checksum(raw):
            self._log.debug("Packet discarded: checksum mismatch")
            return

        dest_addr, packet = self._strip_framing(raw)

        if not self._check_destination(dest_addr):
            self._log.debug(f"Packet discarded: destination {dest_addr!r} not for us")
            return

        try:
            seq, inner = strip_seq(packet)
            bridge_id  = struct.unpack_from("<B", inner, 0)[0]
            if self._rx_callback:
                self._rx_callback(bridge_id, seq, inner)
        except Exception as exc:
            self._log.error(f"RX parse error: {exc}")
