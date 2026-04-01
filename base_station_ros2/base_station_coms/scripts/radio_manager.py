#!/usr/bin/env python3

"""
radio_manager.py

Thin hardware wrapper around digi.xbee.  Owns the serial port and nothing else.
All queue management and scheduling logic lives in bridge_core.TxManager.
"""

import logging
import struct
from typing import Callable, Optional

from bridge_core import strip_seq

# digi.xbee imports — guarded so the module can be imported in unit tests
# without hardware present.
try:
    from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
    from digi.xbee.exception import TransmitException
    _XBEE_AVAILABLE = True
except ImportError:
    _XBEE_AVAILABLE = False


# TODO handle the Sequence Numbers here if the lower level does not
# TODO handle checksum in here unless the lower level already does.

class XBeeRadioDevice:
    """
    Thin, policy-free wrapper around digi.xbee.

    Responsibilities
    ----------------
    - Open / close the serial port.
    - send(address, data) — unicast if address given, broadcast otherwise.
    - set_receive_callback(fn) — fn(bridge_id: int, seq: int, payload: bytes)

    Everything else (queuing, priority, retries) lives in TxManager.
    """

    def __init__(self, port: str, baud: int, logger=None):
        self._port   = port
        self._baud   = baud
        self._log    = logger or logging.getLogger(__name__)
        self._device = None
        self._rx_callback: Optional[Callable] = None

        if not _XBEE_AVAILABLE:
            self._log.warning(
                "digi.xbee not installed — XBeeRadioDevice running in stub mode."
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

    def set_receive_callback(self, fn: Callable[[int, int, bytes], None]) -> None:
        """
        Register a callback invoked on every received packet.

        fn(bridge_id: int, seq: int, payload: bytes)
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
        """Internal digi.xbee callback — strip seq and forward."""
        try:
            raw = xbee_message.data
            seq, payload = strip_seq(raw)
            bridge_id = struct.unpack_from("<B", payload, 0)[0]

            if self._rx_callback:
                self._rx_callback(bridge_id, seq, payload)
        except Exception as exc:
            self._log.error(f"XBee RX parse error: {exc}")
