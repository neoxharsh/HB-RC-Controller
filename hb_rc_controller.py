"""
HB RC BLE Controller
====================

Reusable, UI-independent BLE protocol/controller layer for the HB Toys
HB-R4304 RC car.

Install:
    python -m pip install bleak

Known BLE configuration:
    Address: E0:97:00:00:00:00
    Service: 0000ae30-0000-1000-8000-00805f9b34fb
    Write characteristic: 0000ae01-0000-1000-8000-00805f9b34fb
    Write mode: Write Without Response

Observed 10-byte packet:
    [0] 0xAA       header
    [1] 0x55       header
    [2] throttle   centred at 0x80
    [3] speed      observed range 1..127
    [4] steering   centred at 0x80
    [5] 0x40       observed constant
    [6] light      observed alternating values 0x01 / 0x00
    [7] 0x00       observed constant
    [8] 0x00       observed constant
    [9] checksum   sum(bytes 0..8) & 0xFF

Observed behaviour:
* Continuous packet transmission at approximately 23 ms reproduced the
  working light-control behaviour.
* Lights cycle in this order:
      CONSTANT_ON -> RAPID_BLINK -> OFF -> CONSTANT_ON
  The class remembers the assumed current state and advances through
  intermediate states when set_light_mode() is called.
* The car's current light state is not confirmed to be readable. The initial
  state is therefore an assumption, not a hardware-reported value.
* Throttle/steering byte mapping is inferred and should be tested cautiously.

This class does not implement acceleration curves. Those belong in the UI or
application layer, allowing different frontends to provide their own motion
profiles without changing the BLE protocol layer.

Public synchronous API (safe to call from a Tkinter/UI thread):
    controller.connect()
    controller.set_drive(throttle=0, steering=0, speed=40)
    controller.stop_drive()
    controller.set_light_mode(LightMode.OFF)
    controller.next_light_mode()
    controller.disconnect()
    controller.close()

Safety:
Always test with the vehicle secured, wheels raised, and hands clear.
stop_drive() sends neutral values; it is not a hardware emergency stop.
"""

from __future__ import annotations

import asyncio
import threading
from enum import IntEnum
from typing import Optional

from bleak import BleakClient


class LightMode(IntEnum):
    CONSTANT_ON = 0
    RAPID_BLINK = 1
    OFF = 2


LIGHT_MODE_NAMES = {
    LightMode.CONSTANT_ON: "Constant ON",
    LightMode.RAPID_BLINK: "Rapid blinking",
    LightMode.OFF: "OFF",
}


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, int(value)))


def axis_to_byte(value: int) -> int:
    value = _clamp(value, -100, 100)
    if value == 0:
        return 0x80
    if value > 0:
        return 0x80 + round(value * 127 / 100)
    return 0x80 - round(abs(value) * 127 / 100)


def build_packet(throttle: int, speed: int, steering: int, light_field: int) -> bytes:
    packet = bytearray((
        0xAA, 0x55,
        axis_to_byte(throttle),
        _clamp(speed, 1, 127),
        axis_to_byte(steering),
        0x40,
        0x01 if light_field else 0x00,
        0x00,
        0x00,
        0x00,
    ))
    packet[9] = sum(packet[:9]) & 0xFF
    return bytes(packet)


class HBRCController:
    """Reusable synchronous interface to the HB RC car."""

    SERVICE_UUID = "0000ae30-0000-1000-8000-00805f9b34fb"
    CHARACTERISTIC_UUID = "0000ae01-0000-1000-8000-00805f9b34fb"

    def __init__(
        self,
        address: str = "E0:97:00:00:00:00",
        send_interval: float = 0.023,
        initial_light_mode: LightMode = LightMode.CONSTANT_ON,
        light_transition_delay: float = 0.18,
    ):
        self.address = address
        self.send_interval = max(0.01, float(send_interval))
        self.light_transition_delay = max(0.05, float(light_transition_delay))

        self.throttle = 0
        self.steering = 0
        self.speed = 1
        self.light_mode = LightMode(initial_light_mode)
        self._light_field = 0x01

        self._client: Optional[BleakClient] = None
        self._characteristic = None
        self._sender_task = None
        self._connected = False
        self._running = False

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="HBRC-BLE-Loop", daemon=True
        )
        self._thread.start()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def current_light_mode(self) -> LightMode:
        return self.light_mode

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()
        pending = asyncio.all_tasks(self._loop)
        for task in pending:
            task.cancel()
        if pending:
            self._loop.run_until_complete(
                asyncio.gather(*pending, return_exceptions=True)
            )
        self._loop.close()

    def _call(self, coroutine, timeout: float = 10.0):
        if not self._loop.is_running():
            raise RuntimeError("Controller event loop is not running")
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        return future.result(timeout=timeout)

    async def _connect_async(self):
        if self._connected:
            return
        self._client = BleakClient(self.address)
        await self._client.connect()
        if not self._client.is_connected:
            raise RuntimeError("BLE connection was not established")

        service = self._client.services.get_service(self.SERVICE_UUID)
        if service is None:
            raise RuntimeError("AE30 service was not found")
        self._characteristic = service.get_characteristic(self.CHARACTERISTIC_UUID)
        if self._characteristic is None:
            raise RuntimeError("AE30/AE01 write characteristic was not found")

        self.throttle = 0
        self.steering = 0
        self._connected = True
        self._running = True
        self._sender_task = asyncio.create_task(self._send_loop())

    def connect(self):
        try:
            self._call(self._connect_async(), timeout=20.0)
        except Exception:
            self._connected = False
            self._running = False
            raise

    async def _send_loop(self):
        loop = asyncio.get_running_loop()
        next_send = loop.time()
        try:
            while self._running and self._client and self._client.is_connected:
                packet = build_packet(
                    self.throttle, self.speed, self.steering, self._light_field
                )
                await self._client.write_gatt_char(
                    self._characteristic, packet, response=False
                )
                next_send += self.send_interval
                delay = next_send - loop.time()
                if delay > 0:
                    await asyncio.sleep(delay)
                else:
                    next_send = loop.time()
        except asyncio.CancelledError:
            raise
        except Exception:
            self._connected = False
            self._running = False
            raise

    async def _set_drive_async(self, throttle=None, steering=None, speed=None):
        if throttle is not None:
            self.throttle = _clamp(throttle, -100, 100)
        if steering is not None:
            self.steering = _clamp(steering, -100, 100)
        if speed is not None:
            self.speed = _clamp(speed, 1, 127)

    def set_drive(self, throttle=None, steering=None, speed=None):
        if not self._connected:
            return
        self._call(self._set_drive_async(throttle, steering, speed))

    def stop_drive(self):
        self.set_drive(throttle=0, steering=0)

    async def _advance_light_once(self):
        self.light_mode = LightMode((int(self.light_mode) + 1) % 3)
        self._light_field = 0x00 if self._light_field == 0x01 else 0x01
        await asyncio.sleep(self.light_transition_delay)

    async def _set_light_mode_async(self, target: LightMode):
        target = LightMode(target)
        while self.light_mode != target:
            await self._advance_light_once()

    def set_light_mode(self, target: LightMode):
        if not self._connected:
            raise RuntimeError("Connect to the car before changing lights")
        self._call(self._set_light_mode_async(LightMode(target)), timeout=10.0)

    def next_light_mode(self):
        if not self._connected:
            raise RuntimeError("Connect to the car before changing lights")
        self._call(self._advance_light_once())

    async def _disconnect_async(self):
        self._running = False
        if self._sender_task is not None:
            self._sender_task.cancel()
            try:
                await self._sender_task
            except (asyncio.CancelledError, Exception):
                pass
            self._sender_task = None

        if self._client is not None:
            try:
                if self._client.is_connected and self._characteristic is not None:
                    neutral = build_packet(0, self.speed, 0, self._light_field)
                    await self._client.write_gatt_char(
                        self._characteristic, neutral, response=False
                    )
            except Exception:
                pass
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            finally:
                self._client = None
                self._characteristic = None

        self._connected = False
        self.throttle = 0
        self.steering = 0

    def disconnect(self):
        if self._loop.is_running():
            self._call(self._disconnect_async(), timeout=8.0)

    def close(self):
        try:
            self.disconnect()
        finally:
            if self._loop.is_running():
                self._loop.call_soon_threadsafe(self._loop.stop)
                self._thread.join(timeout=3.0)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
