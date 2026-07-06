"""ctypes bindings for Damiao's libdm_device (DM_Device SDK).

Only covers what we need to open the USB-CANFD adapter, force it into
CAN2.0 @ 1 Mbps, enable channel 0, and send/receive frames.

C API is defined in third_party/dm_device/v1.1.0/dmcan.h in the motorbridge
repo. Sizes are hand-derived; if this stops working after an SDK update,
verify struct layouts against the current dmcan.h.
"""

from __future__ import annotations

import ctypes as C
import os
import platform
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path


def _default_lib_path() -> Path:
    root = Path(__file__).parent / "runtime"
    system = platform.system()
    if system == "Darwin":
        return root / "libdm_device.dylib"
    if system == "Linux":
        return root / "libdm_device.so"
    if system == "Windows":
        return root / "dm_device.dll"
    raise RuntimeError(f"unsupported platform: {system}")


_DEFAULT_LIB = _default_lib_path()


class DmcanError(RuntimeError):
    pass


# ---- device type enum ----
USB2CANFD      = 0
USB2CANFD_DUAL = 1
LINKX4C        = 2


# ---- struct definitions ----
# usb_rx_frame_head_t is packed. It has bitfields — we treat the two bitfield
# words as opaque uint32/uint8 and mask manually. That's more portable across
# compilers than trusting ctypes bitfields.
class _RxHead(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("_id_flags", C.c_uint32),   # can_id:29, esi:1, ext:1, rtr:1
        ("time_stamp", C.c_uint64),
        ("channel", C.c_uint8),
        ("_type_flags", C.c_uint8),  # canfd:1, dir:1, brs:1, ack:1, dlc:4
        ("reserved", C.c_uint16),
    ]

    @property
    def can_id(self) -> int:
        return self._id_flags & 0x1FFFFFFF

    @property
    def ext(self) -> bool:
        return bool((self._id_flags >> 30) & 1)

    @property
    def rtr(self) -> bool:
        return bool((self._id_flags >> 31) & 1)

    @property
    def canfd(self) -> bool:
        return bool(self._type_flags & 1)

    @property
    def dir_rx(self) -> bool:
        return bool((self._type_flags >> 1) & 1)

    @property
    def dlc(self) -> int:
        return (self._type_flags >> 4) & 0x0F


class _UsbRxFrame(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("head", _RxHead),
        ("payload", C.c_uint8 * 64),
    ]


class _ChannelBaudInfo(C.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel", C.c_uint8),
        ("canfd", C.c_bool),
        ("can_baudrate", C.c_uint32),
        ("canfd_baudrate", C.c_uint32),
        ("can_sp", C.c_float),
        ("canfd_sp", C.c_float),
    ]


_RecvCallbackT = C.CFUNCTYPE(None, C.c_void_p, C.POINTER(_UsbRxFrame))
_SentCallbackT = C.CFUNCTYPE(None, C.c_void_p, C.POINTER(_UsbRxFrame))
_ErrCallbackT  = C.CFUNCTYPE(None, C.c_void_p, C.POINTER(_UsbRxFrame))


@dataclass
class RxFrame:
    can_id: int
    ext: bool
    rtr: bool
    canfd: bool
    dlc: int
    time_stamp: int
    channel: int
    data: bytes


class Adapter:
    """Owns the ctypes lib handle, context, device handle, and recv callback."""

    def __init__(self, lib_path: os.PathLike | str | None = None):
        path = str(lib_path or _DEFAULT_LIB)
        if not os.path.exists(path):
            raise DmcanError(f"libdm_device dylib not found at {path}")
        self._lib = C.CDLL(path)
        self._bind()
        self._ctx = C.c_void_p()
        self._dev = C.c_void_p()
        self._rx_queue: queue.Queue[RxFrame] = queue.Queue()
        self._lock = threading.Lock()
        self._shutting_down = False

        # Keep a strong ref to the C callback so it isn't GC'd while the C SDK
        # holds the pointer — losing this ref would crash the SDK's callback.
        self._recv_cb_c = _RecvCallbackT(self._recv_trampoline)
        self._sent_cb_c = _SentCallbackT(lambda *_a: None)
        self._err_cb_c  = _ErrCallbackT(lambda *_a: None)

    def _bind(self):
        L = self._lib
        L.dmcan_context_create.argtypes  = [C.POINTER(C.c_void_p)]
        L.dmcan_context_create.restype   = None
        L.dmcan_context_destroy.argtypes = [C.c_void_p]
        L.dmcan_context_destroy.restype  = None
        L.dmcan_find_devices.argtypes    = [C.c_void_p]
        L.dmcan_find_devices.restype     = C.c_int
        L.dmcan_find_devices_with_type.argtypes = [C.c_void_p, C.c_int]
        L.dmcan_find_devices_with_type.restype  = C.c_int
        L.dmcan_show_all_devices.argtypes = [C.c_void_p]
        L.dmcan_show_all_devices.restype  = None
        L.dmcan_device_get.argtypes = [C.c_void_p, C.POINTER(C.c_void_p), C.c_int]
        L.dmcan_device_get.restype  = C.c_bool
        L.dmcan_device_open.argtypes  = [C.c_void_p]
        L.dmcan_device_open.restype   = C.c_bool
        L.dmcan_device_close.argtypes = [C.c_void_p]
        L.dmcan_device_close.restype  = None
        L.dmcan_device_enable_channel.argtypes  = [C.c_void_p, C.c_uint8]
        L.dmcan_device_enable_channel.restype   = C.c_bool
        L.dmcan_device_disable_channel.argtypes = [C.c_void_p, C.c_uint8]
        L.dmcan_device_disable_channel.restype  = C.c_bool
        L.dmcan_device_get_channel_baudrate.argtypes = [
            C.c_void_p, C.c_uint8, C.POINTER(_ChannelBaudInfo)
        ]
        L.dmcan_device_get_channel_baudrate.restype = C.c_bool
        L.dmcan_device_set_channel_baudrate.argtypes = [
            C.c_void_p, C.c_uint8, _ChannelBaudInfo
        ]
        L.dmcan_device_set_channel_baudrate.restype = C.c_bool
        L.dmcan_device_hook_recv_callback.argtypes = [C.c_void_p, _RecvCallbackT]
        L.dmcan_device_hook_recv_callback.restype  = None
        L.dmcan_device_hook_sent_callback.argtypes = [C.c_void_p, _SentCallbackT]
        L.dmcan_device_hook_sent_callback.restype  = None
        L.dmcan_device_hook_err_callback.argtypes  = [C.c_void_p, _ErrCallbackT]
        L.dmcan_device_hook_err_callback.restype   = None
        L.dmcan_device_send_can.argtypes = [
            C.c_void_p, C.c_uint8, C.c_uint32,
            C.c_bool, C.c_bool, C.c_bool, C.c_bool,
            C.c_uint8, C.POINTER(C.c_uint8),
        ]
        L.dmcan_device_send_can.restype = C.c_bool

    def _recv_trampoline(self, _handle, frame_ptr):
        # SDK invokes this from its own thread. Keep it minimal + exception-safe.
        # During shutdown the SDK can still fire cancelled-transfer callbacks
        # while its own mutex is being torn down; refuse to touch anything then.
        if self._shutting_down:
            return
        try:
            f = frame_ptr.contents
            h = f.head
            dlc = h.dlc
            payload = bytes(f.payload[:dlc])
            self._rx_queue.put_nowait(RxFrame(
                can_id=h.can_id, ext=h.ext, rtr=h.rtr, canfd=h.canfd,
                dlc=dlc, time_stamp=h.time_stamp, channel=h.channel,
                data=payload,
            ))
        except Exception:
            pass  # never let exceptions escape into the C SDK

    # ---- lifecycle ----
    def open(self, device_type: int = USB2CANFD, index: int = 0) -> None:
        self._lib.dmcan_context_create(C.byref(self._ctx))
        if not self._ctx:
            raise DmcanError("dmcan_context_create returned NULL")

        n = self._lib.dmcan_find_devices_with_type(self._ctx, device_type)
        if n <= 0:
            raise DmcanError(f"no devices of type {device_type} found (n={n})")
        if index >= n:
            raise DmcanError(f"device index {index} out of range (found {n})")

        if not self._lib.dmcan_device_get(self._ctx, C.byref(self._dev), index):
            raise DmcanError("dmcan_device_get failed")
        if not self._lib.dmcan_device_open(self._dev):
            raise DmcanError("dmcan_device_open failed")

        self._lib.dmcan_device_hook_recv_callback(self._dev, self._recv_cb_c)
        self._lib.dmcan_device_hook_sent_callback(self._dev, self._sent_cb_c)
        self._lib.dmcan_device_hook_err_callback(self._dev, self._err_cb_c)

    def close(self) -> None:
        # Signal the trampoline to no-op before we start tearing down. The SDK
        # keeps invoking recv callbacks while cancelled transfers unwind, and
        # its own callback-dispatch mutex is being destroyed concurrently — on
        # Linux this reliably triggers a pthread mutex assertion abort unless
        # we quiesce first.
        self._shutting_down = True
        if self._dev:
            try:
                self._lib.dmcan_device_disable_channel(self._dev, 0)
            except Exception:
                pass
            time.sleep(0.5)
            try:
                self._lib.dmcan_device_close(self._dev)
            except Exception:
                pass
            self._dev = C.c_void_p()
        if self._ctx:
            try:
                self._lib.dmcan_context_destroy(self._ctx)
            except Exception:
                pass
            self._ctx = C.c_void_p()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- config ----
    def set_classic_can(self, channel: int = 0, bitrate: int = 1_000_000,
                        sample_point: float = 0.8) -> None:
        info = _ChannelBaudInfo(
            channel=channel,
            canfd=False,
            can_baudrate=bitrate,
            canfd_baudrate=0,
            can_sp=sample_point,
            canfd_sp=0.0,
        )
        if not self._lib.dmcan_device_set_channel_baudrate(self._dev, channel, info):
            raise DmcanError("set_channel_baudrate failed")

    def read_baudrate(self, channel: int = 0) -> _ChannelBaudInfo:
        info = _ChannelBaudInfo()
        if not self._lib.dmcan_device_get_channel_baudrate(self._dev, channel, C.byref(info)):
            raise DmcanError("get_channel_baudrate failed")
        return info

    def enable_channel(self, channel: int = 0) -> None:
        if not self._lib.dmcan_device_enable_channel(self._dev, channel):
            raise DmcanError(f"enable_channel({channel}) failed")

    def disable_channel(self, channel: int = 0) -> None:
        self._lib.dmcan_device_disable_channel(self._dev, channel)

    # ---- send / recv ----
    def send(self, can_id: int, data: bytes, *, channel: int = 0,
             ext: bool = False, rtr: bool = False) -> None:
        dlen = len(data)
        if dlen > 8:
            raise DmcanError("classic CAN payload must be <= 8 bytes")
        buf = (C.c_uint8 * dlen)(*data)
        ok = self._lib.dmcan_device_send_can(
            self._dev, channel, can_id, False, ext, rtr, False, dlen, buf
        )
        if not ok:
            raise DmcanError("send_can returned false")

    def recv(self, timeout: float | None = 0.5) -> RxFrame | None:
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> list[RxFrame]:
        out: list[RxFrame] = []
        while True:
            try:
                out.append(self._rx_queue.get_nowait())
            except queue.Empty:
                return out
