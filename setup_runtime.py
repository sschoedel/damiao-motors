#!/usr/bin/env python3
"""Fetch the correct libdm_device runtime for this host.

Downloads from motorbridge's vendored copy of Damiao's DM_Device SDK.
Places the result under runtime/ so dmcan.py can find it automatically.
"""

import os
import platform
import sys
import urllib.request
from pathlib import Path


BASE = "https://raw.githubusercontent.com/tianrking/motorbridge/main/third_party/dm_device/v1.1.0"


def resolve() -> tuple[str, str]:
    system = platform.system()
    machine = platform.machine().lower()

    if system == "Linux":
        if machine in ("x86_64", "amd64"):
            return f"{BASE}/linux/x86_64/libdm_device.so", "libdm_device.so"
        if machine in ("aarch64", "arm64"):
            return f"{BASE}/linux/aarch64/libdm_device.so", "libdm_device.so"
        raise SystemExit(f"unsupported Linux arch: {machine}")

    if system == "Darwin":
        if machine == "arm64":
            return f"{BASE}/macos/arm64/libdm_device.dylib", "libdm_device.dylib"
        if machine == "x86_64":
            return f"{BASE}/macos/x86_64/libdm_device.dylib", "libdm_device.dylib"
        raise SystemExit(f"unsupported macOS arch: {machine}")

    raise SystemExit(f"unsupported OS: {system}")


def main() -> None:
    url, filename = resolve()
    out_dir = Path(__file__).parent / "runtime"
    out_dir.mkdir(exist_ok=True)
    dest = out_dir / filename

    if dest.exists() and "--force" not in sys.argv:
        print(f"already present: {dest}  (pass --force to redownload)")
        return

    print(f"downloading {url}")
    urllib.request.urlretrieve(url, dest)
    size = os.path.getsize(dest)
    print(f"saved to {dest}  ({size/1024:.1f} KB)")


if __name__ == "__main__":
    main()
