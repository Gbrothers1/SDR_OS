#!/usr/bin/env python3
import glob
import os

def list_iio_devices():
    devices = {}
    for dev in glob.glob("/sys/bus/iio/devices/iio:device*"):
        name_file = os.path.join(dev, "name")
        try:
            with open(name_file, "r") as f:
                device_name = f.read().strip()
                devices[dev] = device_name
        except Exception:
            devices[dev] = "Unknown"
    return devices

if __name__ == "__main__":
    devices = list_iio_devices()
    print("Detected IIO Devices:")
    for dev, name in devices.items():
        print(f"{dev}: {name}")
