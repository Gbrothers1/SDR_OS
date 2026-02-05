#!/usr/bin/env python3
"""
Script to list and display IIO sensor values with device descriptions.

Detected IIO Devices on your system:
  /sys/bus/iio/devices/iio:device5: incli_3d
  /sys/bus/iio/devices/iio:device3: dev_rotation
  /sys/bus/iio/devices/iio:device1: als
  /sys/bus/iio/devices/iio:device4: magn_3d
  /sys/bus/iio/devices/iio:device2: accel_3d
  /sys/bus/iio/devices/iio:device0: gyro_3d

Descriptions for devices (hardcoded):
  gyro_3d   : Gyroscope: Measures angular velocity in 3 axes to track rotation.
  accel_3d  : Accelerometer: Measures acceleration (including gravity) in 3 axes.
  magn_3d   : Magnetometer: Measures magnetic field strength in 3 axes, useful as a digital compass.
  als       : Ambient Light Sensor: Measures ambient light intensity.
  incli_3d  : Inclinometer: Measures tilt (pitch and roll) relative to gravity.
  dev_rotation: Rotation Sensor: Provides overall rotation data, potentially fusing inputs from multiple sensors.
"""

import glob
import os
import re

# Hardcoded descriptions for each known device type.
DEVICE_DESCRIPTIONS = {
    "gyro_3d": "Gyroscope: Measures angular velocity in 3 axes to track rotation.",
    "accel_3d": "Accelerometer: Measures acceleration (including gravity) in 3 axes.",
    "magn_3d": "Magnetometer: Measures magnetic field strength in 3 axes, useful as a digital compass.",
    "als": "Ambient Light Sensor: Measures ambient light intensity for auto brightness and environmental sensing.",
    "incli_3d": "Inclinometer: Measures tilt (pitch and roll) relative to gravity.",
    "dev_rotation": "Rotation Sensor: Provides overall rotation data, potentially fusing inputs from multiple sensors."
}

def get_device_name(device_dir):
    name_file = os.path.join(device_dir, "name")
    if os.path.exists(name_file):
        try:
            return open(name_file).read().strip()
        except Exception as e:
            return f"Error reading name: {e}"
    return "Unknown"

def read_value(file_path):
    try:
        return open(file_path).read().strip()
    except Exception:
        return None

def get_scale_for_channel(device_dir, raw_file):
    # First attempt: replace "_raw" with "_scale"
    candidate = raw_file.replace("_raw", "_scale")
    scale_path = os.path.join(device_dir, candidate)
    if os.path.exists(scale_path):
        return read_value(scale_path)
    # Second attempt: for example, "in_accel_x_raw" -> "in_accel_scale"
    m = re.match(r"(in_[a-zA-Z]+)_\w+_raw", raw_file)
    if m:
        candidate = m.group(1) + "_scale"
        scale_path = os.path.join(device_dir, candidate)
        if os.path.exists(scale_path):
            return read_value(scale_path)
    return None

def process_device(device_dir):
    device_name = get_device_name(device_dir)
    description = DEVICE_DESCRIPTIONS.get(device_name, "No description available for this device.")
    print(f"Device: {device_dir} ({device_name})")
    print(f"  Description: {description}")
    
    # Iterate over all files ending with _raw and display their values
    for filename in sorted(os.listdir(device_dir)):
        if filename.endswith("_raw"):
            file_path = os.path.join(device_dir, filename)
            raw_value_str = read_value(file_path)
            if raw_value_str is None:
                continue

            # Try to convert raw value to float
            try:
                raw_value = float(raw_value_str)
                can_scale = True
            except ValueError:
                raw_value = raw_value_str  # keep as string if conversion fails
                can_scale = False

            scale_str = get_scale_for_channel(device_dir, filename)
            if can_scale and scale_str is not None:
                try:
                    scale = float(scale_str)
                    scaled_value = raw_value * scale
                except ValueError:
                    scaled_value = "Error: invalid scale"
            else:
                scaled_value = None

            if scaled_value is not None:
                print(f"  {filename}: raw = {raw_value}, scaled = {scaled_value}")
            else:
                print(f"  {filename}: raw = {raw_value}")
    print()  # Blank line between devices

def main():
    device_paths = glob.glob("/sys/bus/iio/devices/iio:device*")
    if not device_paths:
        print("No IIO devices found.")
        return

    for device in sorted(device_paths):
        process_device(device)

if __name__ == "__main__":
    main()
