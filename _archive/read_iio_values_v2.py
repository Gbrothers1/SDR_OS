#!/usr/bin/env python3
"""
Live IIO Sensor Data Monitor with Dynamic Terminal Sizing and Missing Data Handling

Detected IIO Devices on your system:
  /sys/bus/iio/devices/iio:device5: incli_3d
  /sys/bus/iio/devices/iio:device3: dev_rotation
  /sys/bus/iio/devices/iio:device1: als
  /sys/bus/iio/devices/iio:device4: magn_3d
  /sys/bus/iio/devices/iio:device2: accel_3d
  /sys/bus/iio/devices/iio:device0: gyro_3d

Hardcoded device descriptions:
  gyro_3d     : Gyroscope - Measures angular velocity in 3 axes.
  accel_3d    : Accelerometer - Measures acceleration (including gravity) in 3 axes.
  magn_3d     : Magnetometer - Measures magnetic field strength in 3 axes.
  als         : Ambient Light Sensor - Measures ambient light intensity.
  incli_3d    : Inclinometer - Measures tilt (pitch and roll) relative to gravity.
  dev_rotation: Rotation Sensor - Provides overall rotation data, fusing sensor inputs.

Press 'q' to quit.
"""

import os
import glob
import re
import time
import curses

# Hardcoded descriptions for known device types.
DEVICE_DESCRIPTIONS = {
    "gyro_3d": "Gyroscope - Measures angular velocity in 3 axes.",
    "accel_3d": "Accelerometer - Measures acceleration (including gravity) in 3 axes.",
    "magn_3d": "Magnetometer - Measures magnetic field strength in 3 axes.",
    "als": "Ambient Light Sensor - Measures ambient light intensity.",
    "incli_3d": "Inclinometer - Measures tilt (pitch and roll) relative to gravity.",
    "dev_rotation": "Rotation Sensor - Provides overall rotation data, fusing sensor inputs."
}

def safe_addstr(win, y, x, text, attr=0):
    """
    A helper function to safely add text to a curses window.
    It will truncate the text to fit within the window width.
    """
    max_y, max_x = win.getmaxyx()
    if y < 0 or y >= max_y:
        return
    if x >= max_x:
        return
    truncated_text = text[: max_x - x]
    try:
        win.addstr(y, x, truncated_text, attr)
    except curses.error:
        pass

def get_device_name(device_dir):
    name_file = os.path.join(device_dir, "name")
    if os.path.exists(name_file):
        try:
            return open(name_file).read().strip()
        except Exception as e:
            return f"Error: {e}"
    return "Unknown"

def read_value(file_path):
    try:
        return open(file_path).read().strip()
    except Exception:
        return None

def get_scale_for_channel(device_dir, raw_file):
    candidate = raw_file.replace("_raw", "_scale")
    scale_path = os.path.join(device_dir, candidate)
    if os.path.exists(scale_path):
        return read_value(scale_path)
    # For names like in_accel_x_raw, try in_accel_scale
    m = re.match(r"(in_[a-zA-Z]+)_\w+_raw", raw_file)
    if m:
        candidate = m.group(1) + "_scale"
        scale_path = os.path.join(device_dir, candidate)
        if os.path.exists(scale_path):
            return read_value(scale_path)
    return None

def get_device_data(device_dir):
    """
    Reads the device name, looks up its description, and gets sensor channel values.
    If a channel is empty/missing, it will be flagged as 'Missing'.
    """
    name = get_device_name(device_dir)
    description = DEVICE_DESCRIPTIONS.get(name, "No description available for this device.")
    values = {}
    for filename in sorted(os.listdir(device_dir)):
        if filename.endswith("_raw"):
            file_path = os.path.join(device_dir, filename)
            raw_value_str = read_value(file_path)
            # If the raw value is missing or empty, mark it as such
            if raw_value_str is None or raw_value_str == "":
                values[filename] = {"raw": "Missing"}
                continue
            try:
                raw_value = float(raw_value_str)
                can_scale = True
            except ValueError:
                raw_value = raw_value_str
                can_scale = False
            scale_str = get_scale_for_channel(device_dir, filename)
            if can_scale and scale_str is not None:
                try:
                    scale = float(scale_str)
                    scaled_value = raw_value * scale
                except ValueError:
                    scaled_value = "Error: invalid scale"
                values[filename] = {"raw": raw_value, "scaled": scaled_value}
            else:
                values[filename] = {"raw": raw_value}
    return {
        "path": device_dir,
        "name": name,
        "description": description,
        "values": values
    }

def draw_interface(stdscr):
    # Curses setup
    curses.curs_set(0)  # Hide the cursor.
    stdscr.nodelay(True)
    stdscr.timeout(1000)  # Refresh every 1 second.

    while True:
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()
        header = " Live IIO Sensor Data Monitor (press 'q' to quit) "
        safe_addstr(stdscr, 0, 0, header.ljust(max_x), curses.A_REVERSE)
        line = 2  # Starting line for content.

        device_dirs = sorted(glob.glob("/sys/bus/iio/devices/iio:device*"))
        if not device_dirs:
            safe_addstr(stdscr, line, 0, "No IIO devices found.")
        else:
            for device in device_dirs:
                data = get_device_data(device)
                title = f"Device: {data['path']} ({data['name']})"
                safe_addstr(stdscr, line, 0, title, curses.A_BOLD | curses.A_UNDERLINE)
                line += 1
                safe_addstr(stdscr, line, 2, f"Description: {data['description']}")
                line += 1
                # If no sensor channels are present
                if not data["values"]:
                    safe_addstr(stdscr, line, 4, "No sensor data available.")
                    line += 1
                else:
                    for channel, val in data["values"].items():
                        if "scaled" in val:
                            channel_line = f"{channel}: raw = {val['raw']}, scaled = {val['scaled']}"
                        else:
                            channel_line = f"{channel}: raw = {val['raw']}"
                        safe_addstr(stdscr, line, 4, channel_line)
                        line += 1
                line += 1  # Blank line after each device.
                if line >= max_y - 2:
                    safe_addstr(stdscr, line, 0, "-- Data truncated. Resize your window for full view. --")
                    break
        
        stdscr.refresh()
        try:
            ch = stdscr.getch()
            if ch == ord('q'):
                break
        except Exception:
            pass

def main():
    curses.wrapper(draw_interface)

if __name__ == "__main__":
    main()
