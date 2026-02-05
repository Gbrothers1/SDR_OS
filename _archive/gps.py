#!/usr/bin/env python3
"""
Script to poll GPS data from the device:
    /dev/ttyUSB2

Device Details:
--------------------------------------------------
Device       : /dev/ttyUSB2
Name         : ttyUSB2
Description  : Dell Wireless 5808e Gobi™ 4G LTE Mobile Broadband Card
Hardware ID  : USB VID:PID=413C:81A9 LOCATION=1-2:1.3
VID          : 16700
PID          : 33193
Serial Number: None
Location     : 1-2:1.3
Manufacturer : Sierra Wireless, Incorporated
Product      : Dell Wireless 5808e Gobi™ 4G LTE Mobile Broadband Card
Interface    : None
--------------------------------------------------

This script attempts several GPS commands:
  - AT+CGNSINF      (standard on many modules)
  - AT!GSTATUS?     (often used on Sierra Wireless devices)
  - AT!GPS?         (another variant seen on some models)

Responses (both raw and parsed when applicable) are printed so you can see which command provides useful GPS info.
"""

import serial
import time
import matplotlib.pyplot as plt

# -------------------------
# Serial and AT utilities
# -------------------------
def open_serial(port='/dev/ttyUSB2', baudrate=115200, timeout=1):
    """Open the serial port."""
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        print(f"Opened serial port {port} at {baudrate} baud.")
        return ser
    except Exception as e:
        print(f"Error opening port {port}: {e}")
        return None

def send_at_command(ser, command, wait=1):
    """
    Sends an AT command and returns the full response.
    Appends \r\n to the command.
    """
    if ser is None:
        print("Serial port not open.")
        return None

    full_command = command.strip() + "\r\n"
    ser.write(full_command.encode('utf-8'))
    print(f"Sent command: {command}")
    time.sleep(wait)
    response = ser.read_all().decode('utf-8', errors='replace')
    print("Response:")
    print(response)
    return response

# -------------------------
# Parsing functions
# -------------------------
def parse_cgnsinf_response(response):
    """
    Parses the response from AT+CGNSINF.
    Expected format (if available):
      +CGNSINF: <run status>,<fix status>,<UTC>,<latitude>,<longitude>,<MSL altitude>,...
    Returns (latitude, longitude, UTC) or (None, None, None) if parsing fails.
    """
    if not response.startswith("+CGNSINF:"):
        return None, None, None
    try:
        # Remove the prefix and split on comma
        parts = response.strip().split(":", 1)[1].split(",")
        if len(parts) >= 5:
            lat_str = parts[3].strip()
            lon_str = parts[4].strip()
            utc = parts[2].strip()
            # Check if values look non-zero or non-empty
            if lat_str and lon_str and lat_str not in {"0", "0.000000"} and lon_str not in {"0", "0.000000"}:
                return float(lat_str), float(lon_str), utc
    except Exception as e:
        print("Error parsing CGNSINF response:", e)
    return None, None, None

# -------------------------
# Main polling function
# -------------------------
def poll_gps_data(ser):
    """
    Polls GPS data using multiple AT commands.
    Returns a tuple (lat, lon, utc) if valid coordinates are parsed from AT+CGNSINF.
    Also prints raw responses from other commands.
    """
    # List of fallback commands to try
    commands = ["AT+CGNSINF", "AT!GSTATUS?", "AT!GPS?"]

    for cmd in commands:
        response = send_at_command(ser, cmd, wait=1)
        # Try parsing for CGNSINF
        if cmd == "AT+CGNSINF":
            lat, lon, utc = parse_cgnsinf_response(response)
            if lat is not None and lon is not None:
                print(f"[CGNSINF] Valid fix found: UTC: {utc}, Latitude: {lat}, Longitude: {lon}")
                return lat, lon, utc
            else:
                print("[CGNSINF] No valid fix parsed from response.")
        else:
            # For other commands, just print the raw response for debugging.
            print(f"[{cmd}] Raw response:")
            print(response)
    return None, None, None

# -------------------------
# Main Script
# -------------------------
def main():
    port = '/dev/ttyUSB2'
    baudrate = 115200  # Adjust if needed
    timeout = 1

    ser = open_serial(port, baudrate, timeout)
    if ser is None:
        return

    # Check basic connectivity
    send_at_command(ser, "AT", wait=0.5)

    # Attempt to power on the GNSS engine.
    # (Your module may require a different command, e.g., "AT!GPS=1".)
    send_at_command(ser, "AT+CGNSPWR=1", wait=1)

    # Allow extra time for the GNSS engine to start and attempt a fix.
    print("Waiting for GNSS fix... (this may take a while; ensure you have a clear view of the sky)")
    time.sleep(10)

    # Set up live plotting (only activated if valid coordinates are parsed)
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Live GPS Coordinates")
    longitudes = []
    latitudes = []

    try:
        while True:
            # Poll through fallback commands.
            lat, lon, utc = poll_gps_data(ser)
            if lat is not None and lon is not None:
                longitudes.append(lon)
                latitudes.append(lat)
                print(f"Acquired GPS fix -> UTC: {utc}, Latitude: {lat}, Longitude: {lon}")
            else:
                print("No valid GPS fix found in this polling cycle.")

            # Update plot if data is available.
            ax.cla()
            ax.set_xlabel("Longitude")
            ax.set_ylabel("Latitude")
            ax.set_title("Live GPS Coordinates")
            ax.scatter(longitudes, latitudes, c='blue')
            plt.draw()
            plt.pause(0.5)

            # Wait a bit before the next polling cycle.
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nInterrupted by user, stopping GPS polling.")
    finally:
        ser.close()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
