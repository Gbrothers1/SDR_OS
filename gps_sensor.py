#!/usr/bin/env python3
"""
Script to list serial devices on Ubuntu.
This helps you identify which serial port corresponds to your GPS module.
"""

import serial.tools.list_ports

def list_serial_devices():
    # Retrieve a list of serial ports
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial devices found.")
        return

    print("Found serial devices:\n")
    for port in ports:
        print("-" * 50)
        print(f"Device       : {port.device}")
        print(f"Name         : {port.name}")
        print(f"Description  : {port.description}")
        print(f"Hardware ID  : {port.hwid}")
        # Additional details might help you identify your GPS device:
        print(f"VID          : {port.vid}")
        print(f"PID          : {port.pid}")
        print(f"Serial Number: {port.serial_number}")
        print(f"Location     : {port.location}")
        print(f"Manufacturer : {port.manufacturer}")
        print(f"Product      : {port.product}")
        print(f"Interface    : {port.interface}")
        print("-" * 50)
        print()

if __name__ == "__main__":
    list_serial_devices()

