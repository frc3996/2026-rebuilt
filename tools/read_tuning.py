#!/usr/bin/env python3
"""
Read FF calibration and PID tuning values from NetworkTables.
Connect to the robot (default 10.39.96.2) or specify an address.

Usage:
    python tools/read_tuning.py
    python tools/read_tuning.py 10.39.96.2
    python tools/read_tuning.py localhost
"""

import sys
import time

import ntcore

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "10.39.96.11"

inst = ntcore.NetworkTableInstance.getDefault()
inst.setServer(ROBOT_IP)
inst.startClient4("tuning_reader")

print(f"Connecting to {ROBOT_IP}...")
timeout = 5.0
start = time.time()
while not inst.isConnected():
    if time.time() - start > timeout:
        print("Failed to connect. Is the robot on?")
        sys.exit(1)
    time.sleep(0.1)
print("Connected!\n")

# Give NT a moment to sync
time.sleep(1.0)

TABLES = {
    "CalFF Shooter": ["kS (volts)", "kV (volts per RPM)", "kF (SparkMax FF)", "Status"],
    "CalFF Kicker Right": [
        "kS (volts)",
        "kV (volts per RPM)",
        "kF (SparkMax FF)",
        "Status",
    ],
    "CalFF Kicker Left": [
        "kS (volts)",
        "kV (volts per RPM)",
        "kF (SparkMax FF)",
        "Status",
    ],
    "PID Tuning": [
        "Shooter kP Low",
        "Shooter kP Mid",
        "Shooter kP High",
        "Kicker R kP Low",
        "Kicker R kP Mid",
        "Kicker R kP High",
        "Kicker L kP Low",
        "Kicker L kP Mid",
        "Kicker L kP High",
    ],
    "Manual": [
        "Shooter RPM",
        "Hood Position",
        "Distance To Hub",
    ],
    "Shooter": ["Velocity RPM", "Target RPM", "Amps"],
    "Kicker": [
        "Right Velocity RPM",
        "Left Velocity RPM",
        "Target RPM",
        "Right Amps",
        "Left Amps",
    ],
}

for table_name, keys in TABLES.items():
    table = inst.getTable(table_name)
    values = {}
    for key in keys:
        entry = table.getEntry(key)
        val = entry.getValue()
        if val.type() != ntcore.NetworkTableType.kUnassigned:
            values[key] = val.value()

    if not values:
        continue

    print(f"── {table_name} ──")
    for key, val in values.items():
        if isinstance(val, float):
            print(f"  {key}: {val:.6g}")
        else:
            print(f"  {key}: {val}")
    print()

inst.stopClient()
