from pymavlink import mavutil
import pygame
import time

from flight_controller import connect_pixhawk, arm_vehicle, disarm_vehicle, send_rc_override, release_rc_override
from navigation import upload_mission, start_mission, fly_to, hover_at_target, return_to_launch
from sensors import get_gps_position, get_barometer, get_imu
# ============================================================
# CONFIGURATION
# Change these values to match your setup
# ============================================================

# Pixhawk connection
CONNECTION_STRING = "COM5"       # Change to your port
BAUD_RATE = 115200

# PWM values
PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000

# Controller settings
SEND_HZ = 20
SEND_INTERVAL = 1.0 / SEND_HZ
DEADZONE = 0.1

# Channel mapping
ROLL_CH     = 1
PITCH_CH    = 2
THROTTLE_CH = 3
YAW_CH      = 4

# Payload servo — Hitec HS-55 plugged into channel 5 on the Pixhawk
SERVO_CHANNEL = 5
SERVO_OPEN_PWM  = 2000   # PWM to release the aid kit - CHECK
SERVO_CLOSED_PWM = 1000  # PWM to keep the aid kit locked - CHECK

# How long to hover while the payload drops (seconds)
HOVER_DURATION = 5

# Delivery waypoints (lat, lon, altitude in meters)
# Replace these coordinates with your actual delivery location
WAYPOINTS = [
    (40.7453, -74.0279, 30),   # Takeoff point (home)
    (40.7500, -74.0300, 30),   # Delivery location
]