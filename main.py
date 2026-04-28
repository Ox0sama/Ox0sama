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

def release_payload(master):
    # Send a PWM signal to the servo channel to open the release mechanism
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command to move a servo
        0,                                      # Confirmation number
        SERVO_CHANNEL,                          # Which channel the HS-55 is on
        SERVO_OPEN_PWM,                         # PWM value to open/release
        0, 0, 0, 0, 0                           # Unused parameters
    )
    print("Payload released — aid kit dropped!")
    time.sleep(2)  # Wait 2 seconds to make sure the servo fully opens

    # Close the servo back (in case of a second delivery or safe stowing)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_CHANNEL,
        SERVO_CLOSED_PWM,                       # PWM value to close/lock
        0, 0, 0, 0, 0
    )
    print("Servo closed.")

def main():
    # Connect to the Pixhawk
    master = connect_pixhawk()

    # Arm the drone (enables motors)
    arm_vehicle(master)

    # Upload the delivery waypoints to the Pixhawk
    upload_mission(master, WAYPOINTS)

    # Begin flying the mission
    start_mission(master)

    # Fly to the delivery location
    fly_to(master, WAYPOINTS[1][0], WAYPOINTS[1][1], WAYPOINTS[1][2])

    # Hover in place while the payload drops
    hover_at_target(master, HOVER_DURATION)

    # Release the aid kit
    release_payload(master)

    # Fly back home
    return_to_launch(master)

    # Disarm once landed
    disarm_vehicle(master)
    print("Mission complete!")

if __name__ == "__main__":
    main()