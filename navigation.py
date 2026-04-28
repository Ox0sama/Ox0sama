from pymavlink import mavutil
import time

def set_mode(master, mode):
    # Check if the mode name is valid before doing anything
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}. Check your spelling.")
        return

    # Convert the mode name (e.g. "AUTO") to the number the Pixhawk understands
    mode_id = master.mode_mapping()[mode]

    # Send the mode change command to the Pixhawk
    master.set_mode(mode_id)

    # Wait for the Pixhawk to confirm the mode changed
    while True:
        # Listen for a response, timeout after 5 seconds
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

        # If no response came back in time, give up
        if ack is None:
            print(f"No response from Pixhawk — mode change timed out.")
            break

        # Check if this response is specifically for our mode change command
        if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode changed to {mode}")
            else:
                print(f"Failed to change mode: {ack.result}")
            break  # Either way, stop waiting

def upload_mission(master, waypoints):
    master.waypoint_clear_all_send()
    master.waypoint_count_send(len(waypoints))
    for i, (lat, lon, alt) in enumerate(waypoints):
        msg = master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
        if msg is None:
            print(f"Pixhawk stopped requesting waypoints at index {i}")
            break
