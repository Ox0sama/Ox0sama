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
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,                                      # Waypoint index number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Altitude is relative to home point
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # This is a navigation waypoint
            0, 1, 0, 0, 0, 0,                       # Default options, don't worry about these
            lat, lon, alt                            # The actual GPS coordinates
        )
        print(f"Sent waypoint {i}: lat={lat}, lon={lon}, alt={alt}m")

    # Wait for the Pixhawk to confirm it received all waypoints (e.g: latitude)
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack is None:
        print("Mission upload timed out — Pixhawk didn't confirm.")
    elif ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"Mission uploaded successfully! ({len(waypoints)} waypoints)")
    else:
        print(f"Mission upload failed. Error code: {ack.type}")

def fly_to(master, lat, lon, alt):
    set_mode(master, "GUIDED")
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        0,                                              # Waypoint index (just 1 point so index 0)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Altitude relative to home
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,          # Fly to this point
        2, 1, 0, 0, 0, 0,                              # The "2" means this is the target point
        lat, lon, alt
    )
    print(f"Flying to lat={lat}, lon={lon}, alt={alt}m")