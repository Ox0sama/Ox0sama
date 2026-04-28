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

    # Keep checking the drone's GPS position until it's close enough to the target
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        
        # If no GPS data came back, skip this check and try again
        if msg is None:
            continue
        
        # GPS coordinates from the Pixhawk come multiplied by 1e7, so we divide to get real lat/lon
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000  # comes in millimeters, convert to meters
        
        # Calculate how far we are from the target (roughly, in degrees)
        lat_diff = abs(current_lat - lat)
        lon_diff = abs(current_lon - lon)
        alt_diff = abs(current_alt - alt)
        
        print(f"\rCurrent: lat={current_lat:.6f}, lon={current_lon:.6f}, alt={current_alt:.1f}m", end="")
        
        # If we're within ~1 meter horizontally and 2 meters vertically, we've arrived
        if lat_diff < 0.00001 and lon_diff < 0.00001 and alt_diff < 2.0:
            print(f"\nArrived at destination!")
            break

def start_mission(master):
    set_mode(master, "AUTO")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,  # Command to begin the mission
        0,                                       # Confirmation number (0 = first attempt)
        0, 0, 0, 0, 0, 0, 0                     # Unused parameters, required as placeholders
    )
    print("Mission started!")
    # Wait for Pixhawk to confirm the mission start command was received
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None:
        print("No response from Pixhawk — mission start timed out.")
    elif ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Pixhawk confirmed — mission is running!")
    else:
        print(f"Mission start failed. Error code: {ack.result}")

def hover_at_target(master, duration_seconds):
    set_mode(master, "LOITER")
    print(f"Hovering for {duration_seconds} seconds...")
    time.sleep(duration_seconds)
    print("Hover complete — ready for next action")

def return_to_launch(master):
    set_mode(master, "RTL") # RTL stands for Return to Launch
    print("Returning to launch point...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            continue

        # Check if the drone has landed (altitude close to 0)
        current_alt = msg.relative_alt / 1000  # millimeters to meters
        print(f"\rAltitude: {current_alt:.1f}m", end="")

        if current_alt < 0.5:
            print("\nDrone has landed safely at home!")
            break
