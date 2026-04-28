from pymavlink import mavutil
import time

# This function will be dedicated for the JeeFly M8N GPS Module
def get_gps_position(master):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg is None:
        print("No GPS data received — check GPS connection.")
        return None
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000  # millimeters to meters

    print(f"GPS Position — lat:{lat:.6f}, lon:{lon:.6f}, alt:{alt:.1f}m")
    return lat, lon, alt