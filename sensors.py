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

# This function will be dedicated for the BMP280
def get_barometer(master):
    msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=5)
    if msg is None:
        print("No barometer data received — check BMP280 connection.")
        return None
    pressure = msg.press_abs        # Atmospheric pressure in millibars
    temperature = msg.temperature / 100.0  # Comes in centidegrees, convert to Celsius

    print(f"Barometer — pressure:{pressure:.1f}mbar, temperature:{temperature:.1f}°C")
    return pressure, temperature

# This function will be dedicated for the MPU-9255
def get_imu(master): # Inertial Measurement Unit -> how drone is moving/rotating in the air
    msg = master.recv_match(type='RAW_IMU', blocking=True, timeout=5)
    if msg is None:
        print("No IMU data received — check MPU-9255 connection.")
        return None
    # Accelerometer — measures forces on the drone in each direction (in mG, milli-G)
    accel_x = msg.xacc
    accel_y = msg.yacc
    accel_z = msg.zacc

    # Gyroscope — measures rotation speed around each axis (in mrad/s)
    gyro_x = msg.xgyro
    gyro_y = msg.ygyro
    gyro_z = msg.zgyro

    # Magnetometer — measures magnetic field, used like a compass (in mT, milli-Tesla)
    mag_x = msg.xmag
    mag_y = msg.ymag
    mag_z = msg.zmag

    print(f"IMU — accel:({accel_x},{accel_y},{accel_z}) gyro:({gyro_x},{gyro_y},{gyro_z}) mag:({mag_x},{mag_y},{mag_z})")
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z