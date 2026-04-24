import time
import pygame
from pymavlink import mavutil

# -----------------------------
# CONNECTION
# -----------------------------
CONNECTION_STRING = "COM5"  # Windows example; adjust as needed for your system
# CONNECTION_STRING = "/dev/ttyACM0"   # Linux example; adjust as needed for your system
BAUD_RATE = 115200

# -----------------------------
# CHANNEL MAPPING
# ArduCopter standard stick mapping is typically:
# CH1 = Roll
# CH2 = Pitch
# CH3 = Throttle
# CH4 = Yaw
# -----------------------------
ROLL_CH = 1
PITCH_CH = 2
THROTTLE_CH = 3
YAW_CH = 4

# PWM values
PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000

# Update rate
SEND_HZ = 20
SEND_INTERVAL = 1.0 / SEND_HZ

# Deadzone for sticks
DEADZONE = 0.1

# -----------------------------
# HELPERS
# -----------------------------
def clamp(value, low, high):
    return max(low, min(high, value))

def apply_deadzone(v, dz=DEADZONE):
    return 0.0 if abs(v) < dz else v

def axis_to_pwm_centered(v):
    """
    For roll/pitch/yaw axes: input -1..1 => 1000..2000 with 1500 center
    """
    v = apply_deadzone(v)
    pwm = int(PWM_MID + (v * 500))
    return clamp(pwm, PWM_MIN, PWM_MAX)

def axis_to_pwm_throttle(v):
    """
    Gamepad axis often reports:
      up/released = -1
      down/pressed = +1
    Convert to 1000..2000
    """
    v = clamp(v, -1.0, 1.0)
    normalized = (v + 1.0) / 2.0   # 0..1
    pwm = int(PWM_MIN + normalized * 1000)
    return clamp(pwm, PWM_MIN, PWM_MAX)

def connect_pixhawk():
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected. System={master.target_system}, Component={master.target_component}")
    return master

def arm_vehicle(master):
    print("Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")

def disarm_vehicle(master):
    print("Disarming...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Disarmed")

def send_rc_override(master, roll, pitch, throttle, yaw):
    """
    Sends RC override for channels 1-4 and ignores 5-18.
    """
    values = [65535] * 18
    values[ROLL_CH - 1] = roll
    values[PITCH_CH - 1] = pitch
    values[THROTTLE_CH - 1] = throttle
    values[YAW_CH - 1] = yaw

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *values
    )

def release_rc_override(master):
    """
    Release channels 1-4 back to RC radio / normal control.
    For channels 1-8, 0 means release.
    For channels 9-18, UINT16_MAX-1 means release, but we're not using those here.
    """
    values = [65535] * 18
    values[0] = 0
    values[1] = 0
    values[2] = 0
    values[3] = 0

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *values
    )
    print("RC override released")

def main():
    master = connect_pixhawk()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Controller detected:", joystick.get_name())

    print("\nControls:")
    print(" Left stick X  -> Roll")
    print(" Left stick Y  -> Pitch")
    print(" Right stick X -> Yaw")
    print(" Right trigger -> Throttle")
    print(" A button      -> Arm")
    print(" B button      -> Disarm")
    print(" Ctrl+C        -> Exit and release override\n")

    try:
        while True:
            pygame.event.pump()

            # Typical Xbox-style mapping; may vary by controller
            left_x = joystick.get_axis(0)      # roll
            left_y = joystick.get_axis(1)      # pitch
            right_x = joystick.get_axis(3)     # yaw
            throttle_axis = joystick.get_axis(5)  # trigger

            # Invert pitch because pushing the stick forward often gives +1
            roll_pwm = axis_to_pwm_centered(left_x)
            pitch_pwm = axis_to_pwm_centered(-left_y)
            yaw_pwm = axis_to_pwm_centered(right_x)
            throttle_pwm = axis_to_pwm_throttle(throttle_axis)

            # Buttons
            a_pressed = joystick.get_button(1)
            b_pressed = joystick.get_button(2)

            if a_pressed:
                arm_vehicle(master)
                time.sleep(0.5)

            if b_pressed:
                disarm_vehicle(master)
                time.sleep(0.5)

            send_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm)

            print(
                f"\rRoll:{roll_pwm} Pitch:{pitch_pwm} Throttle:{throttle_pwm} Yaw:{yaw_pwm}",
                end=""
            )

            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping...")
        release_rc_override(master)
        pygame.quit()

if __name__ == "__main__":
    main()
