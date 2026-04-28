import time
import pygame
from pymavlink import mavutil

# ============================================================
# CONNECTION SETTINGS
# This is where you tell the code HOW to talk to the Pixhawk.
# CONNECTION_STRING is the port your Pixhawk is plugged into.
#   - On Windows, it looks like "COM5" (check Device Manager to find yours)
#   - On Linux/Mac, it looks like "/dev/ttyACM0"
# BAUD_RATE is the communication speed. 115200 is the standard for Pixhawk.
# ============================================================
CONNECTION_STRING = "COM5"
# CONNECTION_STRING = "/dev/ttyACM0"  # Uncomment this line if you're on Linux/Mac
BAUD_RATE = 115200

# ============================================================
# CHANNEL MAPPING
# RC (radio control) drones use "channels" to send commands.
# Each channel controls one thing:
#   CH1 = Roll    (tilting left/right)
#   CH2 = Pitch   (tilting forward/backward)
#   CH3 = Throttle (going up/down — motor speed)
#   CH4 = Yaw     (spinning left/right on the spot)
# These numbers are the ArduCopter standard and should not be changed
# unless you've reconfigured your flight controller.
# ============================================================
ROLL_CH     = 1
PITCH_CH    = 2
THROTTLE_CH = 3
YAW_CH      = 4

# ============================================================
# PWM VALUES
# PWM (Pulse Width Modulation) is the signal language drones use.
# Think of it like a volume knob that goes from 1000 to 2000:
#   PWM_MIN = 1000  -> full left / full back / zero throttle
#   PWM_MID = 1500  -> centered / neutral position
#   PWM_MAX = 2000  -> full right / full forward / full throttle
# ============================================================
PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000

# ============================================================
# UPDATE RATE
# SEND_HZ controls how many times per second we send commands
# to the drone. 20 times/sec (20 Hz) is a good balance between
# responsiveness and not flooding the connection.
# SEND_INTERVAL is just 1 divided by 20 = 0.05 seconds between sends.
# ============================================================
SEND_HZ       = 20
SEND_INTERVAL = 1.0 / SEND_HZ

# ============================================================
# DEADZONE
# Joystick sticks are never perfectly centered — they drift slightly.
# The deadzone ignores any tiny stick movement below this threshold
# so the drone doesn't twitch when you're not touching the controller.
# 0.1 means "ignore movements smaller than 10% of full stick travel."
# ============================================================
DEADZONE = 0.1


# ============================================================
# HELPER FUNCTIONS
# These are small utility tools used by the main control loop.
# ============================================================

def clamp(value, low, high):
    """
    Keeps a number inside a safe range.
    Example: clamp(2500, 1000, 2000) returns 2000
    This prevents us from ever sending a PWM value the drone can't handle.
    """
    return max(low, min(high, value))


def apply_deadzone(v, dz=DEADZONE):
    """
    If the stick value is very small (inside the deadzone), treat it as zero.
    This stops tiny accidental inputs from moving the drone.
    """
    return 0.0 if abs(v) < dz else v


def axis_to_pwm_centered(v):
    """
    Converts a joystick axis value into a PWM signal for roll, pitch, or yaw.
    Joystick axes go from -1.0 (full left/up) to +1.0 (full right/down).
    We map that to PWM 1000–2000, with 1500 as the center (no movement).
    Example: stick at 0.0 (center) → PWM 1500
             stick at 1.0 (full right) → PWM 2000
             stick at -1.0 (full left) → PWM 1000
    """
    v = apply_deadzone(v)
    pwm = int(PWM_MID + (v * 500))
    return clamp(pwm, PWM_MIN, PWM_MAX)


def axis_to_pwm_throttle(v):
    """
    Converts the throttle axis into a PWM signal for motor speed.
    Many controllers report trigger/throttle axes from -1 (released) to +1 (fully pressed).
    We convert that range to PWM 1000 (no throttle) to 2000 (full throttle).
    Example: trigger released (-1.0) → PWM 1000 (motors at minimum)
             trigger half pressed (0.0) → PWM 1500
             trigger fully pressed (1.0) → PWM 2000 (full power)
    """
    v = clamp(v, -1.0, 1.0)
    normalized = (v + 1.0) / 2.0   # Converts -1..1 range into 0..1 range
    pwm = int(PWM_MIN + normalized * 1000)
    return clamp(pwm, PWM_MIN, PWM_MAX)


# ============================================================
# PIXHAWK CONNECTION
# ============================================================

def connect_pixhawk():
    """
    Opens a communication link to the Pixhawk flight controller.
    wait_heartbeat() pauses the code until the Pixhawk responds,
    confirming the connection is live before we do anything else.
    """
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected! System={master.target_system}, Component={master.target_component}")
    return master


# ============================================================
# ARMING AND DISARMING
# "Arming" means allowing the motors to spin.
# The drone WILL NOT fly until it is armed.
# "Disarming" cuts motor power — always do this when landing.
# ============================================================

def arm_vehicle(master):
    """
    Arms the drone (enables motors).
    motors_armed_wait() pauses until the Pixhawk confirms it's armed.
    """
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Motors armed — drone is ready to fly!")


def disarm_vehicle(master):
    """
    Disarms the drone (cuts motor power).
    Always disarm after landing to prevent accidental motor spin.
    """
    print("Disarming motors...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Motors disarmed — drone is safe.")


# ============================================================
# SENDING RC COMMANDS
# ============================================================

def send_rc_override(master, roll, pitch, throttle, yaw):
    """
    Sends joystick commands directly to the Pixhawk as RC channel values.
    This overrides any physical RC radio transmitter that might be connected.
    Channels 5–18 are set to 65535 which means "don't override these channels"
    so any other features (like flight modes) still work normally.
    """
    # Start with 18 channels all set to "don't touch" (65535)
    values = [65535] * 18

    # Fill in the 4 channels we actually want to control
    values[ROLL_CH - 1]     = roll
    values[PITCH_CH - 1]    = pitch
    values[THROTTLE_CH - 1] = throttle
    values[YAW_CH - 1]      = yaw

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *values
    )


def release_rc_override(master):
    """
    Releases our software control of the RC channels.
    Setting channels 1–4 to 0 tells the Pixhawk to stop listening
    to our overrides and go back to the physical RC radio (or disarm).
    Always call this when the program exits so the drone isn't left
    waiting for commands that will never come.
    """
    values = [65535] * 18
    values[0] = 0  # Release Roll
    values[1] = 0  # Release Pitch
    values[2] = 0  # Release Throttle
    values[3] = 0  # Release Yaw

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *values
    )
    print("RC override released — Pixhawk is back in normal control mode.")


# ============================================================
# MAIN CONTROL LOOP
# NOTE: This function will be moved to main.py in the final
# multi-file structure. It is here temporarily for testing.
# ============================================================

def main():
    # Connect to the Pixhawk first
    master = connect_pixhawk()

    # Start up the pygame library (used to read the controller)
    pygame.init()
    pygame.joystick.init()

    # Check that a controller is actually plugged in
    if pygame.joystick.get_count() == 0:
        print("No controller detected. Plug in your controller and try again.")
        return

    # Initialize the first detected controller (index 0)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Controller detected:", joystick.get_name())

    # Print the control layout for the pilot
    print("\nControls:")
    print(" Left stick X  -> Roll    (tilt left/right)")
    print(" Left stick Y  -> Pitch   (tilt forward/back)")
    print(" Right stick X -> Yaw     (spin left/right)")
    print(" Right trigger -> Throttle (motor speed)")
    print(" Arm button    -> Arm drone (enables motors)")
    print(" Disarm button -> Disarm drone (cuts motors)")
    print(" Ctrl+C        -> Exit safely and release control\n")
    print("NOTE: Check your controller's button layout and update")
    print("the button indices below to match your specific controller.\n")

    # --------------------------------------------------------
    # ARMED STATE TRACKING
    # We keep track of whether the drone is armed so we don't
    # accidentally try to arm it again mid-flight, or disarm
    # it when it's already disarmed.
    # --------------------------------------------------------
    is_armed = False

    try:
        while True:
            # Process all controller events (must be called every loop)
            pygame.event.pump()

            # --------------------------------------------------
            # READ JOYSTICK AXES
            # Axis indices depend on your specific controller.
            # Run a test script to find which axis number maps
            # to which stick on YOUR controller before flying.
            # Axis values range from -1.0 to +1.0.
            # --------------------------------------------------
            left_x       = joystick.get_axis(0)  # Left stick horizontal  -> Roll
            left_y       = joystick.get_axis(1)  # Left stick vertical    -> Pitch
            right_x      = joystick.get_axis(3)  # Right stick horizontal -> Yaw
            throttle_axis = joystick.get_axis(5) # Right trigger          -> Throttle

            # Convert raw axis values to PWM signals
            # Pitch is inverted: pushing stick forward (negative axis) should pitch forward (positive PWM)
            roll_pwm     = axis_to_pwm_centered(left_x)
            pitch_pwm    = axis_to_pwm_centered(-left_y)  # Inverted intentionally
            yaw_pwm      = axis_to_pwm_centered(right_x)
            throttle_pwm = axis_to_pwm_throttle(throttle_axis)

            # --------------------------------------------------
            # READ BUTTONS
            # IMPORTANT: Button indices are controller-specific!
            # The numbers below (1 and 2) may not match your controller.
            # Use pygame's joystick test to find the correct indices.
            # Example test: print(joystick.get_button(i)) for i in range(joystick.get_numbuttons())
            # --------------------------------------------------
            arm_pressed   = joystick.get_button(1)  # Change index to match your ARM button
            disarm_pressed = joystick.get_button(2) # Change index to match your DISARM button

            # Arm only if not already armed
            if arm_pressed and not is_armed:
                arm_vehicle(master)
                is_armed = True
                time.sleep(0.5)  # Small delay to prevent button bounce (double-press)

            # Disarm only if currently armed
            if disarm_pressed and is_armed:
                disarm_vehicle(master)
                is_armed = False
                time.sleep(0.5)  # Small delay to prevent button bounce

            # Send the calculated PWM values to the Pixhawk
            send_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm)

            # Print current values to the screen on one line (overwrites itself)
            print(
                f"\rRoll:{roll_pwm}  Pitch:{pitch_pwm}  Throttle:{throttle_pwm}  Yaw:{yaw_pwm}  Armed:{is_armed}",
                end=""
            )

            # Wait before sending the next update (keeps us at SEND_HZ rate)
            time.sleep(SEND_INTERVAL)

    except KeyboardInterrupt:
        # Ctrl+C was pressed — safely shut everything down
        print("\nStopping program...")
        release_rc_override(master)
        pygame.quit()


if __name__ == "__main__":
    main()