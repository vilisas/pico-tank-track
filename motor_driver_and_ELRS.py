from machine import Pin, I2C, UART
from pca9685 import PCA9685
from motor_driver import set_track_speed, motors_stop

import time

# Constants
RESOLUTION  = 4096  # 12-bit resolution for PCA9685
TRACK_LEFT  = 0
TRACK_RIGHT = 1

# ELRS UART Configuration
ELRS_RX_PIN = 1  # GPIO1 for UART0 RX
ELRS_UART = UART(0, baudrate=420000, rx=Pin(ELRS_RX_PIN))
SBUS_PACKET_LENGTH = 25

# Channel mapping
THROTTLE_CH = 2  # Usually channel 3
STEERING_CH = 1  # Usually channel 1
ARMED_CH    = 5  # Usually channel 6 (armed state)

ARM_THRESHOLD = 1500  # Threshold for arming channel
SPEED_ARM_THRESHOLD = 10  # Throttle/steering must be within +/- this to arm
MAX_SPEED = 50  # Max speed for throttle
MAX_TURN_RATE = 50  # Max turn rate for steering
JITTER_THRESHOLD = 7


# Initialize I2C
i2c = I2C(scl=Pin(21), sda=Pin(20))


def read_elrs():
    """
    Read ELRS CRSF data and return channel values.
    Returns: List of 16 channel values (1000-2000 range) or None
    """
    while ELRS_UART.any():
        ELRS_UART.read(ELRS_UART.any())  # Clear any existing data
        # Read one byte to find start of frame
        b = ELRS_UART.read(1)
        if not b or b[0] != 0xC8:  # CRSF address for radio TX
            continue

        # Read length byte
        if not ELRS_UART.any():
            return None
        length = ELRS_UART.read(1)[0]
        if length < 2 or length > 64:
            continue

        # Read rest of frame (length bytes)
        frame = ELRS_UART.read(length)
        if not frame or len(frame) != length:
            continue

        # Check for channel data frame (type 0x16)
        if frame[0] == 0x16:
            # 16 channels, 22 bytes, packed 11 bits per channel, little-endian
            channels = []
            data = frame[1:23]  # 22 bytes
            bits = 0
            bitbuf = 0
            for byte in data:
                bitbuf |= byte << bits
                bits += 8
                if bits >= 11:
                    channels.append(bitbuf & 0x7FF)
                    bitbuf >>= 11
                    bits -= 11
            # Convert to 1000-2000 range
            return [int(ch * 0.624 + 880) for ch in channels[:16]]
            # return [int(ch) for ch in channels[:16]]
    return None

def map_range(x, in_min, in_max, out_min, out_max):
    """Map value from one range to another"""
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# motor does brr
def armed_indication():
    for i in range(3):
        set_track_speed(TRACK_LEFT, 3)
        set_track_speed(TRACK_RIGHT, 3)
        time.sleep(0.1)
        motors_stop()
        time.sleep(0.1)


if __name__ == "__main__":
    print('I2C SCANNER')
    devices = i2c.scan()

    if len(devices) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:', len(devices))
        for device in devices:
            print("I2C hexadecimal address: ", hex(device))
        
        # Main control loop
        try:
            print("Starting ELRS control loop")
            last_armed = False
            armed = False

            while True:
                channels = read_elrs()
                if channels:
                    # Get throttle and steering values
                    throttle = channels[THROTTLE_CH]
                    steering = channels[STEERING_CH-1]  # Adjust for zero-based index

                    # Convert to motor speeds (-100 to 100)
                    base_speed = map_range(throttle, 1000, 2000, -MAX_SPEED, MAX_SPEED)
                    turn_rate = map_range(steering, 1000, 2000, -MAX_TURN_RATE, MAX_TURN_RATE)

                    # Arm channel logic
                    arm_channel_input = channels[ARMED_CH-1] > ARM_THRESHOLD

                    # Only allow arming if both base_speed and turn_rate are near zero
                    if not armed and arm_channel_input:
                        if abs(base_speed) < SPEED_ARM_THRESHOLD and abs(turn_rate) < SPEED_ARM_THRESHOLD:
                            armed = True
                            print("System ARMED")
                            armed_indication()
                        else:
                            print("Cannot arm: throttle/steering not centered")
                    elif armed and not arm_channel_input:
                        armed = False
                        print("System DISARMED")

                    # show all channels
                    print("Channels:", channels[0:8])  # Show first 8 channels for brevity

                    # Calculate individual track speeds
                    left_speed = base_speed + turn_rate
                    right_speed = base_speed - turn_rate

                    # Ensure speeds are within -100 to 100 range
                    if armed:
                        left_speed = max(-100, min(100, left_speed))
                        right_speed = max(-100, min(100, right_speed))
                        # add threshold to avoid jitter
                        if abs(left_speed) < JITTER_THRESHOLD:
                            left_speed = 0
                        if abs(right_speed) < JITTER_THRESHOLD:
                            right_speed = 0
                        print(f"Left Speed: {left_speed}, Right Speed: {right_speed} armed: {armed}")
                    else:
                        left_speed = 0
                        right_speed = 0
                        print("Motors stopped, not armed")

                    # Apply steering
                    if turn_rate > 0:  # Turn right
                        right_speed = right_speed * (100 - turn_rate) / 100
                    else:  # Turn left
                        left_speed = left_speed * (100 + turn_rate) / 100

                    # Set motor speeds
                    set_track_speed(TRACK_LEFT, left_speed)
                    set_track_speed(TRACK_RIGHT, right_speed)

                time.sleep(0.02)  # 50Hz update rate
        except Exception as e:
            print("Error:", e)
            # motors_stop()  # Safety stop
        # finally:
            # motors_stop()