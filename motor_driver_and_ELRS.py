from machine import Pin, I2C, UART
from pca9685 import PCA9685
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
STEERING_CH = 0  # Usually channel 1

# Initialize I2C
i2c = I2C(scl=Pin(21), sda=Pin(20))


def read_elrs():
    """
    Read ELRS SBUS data and return channel values
    Returns: List of 16 channel values (1000-2000 range)
    """
    if ELRS_UART.any():
        data = ELRS_UART.read(SBUS_PACKET_LENGTH)
        if data and len(data) == SBUS_PACKET_LENGTH and data[0] == 0x0F:
            channels = [0] * 16
            
            # Decode SBUS data
            current_byte = 1
            bit_position = 0
            
            for ch in range(16):
                channels[ch] = 0
                bits_needed = 11  # SBUS uses 11 bits per channel
                
                while bits_needed > 0:
                    if current_byte >= SBUS_PACKET_LENGTH - 2:
                        break
                        
                    remaining_bits = 8 - bit_position
                    bits_this_iteration = min(remaining_bits, bits_needed)
                    
                    mask = ((1 << bits_this_iteration) - 1) << bit_position
                    value = (data[current_byte] & mask) >> bit_position
                    
                    channels[ch] |= value << (11 - bits_needed)
                    
                    bits_needed -= bits_this_iteration
                    bit_position += bits_this_iteration
                    
                    if bit_position >= 8:
                        bit_position = 0
                        current_byte += 1
                
                # Convert to 1000-2000 range
                channels[ch] = int(channels[ch] * 0.625 + 880)
                
            return channels
    return None

def map_range(x, in_min, in_max, out_min, out_max):
    """Map value from one range to another"""
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# ...existing code...

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
            while True:
                channels = read_elrs()
                if channels:
                    # Get throttle and steering values
                    throttle = channels[THROTTLE_CH]
                    steering = channels[STEERING_CH]
                    
                    # Convert to motor speeds (-100 to 100)
                    base_speed = map_range(throttle, 1000, 2000, -100, 100)
                    turn_rate = map_range(steering, 1000, 2000, -100, 100)
                    
                    # Calculate individual track speeds
                    left_speed = base_speed
                    right_speed = base_speed
                    
                    # Apply steering
                    if turn_rate > 0:  # Turn right
                        right_speed = right_speed * (100 - turn_rate) / 100
                    else:  # Turn left
                        left_speed = left_speed * (100 + turn_rate) / 100
                    
                    # Set motor speeds
                    # set_track_speed(TRACK_LEFT, left_speed)
                    # set_track_speed(TRACK_RIGHT, right_speed)
                    
                time.sleep(0.02)  # 50Hz update rate
                
        except Exception as e:
            print("Error:", e)
            # motors_stop()  # Safety stop
        finally:
            # motors_stop()