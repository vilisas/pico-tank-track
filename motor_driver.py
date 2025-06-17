from machine import Pin, I2C
from pca9685 import PCA9685
import time

RESOLUTION = 2<<11  # 12-bit resolution for PCA9685

# RESOLUTION  = 4096  # 12-bit resolution for PCA9685
TRACK_LEFT  = 0
TRACK_RIGHT = 1

# Initialize I2C
i2c = I2C(scl=Pin(21), sda=Pin(20))

# PCA9685 initialization
pca = PCA9685(i2c)
pca.freq(50)  # Set PWM frequency to 50Hz


def motors_stop():
    """Stop all motors"""
    # set_motor_speed(MOTOR_A_FWD, MOTOR_A_BWD, 0)
    # set_motor_speed(MOTOR_B_FWD, MOTOR_B_BWD, 0)
    for i in range(0, 5):
        pca.duty(i, 0)

def set_track_speed(track, speed):
    """
    Set speed for a specific track
    track: 0 for left, 1 for right
    speed: -100 to +100 (negative for backward, positive for forward). Set to 0 to stop.
    """
    if track not in [0, 1]:
        raise ValueError("Invalid track specified. Use 0 for left and 1 for right.")

    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
        
    if track == 0:
        if speed >0:
            pca.duty(1, 0)            
            pca.duty(0, int(speed * RESOLUTION / 100))
            pca.duty(2, int(speed * RESOLUTION / 100))
        elif speed <0:
            pca.duty(2, 0)
            pca.duty(0, int(-speed * RESOLUTION / 100))
            pca.duty(1, int(-speed * RESOLUTION / 100))
        elif speed == 0:
            pca.duty(0, 0)
            pca.duty(1, 0)
            pca.duty(2, 0)
    elif track == 1:
        if speed >0:
            pca.duty(5, 0)
            pca.duty(3, int(speed * RESOLUTION / 100))
            pca.duty(4, int(speed * RESOLUTION / 100))
        elif speed <0:
            pca.duty(4, 0)
            pca.duty(3, int(-speed * RESOLUTION / 100))
            pca.duty(5, int(-speed * RESOLUTION / 100))
        elif speed == 0:
            pca.duty(3, 0)
            pca.duty(4, 0)
            pca.duty(5, 0)
        

        
# Example usage
if __name__ == "__main__":
    devices = i2c.scan()
    if len(devices) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:', len(devices))
        for device in devices:
            print("I2C hexadecimal address: ", hex(device))
        
        # Test motor movement
        try:
            min_speed = 30
            max_speed = 70
            for speed in range(min_speed, max_speed):
                set_track_speed(TRACK_LEFT, speed)
                set_track_speed(TRACK_RIGHT, speed)
                time.sleep(0.2)
            for speed in range(max_speed, min_speed, -1):
                set_track_speed(TRACK_LEFT, speed)
                set_track_speed(TRACK_RIGHT, speed)
                time.sleep(0.1)

            print("Stopping")
            # set_track_speed(TRACK_LEFT, 0)  # Stop left track
            # pca.duty(a, 0)
            # pca.duty(b, 0)
            
            
            motors_stop()
            
        except Exception as e:
            print("Error:", e)
            motors_stop()  # Safety stop