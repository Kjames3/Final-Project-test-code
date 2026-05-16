import sys
import time
import threading

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Error: RPi.GPIO not found. Please ensure you are running this on a Raspberry Pi with the RPi.GPIO library installed.")
    sys.exit(1)

# Motor A pins (using BCM numbering based on the Yahboom documentation)
AIN1 = 17
AIN2 = 18

# Motor B pins
BIN1 = 22
BIN2 = 23

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize pins
GPIO.setup(AIN1, GPIO.OUT)
p1 = GPIO.PWM(AIN1, 50)
p1.start(0)

GPIO.setup(AIN2, GPIO.OUT)
p2 = GPIO.PWM(AIN2, 50)
p2.start(0)

GPIO.setup(BIN1, GPIO.OUT)
p3 = GPIO.PWM(BIN1, 50)
p3.start(0)

GPIO.setup(BIN2, GPIO.OUT)
p4 = GPIO.PWM(BIN2, 50)
p4.start(0)

def forward(speed=100):
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(speed)
    p3.ChangeDutyCycle(0)
    p4.ChangeDutyCycle(speed)

def backward(speed=100):
    p1.ChangeDutyCycle(speed)
    p2.ChangeDutyCycle(0)
    p3.ChangeDutyCycle(speed)
    p4.ChangeDutyCycle(0)

def stop():
    p1.ChangeDutyCycle(0)
    p2.ChangeDutyCycle(0)
    p3.ChangeDutyCycle(0)
    p4.ChangeDutyCycle(0)

stop_event = threading.Event()

def wait_for_enter():
    input("Press ENTER at any time to stop the test...\n")
    stop_event.set()

def test_motors():
    print("Starting motor test...")
    print("Sequence: Forward 3s -> Pause 0.5s -> Backward 3s -> Pause 0.5s -> Repeat")
    
    enter_thread = threading.Thread(target=wait_for_enter, daemon=True)
    enter_thread.start()
    
    try:
        while not stop_event.is_set():
            # 1. Spin forward
            print("Motors spinning FORWARD...")
            forward(50)  # Using 50% speed. Adjust up to 100 if needed.
            
            # Wait 3 seconds, breaking early if enter is pressed
            for _ in range(30):
                if stop_event.is_set():
                    break
                time.sleep(0.1)
                
            if stop_event.is_set():
                break
                
            # 2. Pause half a second
            print("Pausing...")
            stop()
            for _ in range(5):
                if stop_event.is_set():
                    break
                time.sleep(0.1)

            if stop_event.is_set():
                break
                
            # 3. Spin backward
            print("Motors spinning BACKWARD...")
            backward(50)
            
            # Wait 3 seconds
            for _ in range(30):
                if stop_event.is_set():
                    break
                time.sleep(0.1)
                
            if stop_event.is_set():
                break
                
            # 4. Pause half a second
            print("Pausing...")
            stop()
            for _ in range(5):
                if stop_event.is_set():
                    break
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        stop_event.set()

    print("\nStopping motors and cleaning up...")
    stop()
    time.sleep(0.1) # Small delay to ensure PWM updates
    p1.stop()
    p2.stop()
    p3.stop()
    p4.stop()
    GPIO.cleanup()
    print("Test completed.")

if __name__ == '__main__':
    test_motors()
