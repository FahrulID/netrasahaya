import serial
import time
import threading

ser = serial.Serial('COM3', 
    baudrate=115200, 
    parity=serial.PARITY_NONE, 
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS, 
    timeout=1
)

# check if the serial port is open
if ser.isOpen():
    print(ser.name + ' is open...')
else:
    exit()

# 0 to 255
motorSpeed1: int = 0
motorSpeed2: int = 0
motorSpeed3: int = 0
motorSpeed4: int = 0

# 0 to 4
motorState1: int = 0
motorState2: int = 0
motorState3: int = 0
motorState4: int = 0

def constructByteFromTwiNibble(nibble1: int, nibble2: int) -> int:
    return (nibble1 << 4) | nibble2

def sendCommand():
    global motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4, motorState1, motorState2, motorState3, motorState4

    secondByte = constructByteFromTwiNibble(motorState1, motorState2)
    thirdByte = constructByteFromTwiNibble(motorState3, motorState4)
    fourthByte = motorSpeed1
    fifthByte = motorSpeed2
    sixthByte = motorSpeed3
    seventhByte = motorSpeed4
    hashByte = secondByte ^ thirdByte ^ fourthByte ^ fifthByte ^ sixthByte ^ seventhByte

    command: bytearray = bytearray([0x66, secondByte, thirdByte, fourthByte, fifthByte, sixthByte, seventhByte, hashByte, 0x99])

    ser.write(command)
    print(f"Command sent: {command}")


def read_from_arduino():
    """Reads data from Arduino in a separate thread."""
    while True:
        try:
            if ser.in_waiting > 0:  # Check if data is available
                line = ser.readline().decode('utf-8').strip()
                print(f"Arduino says: {line}")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
            break


# Start a thread for reading Arduino data
reader_thread = threading.Thread(target=read_from_arduino, daemon=True)
reader_thread.start()

try:
    speed: int = 0
    state: int = 0
    while True:
        motorSpeed1 = speed
        motorSpeed2 = 255 - speed
        motorSpeed3 = speed
        motorSpeed4 = 255 - speed

        motorState1 = state
        motorState2 = 4 - state
        motorState3 = state
        motorState4 = 4 - state

        sendCommand()

        speed = (speed + 1) % 256
        state = (state + 1) % 5
        time.sleep(1)
except KeyboardInterrupt:
    print("KeyboardInterrupt detected. Exiting...")
finally:
    if ser.is_open:
        ser.close()
    print("Serial port closed. Goodbye!")