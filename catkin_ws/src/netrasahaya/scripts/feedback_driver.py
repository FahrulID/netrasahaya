import rospy
import serial as ser
import message_filters
from std_msgs.msg import Int32
from typing import List

motor_speeds: List[int] = [0, 0, 0, 0]
motor_states: List[int] = [0, 0, 0, 0]

def constructByteFromTwiNibble(nibble1: int, nibble2: int) -> int:
    return (nibble1 << 4) | nibble2

def construct_command(speeds: List[int], states: List[int]) -> bytearray:
    secondByte = constructByteFromTwiNibble(states[0], states[1])
    thirdByte = constructByteFromTwiNibble(states[2], states[3])
    fourthByte = speeds[0]
    fifthByte = speeds[1]
    sixthByte = speeds[2]
    seventhByte = speeds[3]
    hashByte = secondByte ^ thirdByte ^ fourthByte ^ fifthByte ^ sixthByte ^ seventhByte

    command: bytearray = bytearray([0x66, secondByte, thirdByte, fourthByte, fifthByte, sixthByte, seventhByte, hashByte, 0x99])
    return command

MOTOR_KIRI = 1
MOTOR_KANAN = 2
MOTOR_DEPAN_KIRI = 0
MOTOR_DEPAN_KANAN = 3

# def callback(kiri: Int32, kanan: Int32, depan_kiri: Int32, depan_kanan: Int32):
#     global motor_speeds, motor_states

#     if kiri.data == 0:
#         motor_states[MOTOR_KIRI] = 0
#     elif kiri.data == 1:
#         motor_states[MOTOR_KIRI] = 2
#     elif kiri.data == 2:
#         motor_states[MOTOR_KIRI] = 1

#     if kanan.data == 0:
#         motor_states[MOTOR_KANAN] = 0
#     elif kanan.data == 1:
#         motor_states[MOTOR_KANAN] = 2
#     elif kanan.data == 2:
#         motor_states[MOTOR_KANAN] = 1

#     if depan_kiri.data == 0:
#         motor_states[MOTOR_DEPAN_KIRI] = 0
#     elif depan_kiri.data == 1:
#         motor_states[MOTOR_DEPAN_KIRI] = 2
#     elif depan_kiri.data == 2:
#         motor_states[MOTOR_DEPAN_KIRI] = 1

#     if depan_kanan.data == 0:
#         motor_states[MOTOR_DEPAN_KANAN] = 0
#     elif depan_kanan.data == 1:
#         motor_states[MOTOR_DEPAN_KANAN] = 2
#     elif depan_kanan.data == 2:
#         motor_states[MOTOR_DEPAN_KANAN] = 1

#     print(f'kiri: {motor_states[MOTOR_KIRI]}, kiri dada: {motor_states[MOTOR_DEPAN_KIRI]}, kanan dada: {motor_states[MOTOR_DEPAN_KANAN]}, kanan: {motor_states[MOTOR_KANAN]}')

def kiri_cb(msg: Int32):
    global motor_speeds, motor_states

    if msg.data == 0:
        motor_states[MOTOR_KIRI] = 0
    elif msg.data == 1:
        motor_states[MOTOR_KIRI] = 3
    elif msg.data == 2:
        motor_states[MOTOR_KIRI] = 2

def kanan_cb(msg: Int32):
    global motor_speeds, motor_states

    if msg.data == 0:
        motor_states[MOTOR_KANAN] = 0
    elif msg.data == 1:
        motor_states[MOTOR_KANAN] = 3
    elif msg.data == 2:
        motor_states[MOTOR_KANAN] = 2

def depan_kiri_cb(msg: Int32):
    global motor_speeds, motor_states

    if msg.data == 0:
        motor_states[MOTOR_DEPAN_KIRI] = 0
    elif msg.data == 1:
        motor_states[MOTOR_DEPAN_KIRI] = 3
    elif msg.data == 2:
        motor_states[MOTOR_DEPAN_KIRI] = 2

def depan_kanan_cb(msg: Int32):
    global motor_speeds, motor_states

    if msg.data == 0:
        motor_states[MOTOR_DEPAN_KANAN] = 0
    elif msg.data == 1:
        motor_states[MOTOR_DEPAN_KANAN] = 3
    elif msg.data == 2:
        motor_states[MOTOR_DEPAN_KANAN] = 2

if __name__ == '__main__':
    rospy.init_node('sensing')

    # kiri_sub = message_filters.Subscriber('/kiri', Int32)
    # kanan_sub = message_filters.Subscriber('/kanan', Int32)
    # depan_kiri_sub = message_filters.Subscriber('/depan_kiri', Int32)
    # depan_kanan_sub = message_filters.Subscriber('/depan_kanan', Int32)

    kiri_sub = rospy.Subscriber('/kiri', Int32, kiri_cb)
    kanan_sub = rospy.Subscriber('/kanan', Int32, kanan_cb)
    depan_kiri_sub = rospy.Subscriber('/depan_kiri', Int32, depan_kiri_cb)
    depan_kanan_sub = rospy.Subscriber('/depan_kanan', Int32, depan_kanan_cb)

    # ts = message_filters.ApproximateTimeSynchronizer([kiri_sub, kanan_sub, depan_kiri_sub, depan_kanan_sub], 10, 0.1, True)
    # ts.registerCallback(callback)
        
    s = ser.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_14101-if00', 
        baudrate=115200, 
        parity=ser.PARITY_NONE, 
        stopbits=ser.STOPBITS_ONE,
        bytesize=ser.EIGHTBITS, 
        timeout=1
    )

    if not s.isOpen():
        print("Serial port is not open, exiting...")

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            command = construct_command(motor_speeds, motor_states)
            s.write(command)
            print(f'Sent command: {command}')
        except Exception as e:
            print(f'Error: {e}')
            break

        rate.sleep()