import threading
from time import sleep
from tkinter import *
import serial as ser
from typing import List

motor_speeds: List[int] = [0, 0, 0, 0]
motor_states: List[int] = [0, 0, 0, 0]
send_state = True

kill_signal = False

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

def command_to_string(command: bytearray) -> str:
    return " ".join(f"{byte:02X}" for byte in command)

def gui():
    global motor_speeds, motor_states, kill_signal, send_state

    root = Tk()

    send_list = ["Send", "Stop"]
    motors_list = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]
    states_list = ["stop", "manual forward", "slow forward", "fast forward", "manual backward", "slow backward", "fast backward"]

    main_frame = Frame(root)
    main_frame.pack()

    def update_send_state(value):
        global send_state
        send_state = value == "Send"
        
    send_state_var = StringVar(root)
    send_state_var.set(send_list[0])
    send_state_menu = OptionMenu(main_frame, send_state_var, *send_list, command=update_send_state)
    send_state_menu.pack(side=TOP)

    def update_motor_speed(index, value):
        motor_speeds[index] = int(value)

    def update_motor_state(index, value):
        motor_states[index] = states_list.index(value)

    for i, motor in enumerate(motors_list):
        frame = Frame(main_frame)
        frame.pack(side=LEFT, padx=10)
        Label(frame, text=motor).pack(side=TOP)
        w = Scale(frame, from_=0, to=255, orient=VERTICAL, command=lambda value, idx=i: update_motor_speed(idx, value))
        w.pack(side=LEFT)

        state = StringVar(frame)
        state.set(states_list[0])
        state_menu = OptionMenu(frame, state, *states_list, command=lambda value, idx=i: update_motor_state(idx, value))
        state_menu.config(width=max(len(state) for state in states_list))
        state_menu.pack(side=LEFT)

    command_label = Label(root, text="")
    command_label.pack(side=BOTTOM)

    def update_gui():
        global motor_speeds, motor_states, kill_signal

        while True:
            if kill_signal:
                print("Exiting update_gui")
                break
            command = construct_command(motor_speeds, motor_states)
            command_label.config(text=f"Command: {command_to_string(command)}, send: {send_state}")
            sleep(0.1)

    update_thread = threading.Thread(target=update_gui, daemon=True)
    update_thread.start()

    root.mainloop()

def serial():
    global motor_speeds, motor_states, kill_signal, send_state

    try:
        s = ser.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_14101-if00', 
            baudrate=115200, 
            parity=ser.PARITY_NONE, 
            stopbits=ser.STOPBITS_ONE,
            bytesize=ser.EIGHTBITS, 
            timeout=1
        )

        if not s.isOpen():
            print("Serial port is not open, exiting...")
            return
        
        def read_from_arduino():
            global kill_signal
            """Reads data from Arduino in a separate thread."""
            while True:
                try:
                    if s.in_waiting > 0:  # Check if data is available
                        line = s.readline().decode('utf-8').strip()
                        print(f"Arduino says: {line}")

                    if kill_signal:
                        print("Exiting read_from_arduino")
                        break
                except Exception as e:
                    print(f"Error reading from Arduino: {e}")
                    break
            print("Exiting read_from_arduino")

        def write_to_arduino():
            global motor_speeds, motor_states, kill_signal, send_state
            """Writes data to Arduino in a separate thread."""
            while True:
                try:
                    if not send_state:
                        print("Not sending command")
                        sleep(0.1)
                        continue

                    command = construct_command(motor_speeds, motor_states)
                    s.write(command)
                    print(f"Command sent: {command}")

                    if kill_signal:
                        print("Exiting write_to_arduino")
                        break
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")
                    break
                finally:
                    sleep(0.1)
            print("Exiting write_to_arduino")
        
        try:
            read_thread = threading.Thread(target=read_from_arduino, daemon=True)
            write_thread = threading.Thread(target=write_to_arduino, daemon=True)

            read_thread.start()
            write_thread.start()

            while not kill_signal:
                sleep(1)
                pass

            read_thread.join()
            write_thread.join()
            print("Exiting serial thread main loop")
        except Exception as e:
            print(f"An error occurred on serial thread main loop: {e}")
    except Exception as e:
        print(f"An error occurred on serial thread: {e}")

def main():
    global kill_signal
    try:
        gui_thread = threading.Thread(target=gui, daemon=True)
        gui_thread.start()

        serial_thread = threading.Thread(target=serial, daemon=True)
        serial_thread.start()

        while not kill_signal:
            sleep(1)
            pass

        print("Exiting main loop")
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, exiting")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        kill_signal = True
        gui_thread.join()
        serial_thread.join()
        print("Exiting...")

if __name__ == "__main__":
    main()
    