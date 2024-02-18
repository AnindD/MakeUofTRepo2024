import threading
import time 
import keyboard 
import serial 

globalInput = ""
temporaryString = ""
lock = threading.Lock()

def read_user_input(key1, key2):
    global temporaryString
    
    try:
        num = 0 
        if keyboard.is_pressed(key1):
            with lock:
                num += 1   
        elif keyboard.is_pressed(key2):
            with lock:
                num -= 1
        if num == - 1: 
            num = 2 
        temporaryString += str(num)
    except:
        pass
        

def send_bluetooth_information():
    global globalInput
    ser = serial.Serial(
        port='COM6',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.SEVENBITS
    )
    while True:
        print(f"SENDING: {globalInput}")
        ser.write((globalInput).encode("utf-8"))  
        time.sleep(1/50)  
    
def main():
    global globalInput, temporaryString
    bluetooth_thread = threading.Thread(target=send_bluetooth_information)

    bluetooth_thread.start()

    while True:
        read_user_input('up', 'down')
        read_user_input('right', 'left')
        read_user_input('e', 'q')
        read_user_input('w', 's')
        
        globalInput = temporaryString
        temporaryString = ""

main()