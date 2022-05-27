#!/usr/bin/env python3
import serial,time
import signal, sys

#def write_to_arduino(vL,vR):

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    arduino = serial.Serial('/dev/ttyACM6',115200,timeout=1.0)
    data_arduino = send_arduino_cmd_motor(arduino,0,0)
    sys.exit(0)


def init_arduino_line():
    arduino = serial.Serial('/dev/ttyACM6',9600,timeout=1.0)
    #arduino = serial.Serial('/dev/ttyACM1',115200,timeout=1.0)
    time.sleep(1.0) # wait for arduino serial line to me ready ...
    data = arduino.readline().decode('utf-8').rstrip()
    print ("init status",len(data),data)
    #arduino.close()
    signal.signal(signal.SIGINT, signal_handler)
    print(arduino)
    return arduino,data[0:-1]

def send_arduino_cmd_motor(arduino,cmdl0, cmdr0):   
    cmdl, cmdr = cmdl0, cmdr0
    if cmdl < 0:     
        cmdl = -cmdl
    if cmdr < 0:
        cmdr = -cmdr    
    strcmd = "{}${}\n".format(cmdl,cmdr)
    #strcmd = cmdl
    #print strcmd
    #arduino.open()
    #arduino.flushInput()
    #arduino.flushOutput()
    print("ca va?")
    arduino.write(strcmd.encode())
    #arduino.close()
    print("Hola")

def get_arduino_cmd_motor(arduino,timeout):    
    strcmd = "C\n"
    #print strcmd
    #arduino.open()
    arduino.write(strcmd.encode())
    t0 = time.time()
    while True:
        data = arduino.readline().decode('utf-8').rstrip()
        if data:
            #print data.rstrip('\n')
            break
        if (time.time()-t0) > timeout:
            break
    #arduino.close()
    #arduino.flushInput()
    return data[0:-1]

def get_arduino_cmd_motor_ones(arduino):    
    strcmd = "C\n"
    #print strcmd
    #arduino.open()

    arduino.write(strcmd.encode())
    data = arduino.readline().decode('utf-8').rstrip()
    #arduino.close()
    #arduino.flushInput()
    return data

if __name__ == '__main__':
    
    arduino, data = init_arduino_line()
    #c=0
    while True :
        #c+=1
        #if c ==10:
            #print("a7a")
            #send_arduino_cmd_motor(arduino, 0, 0 )
            #break
        #send_arduino_cmd_motor(arduino,cmd,cmd)
        send_arduino_cmd_motor(arduino, vL, vR )
        data = get_arduino_cmd_motor_ones(arduino)
        print(data)
        
        
