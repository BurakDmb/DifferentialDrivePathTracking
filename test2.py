import serial
import time
ser = serial.Serial('/dev/ttyACM0',9600)
#ser.flushInput()

while True:
    #s= ser.readline()
    #s= s.strip()
    #print (s.decode("utf-8"))
    #if (s.decode("utf-8") == "<Arduino is ready>"):
    print("sending")
    ans = 'hello Arduino"\n"'
    ans = ans.encode("utf-8")
    ser.write(ans)
    time.sleep(0.5)

