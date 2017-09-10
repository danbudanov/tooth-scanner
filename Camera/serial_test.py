import serial

ser = serial.Serial('/dev/ttyACM0')
x = ser.read(6)
print x
ser.close()
