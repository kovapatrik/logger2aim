import serial
import csv
import time
import math

with serial.Serial('COM4', 9600) as ser:
    try:
        with open('test1.csv', 'w', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            while True:
                ser_bytes = str(ser.readline())[2:-5]
                #if ser_bytes:
                #    Vo = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
                #    print(Vo)
                #    writer.writerow([time.time(), Vo])
                data = ser_bytes.split(',')
                if data[0].isnumeric():
                    writer.writerow([time.time()] + [int(x) for x in data])
                else:
                    writer.writerow(['Time'] + data)
    except KeyboardInterrupt:
        pass
# 4.9999999873762135e-05
