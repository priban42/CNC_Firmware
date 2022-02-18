import serial
import time
from PreParser import PreParser
from collections import deque

#ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)
ser1 = serial.Serial('COM5', 115200, timeout=.1)

vstup = ["ahoj"]

time.sleep(2)

defalutReturn = (69).to_bytes(1, byteorder='big')

bagr = bytearray()

size = 15
bagr.extend((size).to_bytes(1, byteorder='big'))
bagr.extend((69).to_bytes(size, byteorder='big'))

queue = deque()
bufferSize = 64
currentBuffer = 0

Path = "bagr2.gcode"

Parser = PreParser(Path)

start = time.time()

x = 0
max = 300

delay = 2

point1 = time.time()

bagr = Parser.getNext()

cas1 = 0

konec = False

while not konec:

    while currentBuffer + len(bagr) <= bufferSize:
        start1 = time.time()

        ser1.write(bagr)
        currentBuffer += len(bagr)
        queue.append(len(bagr))
        bagr = Parser.getNext()
        if not bagr:
            konec = True
            break

        end1 = time.time()
        cas1 += (end1 - start1)
    while True:
        if ser1.in_waiting:
            serVstup = ser1.read()
            #print(serVstup)
            if serVstup == defalutReturn:
                currentBuffer -= queue.popleft()
                x += 1
                break
end = time.time()
print (cas1)
#print((end - start)-(max*delay/1000))