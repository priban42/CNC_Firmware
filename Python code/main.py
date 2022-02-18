from SerialSender import SerialSender
import time
import timeit

#path = "Incredible Fyyran.gcode"
path = "remove filament.gcode"

#path = "circle mill test.nc"

Sender = SerialSender(path)

numberLine = 0

print_start = time.time()

Sender.requestInfo()
#Sender.insertCommand("G1 X10 Y10 E10")
while Sender.sendNext():
    while True:
        logLine = Sender.getNextLog()
        if not logLine:
            break
        print(str(numberLine) + ":", logLine)
        numberLine += 1

start = time.time()

while True:
    logLine = Sender.getNextLog()
    if not logLine:
        pass
    else:
        start = time.time()
        print(str(numberLine) + ":", logLine)
        numberLine += 1
    if ((time.time() - start) > 5):
        break

print_end = time.time()
print(str(int((print_end - print_start)/3600)) + ":" + str(int(((print_end - print_start)%3600)/60)))