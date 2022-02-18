from PreParser import PreParser
from collections import deque
import serial
import time

class SerialSender:
    def __init__(self, Path, device = 'COM6'):
        self.Serial = serial.Serial(device, 115200, timeout=.1)
        time.sleep(2)
        self.Parser = PreParser()
        self.Parser.initFile(Path)
        self.BufferSize = 64
        self.BufferQueue = deque()
        self.BufferFilled = 0
        self.OutputLog = deque()

        self.nextCommand = 0
        self.nextCommandSize = 0
        self.lastCommandSent = True

        self.commandsSent = 0

        self.LogSettings = {"sentLines":True,
                            "sentParsedCommands":False,
                            "recievedResponse":True,
                            "requestedInfo":False}

    def sendNext(self):
        if self.lastCommandSent:
            self.nextCommand = self.Parser.getNext()
            if not self.nextCommand:
                while not self.__clearBuffer():
                    pass
                return False
            self.lastCommandSent = False

        self.nextCommandSize = len(self.nextCommand)

        if self.BufferFilled + self.nextCommandSize <= self.BufferSize:
            self.Serial.write(self.nextCommand)
            self.BufferQueue.append(self.nextCommandSize)
            self.BufferFilled += self.nextCommandSize
            self.lastCommandSent = True
            self.commandsSent += 1
            if self.LogSettings["sentParsedCommands"]:
                self.OutputLog.append(str(self.nextCommand) + " " + str(len(self.nextCommand)))

        self.__clearBuffer()

        if self.LogSettings["sentLines"]:
            line = self.Parser.getCurrentLine()
            if not line:
                pass
            else:
                self.OutputLog.append(line.rstrip())
        return True

    def getNextLog(self):
        self.__clearBuffer()
        if len(self.OutputLog) == 0:
            return False
        return self.OutputLog.popleft()

    def __clearBuffer(self):
        if self.Serial.in_waiting:
            serVstup = self.Serial.read()
            if serVstup == (69).to_bytes(1, byteorder='big'):
                self.BufferFilled -= self.BufferQueue.popleft()
                return True
            elif serVstup == (42).to_bytes(1, byteorder='big'):
                response = self.Serial.readline().decode('UTF-8').rstrip()
                if self.LogSettings["recievedResponse"]:
                    self.OutputLog.append(response)
        return False

    def insertCommand(self, line):
        self.Parser.insertLine(line)
        if self.LogSettings["sentLines"]:
            if not line:
                pass
            else:
                self.OutputLog.append(line.rstrip())
        return True

    @staticmethod
    def twoBytesToInt(byte1, byte2):
        return (256*byte1+byte2)

    def requestInfo(self):
        settings = {}
        self.Serial.write((29).to_bytes(1, byteorder='big'))
        while not self.Serial.in_waiting:
            pass
        while True:
            line = self.Serial.readline().decode('UTF-8').rstrip()
            value = int.from_bytes(self.Serial.read(2), "big")
            if "spmm" in line:
                value /= 10;
            if line == "end":
                break
            settings[line] = value
            if self.LogSettings["requestedInfo"]:
                self.OutputLog.append(line + ": " + str(value))
        self.Parser.applySettings(settings)
        #return settings