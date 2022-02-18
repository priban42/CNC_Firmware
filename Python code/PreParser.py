import re
from collections import deque
import math

no_Arg_G_Commands = {28: 17,
                     21: 18,
                     90: 19, }

no_Arg_M_Commands = {107: 22,
                     82: 27,
                     84: 28}

single_Arg_G_Commands = {92: 20}

single_Arg_M_Commands = {106: 21,
                         140: 23,
                         190: 24,
                         104: 25,
                         109: 26}

class PreParser:
    def __init__(self):
        self.file_ready = False

        self.settings = {"byteX": 2,
                         "byteY": 2,
                         "byteZ": 3,
                         "byteE": 3,
                         "byteF": 3,
                         "byteC": 1,
                         "byteA": 2,
                         "spmmX": 145.5,
                         "spmmY": 145.5,
                         "spmmZ": 1600,
                         "spmmE": 872.7,
                         "maxdX": 250,
                         "maxdY": 250,
                         "maxdZ": 200,
                         "maxsX": 200,
                         "maxsY": 200,
                         "maxsZ": 200, }

        self.byteSizes = {"X": self.settings["byteX"],
                          "Y": self.settings["byteY"],
                          "Z": self.settings["byteZ"],
                          "E": self.settings["byteE"],
                          "F": self.settings["byteF"],
                          "command": self.settings["byteC"],
                          "generalArg": self.settings["byteA"]}

        self.movement_type = 0# 1 = coreXY

        self.baseSizes = {"X": int((2 ** (8 * self.byteSizes["X"] - 1))),
                          "Y": int((2 ** (8 * self.byteSizes["Y"] - 1))),
                          "Z": int((2 ** (8 * self.byteSizes["Z"] - 1))),
                          "E": int((2 ** (8 * self.byteSizes["E"] - 1)))}

        self.stepsForKeys = {"X": 10.6,
                             "Y": 10.6,
                             "Z": 800.0,
                             "E": 26.58}

        self.smart_acceleration = False

        self.no_acc_max_angle = 16
        self.is_accelerated = False

        self.past_X = 0
        self.past_Y = 0
        self.current_X = 0
        self.current_Y = 0
        self.next_X = 0
        self.next_Y = 0

    def initFile(self, Path):
        self.Path = Path
        self.GcodeFile = open(Path, 'r')
        self.Queue = deque()

        self.currentLine = ""
        self.currentLineReturned = False

        self.file_ready = True

    def applySettings(self, NewSettings):
        for x in NewSettings:
            self.settings[x] = NewSettings[x]

        self.byteSizes = {"X": self.settings["byteX"],
                          "Y": self.settings["byteY"],
                          "Z": self.settings["byteZ"],
                          "E": self.settings["byteE"],
                          "F": self.settings["byteF"],
                          "command": self.settings["byteC"],
                          "generalArg": self.settings["byteA"]}

        self.stepsForKeys = {"X": self.settings["spmmX"],
                             "Y": self.settings["spmmY"],
                             "Z": self.settings["spmmZ"],
                             "E": self.settings["spmmE"]}

        self.movement_type = self.settings["coreXY"]
        print (self.movement_type)

    def getNext(self):
        if len(self.Queue) == 0:
            if not self.file_ready:
                return False
            while True:
                self.currentLine = self.GcodeFile.readline()
                self.currentLineReturned = False
                if not self.currentLine:
                    file_ready = False
                    return False
                elif self.parseLine(self.currentLine) == False:
                    pass
                else:
                    break
        return self.Queue.popleft()

    def insertLine(self, line):
        # self.currentLine = line
        # self.currentLineReturned = False
        # self
        self.parseLine(line)

    def getCurrentLine(self):
        if not self.currentLineReturned:
            self.currentLineReturned = True
            return self.currentLine
        else:
            return False

    def parseLine(self, line):
        if not line:
            return False
        elif line[0] == ";":
            return False
        elif line[0] == "\n":
            return False
        elif line[0] == "G":
            value = int(self.getKeyValue("G", line))
            if value == 1:
                self.parseG1_new(line)
            elif value in no_Arg_G_Commands:  # G command no argument
                self.parseNoArg(no_Arg_G_Commands[value])
            elif value == 92:
                self.parseNoArg(20)
        elif line[0] == "M":
            value = int(self.getKeyValue("M", line))
            if value in no_Arg_M_Commands:  # M command no argument
                self.parseNoArg(no_Arg_M_Commands[value])
            if value in single_Arg_M_Commands:  # M command with argument
                self.parseSingleArg(single_Arg_M_Commands[value], int(self.getKeyValue("S", line)))
        else:
            return False

    @staticmethod
    def getKeyValue(key, string):
        vraceni = re.search(key + "([-0-9.]*)", string).group(1)
        return float(vraceni)

    @staticmethod
    def rotateCoord(key, X, Y):
        if key == "X":
            return(0.70710678118*(X - Y))
        else:
            return (0.70710678118 * (X + Y))

    def parseSingleArg(self, index, argValue):
        vraceni = bytearray()
        vraceni.extend(index.to_bytes(self.byteSizes["command"], byteorder='big'))
        vraceni.extend(argValue.to_bytes(self.byteSizes["generalArg"], byteorder='big'))
        self.Queue.append(vraceni)

    def parseNoArg(self, index):
        vraceni = bytearray()
        vraceni.extend(index.to_bytes(self.byteSizes["command"], byteorder='big'))
        self.Queue.append(vraceni)

    def getPackedCoordinates(self, index, keys, string):
        vraceni = bytearray()
        vraceni.extend(index.to_bytes(self.byteSizes["command"], byteorder='big'))
        if self.movement_type == 1:
            X_value = 0
            Y_value = 0
            if "X" in keys:
                X_value += self.getKeyValue("X", string) * self.stepsForKeys["X"]
            if "Y" in keys:
                Y_value += self.getKeyValue("Y", string) * self.stepsForKeys["Y"]
            for key in keys:
                if key in ["X", "Y"]:
                    value = round(self.rotateCoord(key, X_value, Y_value) + self.baseSizes[key])
                else:
                    value = round(self.baseSizes[key] + self.getKeyValue(key, string) * self.stepsForKeys[key])
                vraceni.extend(value.to_bytes(self.byteSizes[key], byteorder='big'))
            return vraceni
        for key in keys:
            #    value = round(rotateCoord(self.baseSizes[key] + self.getKeyValue(key, string) * self.stepsForKeys[key]))
            value = round(self.baseSizes[key] + self.getKeyValue(key, string) * self.stepsForKeys[key])
            vraceni.extend(value.to_bytes(self.byteSizes[key], byteorder='big'))
        return vraceni

    def update_XYE_coordinates(self, line):
        self.past_X = self.current_X
        self.past_Y = self.current_Y
        if ("X" in line):
            self.current_X = round(self.baseSizes["X"] + self.getKeyValue("X", line) * self.stepsForKeys["X"])
        if ("Y" in line):
            self.current_Y = round(self.baseSizes["Y"] + self.getKeyValue("Y", line) * self.stepsForKeys["Y"])
        if not self.findNextXYE_coordinates():
            self.next_X = self.current_X
            self.next_Y = self.current_Y

    def findNextXYE_coordinates(self):
        current_line_number = self.GcodeFile.tell()
        for x in range(5):
            line = self.GcodeFile.readline()
            if len(line) == 0:
                self.GcodeFile.seek(current_line_number)
                return False
            if line[0] in [";", "#", "\n"]:
                self.GcodeFile.seek(current_line_number)
                return False
            if line == False:
                self.GcodeFile.seek(current_line_number)
                return False
            if all(x in line for x in ["G1", "X", "Y", "E"]):
                if self.movement_type == 1:
                    self.next_X = round(self.baseSizes["X"] + self.getKeyValue("X", line) * self.stepsForKeys["X"])
                    self.next_Y = round(self.baseSizes["Y"] + self.getKeyValue("Y", line) * self.stepsForKeys["Y"])
                self.next_X = round(self.baseSizes["X"] + self.getKeyValue("X", line) * self.stepsForKeys["X"])
                self.next_Y = round(self.baseSizes["Y"] + self.getKeyValue("Y", line) * self.stepsForKeys["Y"])
                self.GcodeFile.seek(current_line_number)
                return True
            elif any(x in line for x in ["G1"]):
                self.GcodeFile.seek(current_line_number)
                return False
        self.GcodeFile.seek(current_line_number)
        return False

    @staticmethod
    def getAngleXYE(fist_point, second_point, third_point):
        vector1 = [second_point[0] - fist_point[0], second_point[1] - fist_point[1]]
        vector2 = [third_point[0] - second_point[0], third_point[1] - second_point[1]]
        cosA = round((vector1[0] * vector2[0] + vector1[1] * vector2[1]) / (
                    ((vector1[0] * vector1[0] + vector1[1] * vector1[1]) ** (1 / 2)) * (
                        (vector2[0] * vector2[0] + vector2[1] * vector2[1]) ** (1 / 2))), 5)
        print(round(math.degrees(math.acos(cosA)), 2))
        return round(math.degrees(math.acos(cosA)), 2)

    def parseG1_new(self, line):

        if "F" in line:
            F_value = self.getKeyValue("F", line)
            F_value = int(F_value)
            vraceni = bytearray()
            vraceni.extend((2).to_bytes(self.byteSizes["command"], byteorder='big'))
            vraceni.extend(F_value.to_bytes(self.byteSizes["F"], byteorder='big'))
            self.Queue.append(vraceni)

        self.update_XYE_coordinates(line)

        if "X" in line:
            if "Y" in line:
                if "E" in line:
                    if "Z" in line:
                        self.Queue.append(self.getPackedCoordinates(9, ["X", "Y", "Z", "E"], line))
                        return
                    else:
                        if self.smart_acceleration:
                            if (self.next_X - self.current_X == 0 and self.next_Y - self.current_Y == 0):
                                if self.is_accelerated:
                                    self.Queue.append(self.getPackedCoordinates(34, ["X", "Y", "E"], line))  # end acc
                                    self.is_accelerated = False
                                    print("end acc")
                                else:
                                    self.Queue.append(self.getPackedCoordinates(1, ["X", "Y", "E"], line))  # double acc
                                    print("double acc")
                            else:
                                if (self.getAngleXYE([self.past_X, self.past_Y], [self.current_X, self.current_Y],
                                                     [self.next_X, self.next_Y]) < self.no_acc_max_angle):

                                    if self.is_accelerated:
                                        self.Queue.append(
                                            self.getPackedCoordinates(33, ["X", "Y", "E"], line))  # no acc
                                        print("no acc")
                                    else:
                                        if (((self.past_X - self.current_X) ** 2 + (
                                                self.past_Y - self.current_Y) ** 2) ** (1 / 2)) > 0.2:
                                            self.Queue.append(
                                                self.getPackedCoordinates(32, ["X", "Y", "E"], line))  # start acc
                                            self.is_accelerated = True
                                            print("start acc2")
                                        else:
                                            self.Queue.append(self.getPackedCoordinates(1, ["X", "Y", "E"], line))
                                else:
                                    if self.is_accelerated:
                                        self.Queue.append(
                                            self.getPackedCoordinates(34, ["X", "Y", "E"], line))  # end acc
                                        print("end acc2")
                                        self.is_accelerated = False
                                    else:
                                        self.Queue.append(
                                            self.getPackedCoordinates(1, ["X", "Y", "E"], line))  # double acc
                                        print("double acc2")
                        else:
                            self.Queue.append(self.getPackedCoordinates(1, ["X", "Y", "E"], line))
                        return
                elif "Z" in line:
                    self.Queue.append(self.getPackedCoordinates(6, ["X", "Y", "Z"], line))  ###################
                    return
                else:
                    self.Queue.append(self.getPackedCoordinates(5, ["X", "Y"], line))
                    return
            elif "E" in line:
                if "Z" in line:
                    self.Queue.append(self.getPackedCoordinates(15, ["X", "Z", "E"], line))
                    return
                else:
                    self.Queue.append(self.getPackedCoordinates(10, ["X", "E"], line))
                    return
            elif "Z" in line:
                self.Queue.append(self.getPackedCoordinates(13, ["X", "Z"], line))
                return
            else:
                self.Queue.append(self.getPackedCoordinates(7, ["X"], line))
                return
        elif "Y" in line:
            if "E" in line:
                if "Z" in line:
                    self.Queue.append(self.getPackedCoordinates(16, ["Y", "Z", "E"], line))
                    return
                else:
                    self.Queue.append(self.getPackedCoordinates(11, ["Y", "E"], line))
                    return
            elif "Z" in line:
                self.Queue.append(self.getPackedCoordinates(14, ["Y", "Z"], line))
                return
            else:
                self.Queue.append(self.getPackedCoordinates(8, ["Y"], line))
                return
        elif "Z" in line:
            if "E" in line:
                self.Queue.append(self.getPackedCoordinates(12, ["Z", "E"], line))
                return
            else:
                self.Queue.append(self.getPackedCoordinates(4, ["Z"], line))
                return
        elif "E" in line:
            self.Queue.append(self.getPackedCoordinates(3, ["E"], line))
            return


if __name__ == "__main__":
    Parser = PreParser()
    Parser.initFile("remove filament.gcode")
    next = Parser.getNext()

    while next:
        print(Parser.getCurrentLine())
        next = Parser.getNext()
