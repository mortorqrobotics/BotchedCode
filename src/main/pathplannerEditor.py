import math
import os
from pathlib import Path
import json

blueReefTags = {
    17:(160.39,130.17,240),
    18:(144,158.5,180),
    19:(160.39,186.83,120),
    20:(193.1,186.83,60),
    21:(209.49,158.5,0),
    22:(193.1,130.17,300)
}
redReefTags = {
    6:(530.49,130.17,300),
    7:(546.87,158.5,0),
    8:(530.49,186.83,60),
    9:(497.77,186.83,120),
    10:(481.39,158.5,180),
    11:(497.77,130.17,240),
}
blueStationTags = {
    12:(33.51, 25.8, 54),
    13:(33.51, 291.2, 306)
}
redStationTags = {
    1:(657.37, 25.8, 126),
    2:(657.37, 291.2, 234)
}

redToBlue = {"10": "21", "9": "22", "8": "17", "7": "18", "6": "19", "11": "20", "10alt": "21alt", "9alt": "22alt", "8alt": "17alt", "7alt": "18alt", "6alt": "19alt", "11alt": "20alt", "1":"13", "2": "12", "1alt":"13alt", "2alt":"12alt"}
blueToRed = {v: k for k, v in redToBlue.items()}

def conversion(tag, offCenter):
    x,y,angle = tag[0], tag[1], tag[2]
    xOffset = 0.61
    dist = 0.33
    yOffset = -dist/2 if offCenter else dist/2
    angleOffset = math.atan(yOffset/xOffset)
    dist = math.sqrt(xOffset**2 + yOffset**2)
    inToM = 0.0254
    degToRad = math.pi/180
    xPos = x*inToM + dist*math.cos(angle*degToRad+angleOffset)
    yPos = y*inToM + dist*math.sin(angle*degToRad+angleOffset)
    return round(xPos,2),round(yPos,2)

def stConversion(tag, offCenter):
    x,y,angle = tag[0], tag[1], tag[2]
    xOffset = 0.4
    yOffset = -0.03
    angleOffset = math.atan(yOffset/xOffset)
    dist = math.sqrt(xOffset**2 + yOffset**2)
    inToM = 0.0254
    degToRad = math.pi/180
    xPos = x*inToM + dist*math.cos(angle*degToRad+angleOffset)
    yPos = y*inToM + dist*math.sin(angle*degToRad+angleOffset)
    return round(xPos,2),round(yPos,2)

bluePoses = {}

def printTags(color, name, tags):
    print(color + " " + name)
    for item in tags:
        xPos, yPos = conversion(tags[item], False)
        xPosa, yPosa = conversion(tags[item], True)
        bluePoses[str(item)] =(xPos,yPos)
        bluePoses[str(item)+"alt"] =(xPosa,yPosa)
        print(str(item) + "/" + blueToRed[str(item)]+ ": " + str(xPos) + ", " + str(yPos) + " | " + str(xPosa) + ", " + str(yPosa))
    print()

def printTagsSt(color, name, tags):
    print(color + " " + name)
    for item in tags:
        xPos, yPos = stConversion(tags[item], False)
        xPosa, yPosa = stConversion(tags[item], True)
        bluePoses[str(item)] =(xPos,yPos)
        bluePoses[str(item)+"alt"] =(xPosa,yPosa)
        print(str(item) + "/" + blueToRed[str(item)]+ ": " + str(xPos) + ", " + str(yPos) + " | " + str(xPosa) + ", " + str(yPosa))
    print()

print()
printTags("Blue", "Reef", blueReefTags)
printTagsSt("Blue", "Station", blueStationTags)
#printTags("Red", "Reef", redReefTags)

pathsFolder = str(Path.cwd()) +"\\src\\main\\deploy\\pathplanner\\paths"

for path in os.scandir(pathsFolder):  
    data = 0
    #print(path.path)
    with open(path.path, "r") as f:
        data = json.loads(f.read())
        for element in data["waypoints"]:
            if (element["linkedName"] in redToBlue):
                element["anchor"]["x"] = float(bluePoses[redToBlue[str(element["linkedName"])]][0])
                element["anchor"]["y"] = float(bluePoses[redToBlue[str(element["linkedName"])]][1])
    with open(path.path, 'w') as f:
        f.write(json.dumps(data, indent = 2))