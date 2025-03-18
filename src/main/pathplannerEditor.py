import math
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

redToBlue = {"10": "21", "9": "22", "8": "17", "7": "18", "6": "19", "11": "20", "10alt": "21alt", "9alt": "22alt", "8alt": "17alt", "7alt": "18alt", "6alt": "19alt", "11alt": "20alt"}
blueToRed = {v: k for k, v in redToBlue.items()}

def conversion(tag, offCenter):
    x,y,angle = tag[0], tag[1], tag[2]
    xOffset = 0.65
    yOffset = -0.2 if offCenter else 0.1
    angleOffset = math.atan(yOffset/xOffset)
    dist = math.sqrt(xOffset**2 + yOffset**2)
    inToM = 0.0254
    degToRad = math.pi/180
    xPos = x*inToM + dist*math.cos(angle*degToRad+angleOffset)
    yPos = y*inToM + dist*math.sin(angle*degToRad+angleOffset)
    return round(xPos,2),round(yPos,2)

bluePoses = {}
bluePosesAlt = {}

def printTags(color, name, tags):
    print(color + " " + name)
    for item in tags:
        xPos, yPos = conversion(tags[item], False)
        xPosa, yPosa = conversion(tags[item], True)
        bluePoses[str(item)] =(xPos,yPos)
        bluePoses[str(item)+"alt"] =(xPosa,yPosa)
        print(str(item) + "/" + blueToRed[str(item)]+ ": " + str(xPos) + ", " + str(yPos) + " | " + str(xPosa) + ", " + str(yPosa))
    print()

print()
printTags("Blue", "Reef", blueReefTags)

# with open("src\main\deploy\pathplanner\paths") as folder:
#     for file in folder:
#         data = file.read()
#         d = json.loads(data)
#         for point in d["waypoints"]:
#             point["anchor"]["x"] = d["waypoints"][]
#         with open("src\main\deploy\pathplanner\paths", 'w') as f:
#             f.write(json.dumps(d))



#printTags("Red", "Reef", redReefTags)