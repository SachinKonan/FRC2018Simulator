import cv2
import os
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))
angle = lambda x,y: (np.arctan2(y,-1*x)*180/np.pi)%360
imgname = 'resized_game.png'

img = cv2.imread(imgname)
print('Shape', img.shape)
imgcopy = img
coords = [[15, 313], [15, 313]]
end = []
RED  = (0, 0, 255)
BLUE = (255,0,0 )
def lineDrawer(img, x):
    if(len(x) == 1):
        return
    start = (x[-2][0], x[-2][1])
    end = (x[-1][0], x[-1][1])
    cv2.line(img, start, end, RED, 1)
    x.pop(-1)
    lineDrawer(img, x)

def recurseRobotInstructions(coords, robotcoords = []):
    if(len(coords) == 1):
        return robotcoords
    start = coords[0]
    end = coords[1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    pot_angle = angle(dy,dx)
    distance = np.sqrt((dx)**2 + (dy)**2)
    robotcoords.append('T%s' %(int(pot_angle)))
    robotcoords.append('F%s' %(int(distance)))
    coords.pop(0)
    return recurseRobotInstructions(coords, robotcoords)

def eventHandler(event, x, y, k, s):
    global imgcopy, img, coords
    imgcopy = img.copy()
    if(event == cv2.EVENT_LBUTTONDOWN):
        coords.append([x,y])
    elif(event == cv2.EVENT_MOUSEMOVE):
        coords[-1] = [x,y]
    else:
        pass

    x = coords.copy()
    while(len(x) > 1):
        start = (x[-2][0], x[-2][1])
        end = (x[-1][0], x[-1][1])
        cv2.line(imgcopy, start, start, BLUE, 5)
        cv2.line(imgcopy, end, end, BLUE, 5)
        cv2.line(imgcopy, start, end, RED, 1)
        x.pop(-1)

cv2.namedWindow("img")
cv2.setMouseCallback("img", eventHandler)

while True:
    cv2.imshow("img", imgcopy)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if(key == ord("y")):
        print(coords)
        print(recurseRobotInstructions(coords[0:-1]))
        break
    if key == ord("q"):
        break
cv2.destroyAllWindows()
