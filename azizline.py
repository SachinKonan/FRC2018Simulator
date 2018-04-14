import cv2
import os
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))
angle = lambda x,y: (np.arctan2(y,-1*x)*180/np.pi)%360
imgname = 'resized_game.png'

img = cv2.imread(imgname)
print('Shape', img.shape)
imgcopy = img
coords = []
end = []
RED  = (0, 0, 255)
BLUE = (255,0,0 )
"""These should be in inches"""
real_robot_side = 33.5
real_robot_len = 38.5

robot_side = real_robot_side*2
robot_len = real_robot_len*2

base_angle = np.arctan((robot_len/2)/(robot_side/2))*180/np.pi
target_radius = np.sqrt((robot_side/2)**2 + (robot_len/2)**2)

def lineDrawer(img, x):
    if(len(x) == 1):
        return
    start = (x[-2][0], x[-2][1])
    end = (x[-1][0], x[-1][1])
    cv2.line(img, start, end, RED, 1)
    x.pop(-1)
    lineDrawer(img, x)

def recurseRobotInstructions(coords, robotcoords = [], robot_scaledcoords = []):
    if(len(coords) == 1):
        return robotcoords, robot_scaledcoords
    start = coords[0]
    end = coords[1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    pot_angle = angle(dy,dx)
    distance = np.sqrt((dx)**2 + (dy)**2)
    robotcoords.append('T%s' %(round(pot_angle,2)))
    robotcoords.append('F%s' %(round(distance,2)))

    sdx = dx/2
    sdy = dy/2
    sdistance = np.sqrt((sdx)**2 + (sdy)**2)
    robot_scaledcoords.append('T%s' %(round(pot_angle,2)))
    robot_scaledcoords.append('F%s' %(round(sdistance,2)))
    coords.pop(0)
    return recurseRobotInstructions(coords, robotcoords)

def eventHandler(event, x, y, k, s):
    global imgcopy, img, coords
    imgcopy = img.copy()
    if(event == cv2.EVENT_LBUTTONDOWN):
        if(len(coords) == 0):
            coords.append([x,y])
            coords.append([x,y])
        else:
            coords.append([x,y])
    elif(event == cv2.EVENT_MOUSEMOVE):
        if(len(coords) >= 2):
            coords[-1] = [x,y]
        else:
            pass
    else:
        pass

    x = coords.copy()
    while(len(x) > 1):
        start = (x[-2][0], x[-2][1])
        end = (x[-1][0], x[-1][1])
        cv2.line(imgcopy, start, start, BLUE, 5)
        cv2.line(imgcopy, end, end, BLUE, 5)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        pot_angle = angle(dy,dx)
        angle1 = base_angle - pot_angle
        angle2 = base_angle + pot_angle
        start_bottom = (start[0] - int(target_radius*np.cos(np.pi*angle1/180)), start[1] + int(target_radius*np.sin(np.pi*angle1/180)))
        start_top = (end[0]  - int(target_radius*np.cos(np.pi*angle2/180)), end[1] - int(target_radius*np.sin(np.pi*angle2/180)))

        end_bottom = (start[0]  + int(target_radius*np.cos(np.pi*angle2/180)), start[1] + int(target_radius*np.sin(np.pi*angle2/180)))
        end_top = (end[0] + int(target_radius*np.cos(np.pi*angle1/180)), end[1]- int(target_radius*np.sin(np.pi*angle1/180)))

        cv2.line(imgcopy, start_bottom, start_top ,BLUE, 1 )
        cv2.line(imgcopy, end_bottom, end_top ,BLUE, 1 )
        cv2.line(imgcopy, start_bottom, end_bottom ,BLUE, 1 )
        cv2.line(imgcopy, start_top, end_top ,BLUE, 1 )
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
