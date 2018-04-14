import numpy as np
import pygame
import sys
import os
from collisionutils import *
from colors import *
from utilutils import *
from chassis import AutoPathFollower, RobotDrive
import initObstacles

os.chdir(os.path.dirname(os.path.realpath(__file__)))

size = (1340, 684)

def allDisplay(obstacles, screen):
    displayObstacles(obstacles, screen, color_matter = True)
    pygame.draw.rect(screen, obstacles['Switch']['RedSwitchTop'][0], obstacles['Switch']['RedSwitchTop'][1:])
    pygame.draw.rect(screen, obstacles['Switch']['RedSwitchBottom'][0], obstacles['Switch']['RedSwitchBottom'][1:])
    pygame.draw.rect(screen, obstacles['Switch']['BlueSwitchTop'][0], obstacles['Switch']['BlueSwitchTop'][1:])
    pygame.draw.rect(screen, obstacles['Switch']['BlueSwitchBottom'][0], obstacles['Switch']['BlueSwitchBottom'][1:])

def displayObstacles(obstacles, screen, color_matter = True):
    for i in obstacles.keys():
        if(isinstance(obstacles[i], dict) and i is 'Cube'):
            displayObstacles(obstacles[i], screen, False)
        elif(isinstance(obstacles[i], dict)):
            displayObstacles(obstacles[i], screen, True)
        else:
            if(not('Top' in i) and not('Bottom' in i) ):
                if(color_matter):
                    pygame.draw.rect(screen, obstacles[i][0], obstacles[i][1:])
                else:
                    pygame.draw.polygon(screen, YELLOW, obstacles[i][0])

def distance_calculator(robot_mid, target_mid):
    return np.sqrt((target_mid[0]- robot_mid[0])**2 +  (target_mid[1]- robot_mid[1])**2)

def instant_distancetoObstacles(robot_mid, obstacles, threshold):
    distances = list(map(lambda x: [x, distance_calculator(robot_mid, getMidpoint(obstacles['Switch'][x][1:]))] ,obstacles['Switch'].keys())) +  list(map(lambda x: [x, distance_calculator(robot_mid, getMidpoint(obstacles['Scale'][x][1:]))] ,obstacles['Scale'].keys()))
    s = sorted(distances, key = lambda x: x[1])
    id = s[0][0]
    return obstacles['Switch' if 'Switch' in id else 'Scale'][id]

def distancetoScaleInput(gripper_mid, threshold):
    return min(distance_calculator(gripper_mid, getMidpoint(obstacles['Scale']['Scale_Input1'][1:] )), distance_calculator(gripper_mid,  getMidpoint(obstacles['Scale']['Scale_Input2'][1:])) )  < threshold


def instant_distancetoCubes(robot_mid, obstacles, threshold):
    distances = list(map(lambda x: [x, distance_calculator(robot_mid, obstacles['Cube'][x][1])] ,obstacles['Cube'].keys()))
    s = sorted(distances, key = lambda x: x[1])
    return obstacles['Cube'][s[0][0]], s[0][0]

def drawRobottoNearestCube(screen, robot_gripper_mid, min_cube):
    pygame.draw.line(screen, BLACK, robot_gripper_mid, min_cube)

def checkObstacles(coords, obstacles):
    k = ['Switch', 'Scale']
    for i in k:
        for j in obstacles[i].keys():
            collided = collide(coords, getPolygon(obstacles[i][j][1:]) )
            if(collided):
                return False, j
    return True, None

def showPath(screen, coords):
    if(len(coords) == 1):
        return
    pygame.draw.line(screen, BLUE, coords[0], coords[1])
    coords.pop(0)
    return showPath(screen, coords)

pygame.init()
screen = pygame.display.set_mode(size)
screen.fill(WHITE)
clock = pygame.time.Clock()
inc = 10
height_inc = 10

start = initObstacles.start
tasks = ['T90.0', 'F306.0', 'T180.0', 'F43.0']
path =  [[40, 95], [346, 95], [346, 138], [346, 138]]
real_start = [path[0][0], path[0][1], start[2], start[3]]
chassis = RobotDrive(x_mid = real_start[0], y_mid=real_start[1], w= real_start[2], h = real_start[3], Gripper = True, startCube = True)
#chassis.rotate(90)
auto = AutoPathFollower(chassis, screen, tasks)
done = False

while(not done):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    obstacles = initObstacles.obstacles
    screen.fill(WHITE)
    bool = auto.autoPeriodic()
    keys=pygame.key.get_pressed()


    if(bool):
        angle = chassis.angle
        if keys[pygame.K_RIGHT]:
            angle += inc
        if keys[pygame.K_LEFT]:
            angle += -1*inc
        if keys[pygame.K_UP]:
            chassis.translate(height_inc)
            coords = chassis.getTotalRobotCoordinates()
            collided, id = checkObstacles(coords, obstacles)
            if(collided and not checkOutofBounds(coords, size) ):
                angle += 1.5*(2*np.random.rand() - 1)
            else:
                if(id in ['RedSwitchTop', 'RedSwitchBottom', 'BlueSwitchTop', 'BlueSwitchBottom','Scale_Input1', 'Scale_Input2'] and chassis.target_attached):
                    print('You scored!!!!!')
                    id = chassis.target
                    chassis.detachTarget()
                    obstacles['Cube'].pop(id, None)
                else:
                    print(False)
                chassis.translate(-height_inc*2)

        if keys[pygame.K_DOWN]:
            chassis.translate(-1*height_inc)
            coords = chassis.getTotalRobotCoordinates()
            collided, id = checkObstacles(coords, obstacles)
            if(collided and not checkOutofBounds(coords, size) ):
                angle += 1.5*(2*np.random.rand() - 1)
            else:
                if(id in ['RedSwitchTop', 'RedSwitchBottom', 'BlueSwitchTop', 'BlueSwitchBottom','Scale_Input1', 'Scale_Input2'] and chassis.target_attached):
                    print('You scored!!!!!')
                    id = chassis.target
                    chassis.detachTarget()
                    obstacles['Cube'].pop(id, None)
                else:
                    print(False)
                chassis.translate(height_inc)
        chassis.rotate(angle)
    else:
        showPath(screen, path.copy())

    if keys[pygame.K_d] and chassis.target_attached:
        chassis.detachTarget()

    min_cube, id = instant_distancetoCubes(chassis.robotSprite, obstacles, chassis.getTotalLen())
    #print(id)
    if(distance_calculator(chassis.robotSprite, min_cube[1]) < 50 and chassis.target_attached == False):
        chassis.setTarget(id)

    #print(id)
    chassis.drawRobot(screen)
    allDisplay(obstacles, screen)
    drawRobottoNearestCube(screen, chassis.gripper_center, min_cube[1])
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
