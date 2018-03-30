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

def displayObstacles(obstacles, screen, color_matter = True):
    for i in obstacles.keys():
        if(isinstance(obstacles[i], dict) and i is 'Cube'):
            displayObstacles(obstacles[i], screen, False)
        elif(isinstance(obstacles[i], dict)):
            displayObstacles(obstacles[i], screen, True)
        else:
            if(color_matter):
                pygame.draw.rect(screen, obstacles[i][0], obstacles[i][1:])
            else:
                pygame.draw.polygon(screen, YELLOW, obstacles[i][0])

def distance_calculator(robot_mid, target_mid):
    return np.sqrt((target_mid[0]- robot_mid[0])**2 +  (target_mid[1]- robot_mid[1])**2)

def instant_distancetoObstacles(robot, obstacles, threshold):
    robot_mid = robot.robotSprite
    distances = list(map(lambda x: [x, distance_calculator(robot_mid, getMidpoint(obstacles['Switch'][x][1:]))] ,obstacles['Switch'].keys())) +  list(map(lambda x: [x, distance_calculator(robot_mid, getMidpoint(obstacles['Scale'][x][1:]))] ,obstacles['Scale'].keys()))
    s = sorted(distances, key = lambda x: x[1])
    id = s[0][0]
    return obstacles['Switch' if 'Switch' in id else 'Scale'][id]

def instant_distancetoCubes(robot, obstacles, threshold):
    robot_mid = robot.robotSprite
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
                return False
    return True

pygame.init()
screen = pygame.display.set_mode(size)
screen.fill(WHITE)
clock = pygame.time.Clock()
inc = 10
height_inc = 10

start = initObstacles.start

chassis = RobotDrive(x_mid = start[0], y_mid=start[1], w= start[2], h = start[3], Gripper = True)
chassis.angle = 90
auto = AutoPathFollower(chassis, screen)
done = False

while(not done):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    obstacles = initObstacles.obstacles
    screen.fill(WHITE)
    bool = auto.autoPeriodic()
    if(bool):
        keys=pygame.key.get_pressed()
        angle = chassis.angle
        if keys[pygame.K_RIGHT]:
            angle += inc
        if keys[pygame.K_LEFT]:
            angle += -1*inc
        if keys[pygame.K_UP]:
            chassis.translate(height_inc)
            coords = chassis.getTotalRobotCoordinates()
            if(checkObstacles(coords, obstacles) and not checkOutofBounds(coords, size) ):
                angle += 0#(np.random.rand()*2 - 1)*1
            else:
                chassis.translate(-height_inc)
        if keys[pygame.K_DOWN]:
            chassis.translate(-1*height_inc)
            coords = chassis.getTotalRobotCoordinates()
            if(checkObstacles(coords, obstacles) and not checkOutofBounds(coords, size) ):
                angle +=  0#(np.random.rand()*2 - 1)*1
            else:
                chassis.translate(height_inc)
        if keys[pygame.K_d] and chassis.target_attached:
            chassis.detachTarget()

        chassis.rotate(angle)

    min_cube, id = instant_distancetoCubes(chassis, obstacles, chassis.getTotalLen())
    if(distance_calculator(chassis.robotSprite, min_cube[1]) < 20 and chassis.target_attached == False):
        chassis.setTarget(id)

    if(chassis.target_attached and min(distance_calculator(chassis.gripper_center, getMidpoint(obstacles['Scale']['Scale_Input1'][1:] )), distance_calculator(chassis.gripper_center,  getMidpoint(obstacles['Scale']['Scale_Input2'][1:])) )  < 60):
        print('You scored on the scale!!!!!')
        id = chassis.target
        chassis.detachTarget()
        obstacles['Cube'].pop(id, None)

    #print(id)
    chassis.drawRobot(screen)
    displayObstacles(obstacles, screen)
    drawRobottoNearestCube(screen, chassis.gripper_center, min_cube[1])
    pygame.display.flip()
    clock.tick(10)

pygame.quit()
