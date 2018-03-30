import numpy as np
import pygame
import sys
import os

os.chdir(os.path.dirname(os.path.realpath(__file__)))
from collisionutils import *


obstacles = {
'Cubes':
    [

    ],
}

class RobotDrive:
    def __init__(self,x_mid,y_mid, w, h, Gripper = False):
        self.h = h
        self.w = w
        #assumes that angle rotations happen around the center of the robot, which is the case for a differential drive
        self.robotSprite = [x_mid,y_mid]
        self.base_angle = np.arctan(self.h/self.w)*180/np.pi
        self.radius = np.sqrt((self.w/2)**2 + (self.h/2)**2)
        self.angle = 0
        self.gripper = Gripper
        if(self.gripper):
            self.gripper_len = 0.35*self.h
            self.gripper_width = self.w
            self.gripper_radius = self.h/2  + self.gripper_len/2
            self.gripper_center = [self.robotSprite[0], self.robotSprite[1] - self.gripper_radius ]
            self.target_attached = False

        self.encoder = 0

    def robotRectCoordinates(self):
        coords = []
        angle1 = self.base_angle - self.angle
        angle2 = self.base_angle + self.angle
        coords.append([self.robotSprite[0]  - self.radius*np.cos(np.pi*angle2/180), self.robotSprite[1] - self.radius*np.sin(np.pi*angle2/180)])
        coords.append([self.robotSprite[0]  + self.radius*np.cos(np.pi*angle1/180), self.robotSprite[1] - self.radius*np.sin(np.pi*angle1/180)])
        coords.append([self.robotSprite[0]  + self.radius*np.cos(np.pi*angle2/180), self.robotSprite[1] + self.radius*np.sin(np.pi*angle2/180)])
        coords.append([self.robotSprite[0] - self.radius*np.cos(np.pi*angle1/180), self.robotSprite[1] + self.radius*np.sin(np.pi*angle1/180)])
        return coords

    def targetRectCoordinates(self):
        coords = []
        angle1 = target_baseangle - self.angle
        angle2 = target_baseangle + self.angle
        coords.append([self.gripper_center[0]  - target_radius*np.cos(np.pi*angle2/180), self.gripper_center[1] - target_radius*np.sin(np.pi*angle2/180)])
        coords.append([self.gripper_center[0]  + target_radius*np.cos(np.pi*angle1/180), self.gripper_center[1] - target_radius*np.sin(np.pi*angle1/180)])
        coords.append([self.gripper_center[0]  + target_radius*np.cos(np.pi*angle2/180), self.gripper_center[1] + target_radius*np.sin(np.pi*angle2/180)])
        coords.append([self.gripper_center[0] - target_radius*np.cos(np.pi*angle1/180), self.gripper_center[1] + target_radius*np.sin(np.pi*angle1/180)])
        return coords

    def getTotalRobotCoordinates(self):
        if(self.gripper):
            coords = self.robotRectCoordinates()
            upper_left = [coords[0][0] + self.gripper_len*np.sin(self.angle*np.pi/180), coords[0][1] - self.gripper_len*np.cos(self.angle*np.pi/180)]
            upper_right = [coords[1][0] + self.gripper_len*np.sin(self.angle*np.pi/180), coords[1][1] - self.gripper_len*np.cos(self.angle*np.pi/180)]
            return [upper_left, upper_right, coords[2], coords[3]]
        else:
            return self.robotRectCoordinates()

    def drawOrientationArrow(self,screen):
        line_mag = 0.75*self.h/2
        arrow_mag = 0.25*line_mag
        final_x = self.robotSprite[0] + line_mag*np.sin(self.angle*np.pi/180)
        final_y = self.robotSprite[1]  - line_mag*np.cos(self.angle*np.pi/180)
        pygame.draw.line(screen, RED, [self.robotSprite[0], self.robotSprite[1]], [final_x, final_y], 3)
        pygame.draw.line(screen, RED, [final_x, final_y], [final_x - arrow_mag*np.cos((45 - self.angle)*np.pi/180), final_y + arrow_mag*np.sin((45 - self.angle)*np.pi/180)], 1)
        pygame.draw.line(screen, RED, [final_x, final_y], [final_x + arrow_mag*np.cos((45 + self.angle)*np.pi/180),final_y + arrow_mag*np.sin((45 + self.angle)*np.pi/180)], 1)

    def drawRobot(self, screen):
        pygame.draw.polygon(screen, BLACK, self.robotRectCoordinates())
        """coords = self.robotRectCoordinates()
        pygame.draw.line(screen, RED, coords[0],coords[0], 3)
        pygame.draw.line(screen, GREEN, coords[1],coords[1], 3)
        pygame.draw.line(screen, BLUE, coords[2],coords[2], 3)
        pygame.draw.line(screen, BLACK, coords[3],coords[3], 3)"""
        self.drawOrientationArrow(screen)
        if(self.gripper):
            self.drawGripper(screen)

    def adjustGripperCenter(self):
        self.gripper_center[0] = self.robotSprite[0] + self.gripper_radius*np.sin(self.angle*np.pi/180)
        self.gripper_center[1] = self.robotSprite[1] - self.gripper_radius*np.cos(self.angle*np.pi/180)
        #if(self.target_attached):
        #    global target_mid
        #    target_mid = self.gripper_center

    def rotate(self, angle):
        self.angle = angle%360
        if self.gripper:
            self.adjustGripperCenter()

    def translate(self,mag):
        self.robotSprite[0] = self.robotSprite[0] + mag*np.sin(self.angle*np.pi/180)
        self.robotSprite[1] = self.robotSprite[1] - mag*np.cos(self.angle*np.pi/180)
        if self.gripper:
            self.adjustGripperCenter()
        self.encoder +=mag

    def drawGripper(self, screen):
        angle1 = self.base_angle - self.angle
        angle2 = self.base_angle + self.angle
        top_left = [self.robotSprite[0]  - self.radius*np.cos(np.pi*angle2/180), self.robotSprite[1] - self.radius*np.sin(np.pi*angle2/180)]

        int_array = lambda x: list(map(lambda y: int(y), x))

        left_start = [top_left[0] +  (self.w - self.gripper_width)*0.5*np.cos(np.pi*self.angle/180),top_left[1] + (self.w - self.gripper_width)*0.5*np.sin(np.pi*self.angle/180)]
        right_start = [left_start[0] + self.gripper_width*np.cos(np.pi*self.angle/180), left_start[1] + self.gripper_width*np.sin(np.pi*self.angle/180)]
        left_end = [left_start[0] + self.gripper_len*np.sin(np.pi*(self.angle)/180), left_start[1] - self.gripper_len*np.cos(np.pi*(self.angle)/180)]
        right_end = [right_start[0] + self.gripper_len*np.sin(np.pi*(self.angle)/180), right_start[1] - self.gripper_len*np.cos(np.pi*(self.angle)/180)]
        pygame.draw.line(screen, GREEN, left_start,right_start, 2)
        pygame.draw.line(screen, GREEN, left_start,left_end, 2)
        pygame.draw.line(screen, GREEN, right_start,right_end, 2)
        pygame.draw.circle(screen, BLUE,int_array(left_end) , int(0.25*self.gripper_len), 1)
        pygame.draw.circle(screen, BLUE,int_array(right_end), int(0.25*self.gripper_len), 1)
        pygame.draw.line(screen, RED, self.gripper_center, self.gripper_center, 3)
        if(self.target_attached and self.gripper):
            global target_polygon
            coords = self.targetRectCoordinates()
            target_polygon = coords
            pygame.draw.polygon(screen, GREEN, coords)

    def resetEncoder(self):
        self.encoder = 0

    def autoDrive(self, target):
        if(abs(self.encoder - target) > 2):
            sign = (target - self.encoder)/abs(target - self.encoder)
            self.translate(sign*4)
            return False
        else:
            return True

    def autoTurn(self, target):
        target = target%360
        if(abs(self.angle - target) > 2):
            sign = (target - self.angle)/abs(target - self.angle)
            self.rotate(self.angle + sign*10)
            return False
        else:
            return True

class AutoPathFollower:
    def __init__(self, chassis, screen):
        #self.tasks = ['T0', 'F50', 'T60', 'F50', 'F-50', 'T210']
        self.tasks = ['T0']
        self.chassis = chassis
        self.screen = screen

    def autoPeriodic(self, verbose = False):
        if(len(self.tasks) != 0):
            task = self.tasks[0]
            done = False
            if(task[0] == 'F'):
                x = float(task[1:])
                if(verbose):
                    self.displayTargetLine(x)
                done = self.chassis.autoDrive(x)
            if(task[0] == 'T'):
                done = self.chassis.autoTurn(float(task[1:]))
            if(done == True):
                self.chassis.resetEncoder()
                self.tasks.pop(0)
            return False
        else:
            return True

    def displayTargetLine(self, x):
        pass

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

inc = 10
height_inc = 5
length = 100
height = 350
start_coords = [350,350]

Hard_Lims = [30, 150]
Hard_YLims = [int(0 + length *np.cos(30*np.pi/180)), int(700 -length *np.cos(30*np.pi/180))]
#87, 663
print(Hard_YLims)
pygame.font.init()
font = pygame.font.SysFont("sans", 10)

pygame.init()
size = (700, 700)
screen = pygame.display.set_mode(size)
screen.fill(WHITE)
clock = pygame.time.Clock()

angle = 0
translate = 0

done = True
chassis = RobotDrive(350, 350, 20, 20, True)
auto = AutoPathFollower(chassis, screen)

target = [30, 30, 15, 15]
target_baseangle = np.arctan(target[3]/target[2])*180/np.pi
target_radius = np.sqrt((target[2]/2)**2 + (target[3]/2)**2)
target_polygon = [[target[0], target[1]], [target[0] + target[2], target[1]], [target[0] + target[2], target[1] + target[3]], [target[0], target[1] + target[3] ] ]
target_mid = [target[0] + target[2]/2, target[1] + target[3]/2]
show_unattachedtarg = True

obstacle_rect = [50, 100, 600, 30]
obstacle_polygon = [[obstacle_rect[0], obstacle_rect[1]], [obstacle_rect[0] + obstacle_rect[2], obstacle_rect[1]], [obstacle_rect[0] + obstacle_rect[2], obstacle_rect[1] + obstacle_rect[3]], [obstacle_rect[0], obstacle_rect[1] + obstacle_rect[3]]]
field_polygon = [[0,0],[0 + size[0], 0],[ 0 + size[0], 0 + size[1]],[0, size[1]]]

while(done):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = False

    bool = auto.autoPeriodic()
    #print('Gyro Angle: %s ' %(chassis.angle))
    #print('Encoder: %s ' % (chassis.encoder))

    screen.fill(WHITE)
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
            if(not collide(coords, obstacle_polygon) and not checkOutofBounds(coords, size) ):
                angle += (np.random.rand()*2 - 1)*1
            else:
                chassis.translate(-height_inc)

        if keys[pygame.K_DOWN]:
            chassis.translate(-1*height_inc)
            coords = chassis.getTotalRobotCoordinates()
            if(not collide(coords, obstacle_polygon) and not checkOutofBounds(coords, size) ):
                angle +=  (np.random.rand()*2 - 1)*1
            else:
                chassis.translate(height_inc)

        chassis.rotate(angle)
        """
        if keys[pygame.K_d]:
            chassis.target_attached = False
            show_unattachedtarg = True
            print('Detached')
        else:
            distance = np.sqrt((target_mid[0]- chassis.gripper_center[0])**2 +  (target_mid[1]- chassis.gripper_center[1])**2)
            #print('Distance to Target: %s' %(distance))
            if(distance > 10):
                pygame.draw.line(screen, BLACK, chassis.gripper_center, target_mid)
            else:
                show_unattachedtarg = False
                chassis.target_attached = True"""

    chassis.drawRobot(screen)
    if(show_unattachedtarg):
        pygame.draw.polygon(screen, GREEN, target_polygon)
    pygame.draw.rect(screen, RED, [50, 100, 600, 30])
    pygame.display.flip()
    clock.tick(10)
