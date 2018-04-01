import numpy as np
import pygame
import sys
import os
from collisionutils import *
from colors import *
import initObstacles

os.chdir(os.path.dirname(os.path.realpath(__file__)))

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
            self.target = None

        self.encoder = 0

    def getTotalLen(self):
        return max(self.h, self.w) + (self.gripper)*self.gripper_len

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
        target_baseangle = initObstacles.obstacles['Cube'][self.target][2][1]
        target_radius = initObstacles.obstacles['Cube'][self.target][2][0]
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
        pygame.draw.line(screen, RED, [self.robotSprite[0], self.robotSprite[1]], [final_x, final_y], 2)
        pygame.draw.line(screen, RED, [final_x, final_y], [final_x - arrow_mag*np.cos((45 - self.angle)*np.pi/180), final_y + arrow_mag*np.sin((45 - self.angle)*np.pi/180)], 2)
        pygame.draw.line(screen, RED, [final_x, final_y], [final_x + arrow_mag*np.cos((45 + self.angle)*np.pi/180),final_y + arrow_mag*np.sin((45 + self.angle)*np.pi/180)], 2)

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
        if(self.target_attached and self.target != None):
            coords = self.targetRectCoordinates()
            initObstacles.obstacles['Cube'][self.target][0] = coords
            initObstacles.obstacles['Cube'][self.target][1] = self.gripper_center

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

    def setTarget(self, target):
        self.target = target
        self.target_attached = True

    def detachTarget(self):
        self.target = None
        self.target_attached = False

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
        """if(self.target_attached and self.target != None):
            coords = self.targetRectCoordinates()
            initObstacles.obstacles['Cube'][self.target][0] = coords
            initObstacles.obstacles['Cube'][self.target][1] = self.gripper_center"""

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
        if(abs(self.angle - target) > 5):
            sign = (target - self.angle)/abs(target - self.angle)
            self.rotate(self.angle + sign*10)
            return False
        else:
            return True

class AutoPathFollower:
    def __init__(self, chassis, screen):
        self.tasks = ['T53', 'F336', 'T90', 'F187', 'T179', 'F236']
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
