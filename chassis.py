import numpy as np
import pygame
import sys
import os
from collisionutils import *
from colors import *
import initObstacles
import matplotlib.pyplot as plt

os.chdir(os.path.dirname(os.path.realpath(__file__)))

class RobotDrive:
    def __init__(self,x_mid,y_mid, w, h, Gripper = False, startCube = True):
        self.h = h*0.65
        self.w = w
        #assumes that angle rotations happen around the center of the robot, which is the case for a differential drive
        self.robotSprite = [x_mid,y_mid]
        self.base_angle = np.arctan(self.h/self.w)*180/np.pi
        self.radius = np.sqrt((self.w/2)**2 + (self.h/2)**2)
        self.angle = 0
        self.gripper = Gripper
        if(self.gripper):
            self.gripper_len = 0.30*h
            #self.gripper_width = self.w
            self.gripper_width = 26
            self.gripper_radius = self.h/2  + self.gripper_len/2
            self.gripper_center = [self.robotSprite[0], self.robotSprite[1] - self.gripper_radius ]
            self.target_attached = False
            self.target = None

        if(startCube):
            self.target_attached = True
            self.target = 'RobotCube'
            initObstacles.obstacles['Cube'][self.target] = initObstacles.obstacles['Cube']['RedCube1']
            initObstacles.obstacles['Cube'][self.target][0] = self.targetRectCoordinates()
            initObstacles.obstacles['Cube'][self.target][1] = self.gripper_center
        self.encoder = 0

    def getTotalLen(self):
        return max(self.h, self.w) + (self.gripper)*self.gripper_len

    def robotRectCoordinates(self, **kwargs):
        coords = []
        if(not 'angle' in kwargs.keys()):
            angle1 = self.base_angle - self.angle
            angle2 = self.base_angle + self.angle
        else:
            angle1 = self.base_angle - kwargs['angle']
            angle2 = self.base_angle + kwargs['angle']

        center_x = self.robotSprite[0]
        center_y = self.robotSprite[1]
        if('center' in kwargs.keys()):
            center_x = kwargs['center'][0]
            center_y = kwargs['center'][1]
        coords.append([center_x  - self.radius*np.cos(np.pi*angle2/180),center_y - self.radius*np.sin(np.pi*angle2/180)])
        coords.append([center_x  + self.radius*np.cos(np.pi*angle1/180),center_y - self.radius*np.sin(np.pi*angle1/180)])
        coords.append([center_x  + self.radius*np.cos(np.pi*angle2/180),center_y + self.radius*np.sin(np.pi*angle2/180)])
        coords.append([center_x - self.radius*np.cos(np.pi*angle1/180),center_y + self.radius*np.sin(np.pi*angle1/180)])
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

    def getTotalRobotCoordinates(self, **kwargs):
        if(self.gripper):
            if(len(kwargs) is 0):
                coords = self.robotRectCoordinates()
                upper_left = [coords[0][0] + self.gripper_len*np.sin(self.angle*np.pi/180), coords[0][1] - self.gripper_len*np.cos(self.angle*np.pi/180)]
                upper_right = [coords[1][0] + self.gripper_len*np.sin(self.angle*np.pi/180), coords[1][1] - self.gripper_len*np.cos(self.angle*np.pi/180)]
                return [upper_left, upper_right, coords[2], coords[3]]
            else:
                coords = self.robotRectCoordinates(**kwargs)
                upper_left = [coords[0][0] + self.gripper_len*np.sin(kwargs['angle']*np.pi/180), coords[0][1] - self.gripper_len*np.cos(kwargs['angle']*np.pi/180)]
                upper_right = [coords[1][0] + self.gripper_len*np.sin(kwargs['angle']*np.pi/180), coords[1][1] - self.gripper_len*np.cos(kwargs['angle']*np.pi/180)]
                return [upper_left, upper_right, coords[2], coords[3]]
        else:
            if(len(kwargs) is 0):
                return self.robotRectCoordinates()
            else:
                return self.robotRectCoordinates(**kwargs)

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

    def translate(self,mag, **kwargs):
        if(len(kwargs) is 0 and not 'angle' in kwargs.keys()):
            self.robotSprite[0] = self.robotSprite[0] + mag*np.sin(self.angle*np.pi/180)
            self.robotSprite[1] = self.robotSprite[1] - mag*np.cos(self.angle*np.pi/180)
            if self.gripper:
                self.adjustGripperCenter()
            self.encoder +=mag
        else:
            center = []
            center.append(self.robotSprite[0] + mag*np.sin(kwargs['angle']*np.pi/180))
            center.append(self.robotSprite[1] - mag*np.cos(kwargs['angle']*np.pi/180))
            return self.getTotalRobotCoordinates(angle = kwargs['angle'], center = center), center

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
        pygame.draw.circle(screen, BLUE,int_array(left_end) , int(0.05*self.h), 1)
        pygame.draw.circle(screen, BLUE,int_array(right_end), int(0.05*self.h), 1)
        pygame.draw.line(screen, RED, self.gripper_center, self.gripper_center, 3)
        """if(self.target_attached and self.target != None):
            coords = self.targetRectCoordinates()
            initObstacles.obstacles['Cube'][self.target][0] = coords
            initObstacles.obstacles['Cube'][self.target][1] = self.gripper_center"""

    def resetEncoder(self):
        self.encoder = 0

    def autoDrive(self, target):
        if(abs(self.encoder - target) > 0.5):
            sign = (target - self.encoder)/abs(target - self.encoder)
            self.translate(sign*abs(target - self.encoder)/target *20)
            return False
        else:
            return True

    def autoTurn(self, target):
        target = target%360
        if(abs(self.angle - target) > 0.1):
            sign = (target - self.angle)/abs(target - self.angle)
            self.rotate(self.angle + sign*abs(target-self.angle))
            return False
        else:
            return True

    def moveOptions(self, **kwargs):
        angles = [0, 90, 180, 270]
        realangles = []
        final_intersect = False
        for ix,x in enumerate(angles):
            coords, center = self.translate(3,angle = x)
            collided, id = checkObstacles(coords, initObstacles.obstacles)
            size = (1340, 684)
            #print('Collided',collided)
            if(not collided or checkOutofBounds(coords, size)):
                pass
            else:
                realangles.append([x, center])

            if('obj' in kwargs.keys()):
                collided2 = check1Obstacle(coords, initObstacles.obstacles[kwargs['obj'][0]][kwargs['obj'][1]] )
                if(not collided2):
                    final_intersect = True
        if('obj' in kwargs.keys()):
            return realangles, final_intersect
        else:
            return realangles

"""
            if('obj' in kwargs.keys()):
                collided2 = check1Obstacle(coords, initObstacles.obstacles[kwargs['obj'][0]][kwargs['obj'][1]] )
                if(not collided2):
                    final_intersect = True
        if('obj' in kwargs.keys()):
            return angles, final_intersect
        else:"""

from collections import defaultdict
class AStarSim:
    def __init__(self, chassis, obj = ('Scale', 'Scale_Input1')):
        self.chassis = chassis
        self.distances = defaultdict(lambda : defaultdict(lambda : [9999999,0,[0,0]]))
        self.visited = defaultdict(lambda : defaultdict(lambda : False))
        self.distances[0][0] = [0, self.chassis.angle, self.chassis.robotSprite]
        self.visited[0][0] = False
        self.obj = obj
        self.center_obj = getMidpoint(initObstacles.obstacles[obj[0]][obj[1]][1:])[0:2]

    def getMin(self):
        min = 999999999
        coords = [0,0]
        for k in self.distances.keys():
            for k2, v in self.distances[k].items():
                if(not self.visited[k][k2] and self.distances[k][k2][0] < min):
                    min = self.distances[k][k2][0]
                    coords = [k,k2]
        return min, coords

    def euclidist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def nesteddictprint(self,dict, indent = ''):
        for k, v  in dict.items():
            if(isinstance(v, defaultdict)):
                print(k)
                self.nesteddictprint(v, indent = indent + '|***')
            else:
                if(isinstance(v, list)):
                    print(indent + str(k) + ':' + str(v[0]))
                else:
                    print(indent + str(k) + ':' + str(v))

    def visualizeDistance_Matrix(self):
        max = 500
        blank = np.zeros(shape = (max, max))
        low_row = min(self.distances.keys())

        max = 0
        for k in self.distances.keys():
            for k2 in self.distances[k].keys():
                if(self.distances[k][k2][0] > max):
                    max = self.distances[k][k2][0]

        for k in self.distances.keys():
            for k2 in self.distances[k].keys():
                blank[k - low_row][k2] = self.distances[k][k2][0]/max

        plt.imshow(blank, cmap='Greys',  interpolation='nearest')
        plt.grid()
        plt.show()

    def main(self):
        end = False
        coords = [0,0]
        while(self.euclidist(self.distances[coords[0]][coords[1]][2], self.center_obj) > 150):
            low,coords = self.getMin()
            print(coords, self.distances[coords[0]][coords[1]][1])
            print('Distance:', self.euclidist(self.distances[coords[0]][coords[1]][2], self.center_obj))
            self.visited[coords[0]][coords[1]] = True
            self.chassis.robotSprite = self.distances[coords[0]][coords[1]][2]
            self.chassis.angle = self.distances[coords[0]][coords[1]][1]
            angles, end = self.chassis.moveOptions(obj = self.obj)
            #print('Iteration', i)
            #print(angles)
            for i, center in angles:
                if(i == 0 and not self.visited[coords[0] - 1][coords[1]] ):
                    self.distances[coords[0] - 1][coords[1]][0] = min(self.distances[coords[0] - 1][coords[1]][0], self.distances[coords[0]][coords[1]][0] + 1 + self.euclidist(center, self.center_obj) )
                    self.distances[coords[0] - 1][coords[1]][1] = 0
                    self.distances[coords[0] - 1][coords[1]][2] = center
                elif(i == 90 and not self.visited[coords[0]][coords[1] + 1] ):
                    self.distances[coords[0]][coords[1] + 1][0] = min(self.distances[coords[0]][coords[1] + 1][0], self.distances[coords[0]][coords[1]][0] + 1+ self.euclidist(center, self.center_obj) )
                    self.distances[coords[0]][coords[1] + 1][1] = 90
                    self.distances[coords[0]][coords[1] + 1][2] = center
                elif(i == 180 and not self.visited[coords[0] + 1][coords[1]] ):
                    self.distances[coords[0] + 1][coords[1]][0] = min(self.distances[coords[0] + 1][coords[1]][0], self.distances[coords[0]][coords[1]][0] + 1 + self.euclidist(center, self.center_obj) )
                    self.distances[coords[0] + 1][coords[1]][1] = 180
                    self.distances[coords[0] + 1][coords[1]][2] = center
                elif(i == 270 and not self.visited[coords[0]][coords[1] - 1]):
                    self.distances[coords[0]][coords[1] - 1][0] = min(self.distances[coords[0]][coords[1] - 1][0], self.distances[coords[0]][coords[1]][0] + 1 + self.euclidist(center, self.center_obj) )
                    self.distances[coords[0]][coords[1] - 1][1] = 270
                    self.distances[coords[0]][coords[1] - 1][2] = center
                else:
                    break
        print('Last Cell:', coords)
        #print('Options:', self.getOptions(coords))
        #print('Finished')
        #print('Path:', self.recursePath(self.distances, coords))
        #print(self.nesteddictprint(self.distances))
        print(self.visualizeDistance_Matrix())

    def keys_exists(self,element, *keys):
        _element = element
        for key in keys:
            try:
                _element = _element[key]
            except KeyError:
                return False
        return True

    def getOptions(self, pos):
        options = []
        if(self.keys_exists(self.distances, pos[0] - 1, pos[1])):
            options.append([pos[0] - 1, pos[1]])
        elif(self.keys_exists(self.distances, pos[0] + 1, pos[1])):
            options.append([pos[0] + 1, pos[1]])
        elif(self.keys_exists(self.distances, pos[0], pos[1] + 1)):
            options.append([pos[0], pos[1] + 1])
        elif(self.keys_exists(self.distances, pos[0], pos[1] - 1)):
            options.append([pos[0], pos[1] - 1])
        else:
            pass
        return options

    def recursePath(self,distance_mat, pos, path = [], count = 0):
        if(pos[0] == 0 and pos[1] == 0 or count == 15):
            return path
        neighbor_cells = self.getOptions(pos)
        cell = min(neighbor_cells, key = lambda x: self.distances[x[0]][x[1]][0])
        print(cell)
        diff = [cell[0] - pos[0], cell[1] - pos[1]]
        if(diff[0] != 0 and diff[1] == 0):
            if(diff[0] > 0):
                path.insert(0,'N1')
            else:
                path.insert(0, 'S1')
        elif(diff[0] == 0 and diff[1] != 0):
            if(diff[1] > 0):
                path.insert(0,'E1')
            else:
                path.insert(0, 'W1')
        else:
            path.insert(0, 'invalid')
        return self.recursePath(distance_mat, cell, path, count = count + 1)

class AutoPathFollower:
    def __init__(self, chassis, screen, t):
        self.tasks = t
        if(len(self.tasks) == 0):
            raise Exception('Your task length is not long enough')
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

if __name__ == '__main__':
    start = initObstacles.start
    tasks = ['T90.0', ]
    path =  [[40, 95], [346, 95], [346, 138], [346, 138]]
    real_start = [path[0][0], path[0][1], start[2], start[3]]
    chassis = RobotDrive(x_mid = real_start[0], y_mid=real_start[1], w= real_start[2], h = real_start[3], Gripper = True, startCube = True)
    astar = AStarSim(chassis)
    print(astar.center_obj)
    astar.main()
