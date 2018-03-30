import numpy as np
import pygame

class Arm:
    def __init__(self, start, length, angle):
        self.len = length
        self.hand_len = 0.5*self.len
        self.start = start
        self.angle = angle
        self.bias = -90

        self.end = [self.start[0] + int(self.len*np.cos(self.angle*np.pi/180)), self.start[1] + int(self.len*np.sin(self.angle*np.pi/180))]

        self.hand_start = self.end
        self.hand_end =  [self.hand_start[0] + self.hand_len, self.hand_start[1]]

    def rotate(self, val):
        self.angle = val
        self.end[0] = self.start[0] + int(self.len*np.cos((val+self.bias)*np.pi/180))
        self.end[1] = self.start[1]+ int(self.len*np.sin((val + self.bias)*np.pi/180))
        self.rotate_hand(val)

    def rotate_hand(self, val):
        self.hand_start = self.end
        self.hand_end = [self.hand_start[0] + self.hand_len, self.hand_start[1]]

    def translate(self, val):
        self.start[1] = val
        self.end[1] = self.start[1] + int(self.len* np.sin((self.angle + self.bias) * np.pi/180) )

        self.hand_start = self.end
        self.hand_end = [self.hand_start[0] + self.hand_len, self.hand_start[1]]

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

inc = 10
height_inc = 10
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

angle = 30
done = True

arm = Arm(start = start_coords, length = length, angle = angle)

while(done):

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = False

    keys=pygame.key.get_pressed()
    if( angle <= Hard_Lims[1] and angle >= Hard_Lims[0]):
        if keys[pygame.K_RIGHT]:
            if(angle < Hard_Lims[1] - inc):
                angle += inc
        if keys[pygame.K_LEFT]:
            if(angle > Hard_Lims[0] + inc):
                angle += -1*inc

    if(height <= Hard_YLims[1] and height >= Hard_YLims[0]):
        if keys[pygame.K_UP]:
            if(height > Hard_YLims[0] + height_inc):
                height += -height_inc
        if keys[pygame.K_DOWN]:
            if(height < Hard_YLims[1] - height_inc):
                height += height_inc
    print(height, angle)
    arm.rotate(angle)
    arm.translate(height)

    screen.fill(WHITE)
    pygame.draw.line(screen, RED, arm.start, arm.end)
    pygame.draw.line(screen, RED, arm.hand_start, arm.hand_end)
    pygame.display.flip()
    clock.tick(30)
    angle += (2*np.random.random() -1)*0.75
    height += (2*np.random.random() -1)*0.75
