from colors import *
import os
from utilutils import getPolygon_Mid_RadiusAngle_Info

os.chdir(os.path.dirname(os.path.realpath(__file__)))

parser = lambda x: list(map(lambda y: float(y), x))

def fileReader(filename):
    f = open(filename, 'r')
    obstacles ={
        'Cube':
        {

        },
        'Switch':
        {

        },
        'Scale':
        {

        }
    }
    f.readline()
    l = f.readline()
    while(l):
        splits = l.split(':')
        id = splits[0]
        str_numbers = splits[1]
        numbers = parser(str_numbers.split(','))
        #start has the following format: [x_mid, y_mid, w, h]
        start = []
        if('Cube' in id):
            c = RED
            if('Blue' in id):
                c = BLUE
            obstacles['Cube'][id] = getPolygon_Mid_RadiusAngle_Info([numbers[1], numbers[0], numbers[3], numbers[2]])
        if('Scale' in id):
            if('Scale_Input' in id):
                obstacles['Scale'][id] = [GREEN, numbers[1], numbers[0], numbers[3], numbers[2]]
            else:
                obstacles['Scale'][id] = [BLACK, numbers[1], numbers[0], numbers[3], numbers[2]]
        if('Switch' in id):
            if('Top' in id or 'Bottom' in id):
                obstacles['Switch'][id] = [BLACK, numbers[1], numbers[0], numbers[3], numbers[2]]
            else:
                c = RED
                if('Blue' in id):
                    c = BLUE
                obstacles['Switch'][id] = [c,numbers[1], numbers[0], numbers[3], numbers[2]]
        if('Start' in id):
            placeholder = [numbers[1], numbers[0], numbers[3], numbers[2]]
            start = [placeholder[0] + placeholder[2]/2, placeholder[1] + placeholder[3]/2, placeholder[2], placeholder[3]]
        l = f.readline()
    return obstacles, start
