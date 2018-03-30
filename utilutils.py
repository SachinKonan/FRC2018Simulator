import numpy as np

def getMidpoint(rect_form):
    return [rect_form[0] + rect_form[2]/2, rect_form[1] + rect_form[3]/2, rect_form[2], rect_form[3]]

def getPolygon(target):
    return [[target[0], target[1]], [target[0] + target[2], target[1]], [target[0] + target[2], target[1] + target[3]], [target[0], target[1] + target[3] ] ]
    
def getPolygon_Mid_RadiusAngle_Info(target):
    target_baseangle = np.arctan(target[3]/target[2])*180/np.pi
    target_radius = np.sqrt((target[2]/2)**2 + (target[3]/2)**2)
    target_polygon = [[target[0], target[1]], [target[0] + target[2], target[1]], [target[0] + target[2], target[1] + target[3]], [target[0], target[1] + target[3] ] ]
    target_mid = [target[0] + target[2]/2, target[1] + target[3]/2]
    return [target_polygon, target_mid, [target_radius, target_baseangle]]
