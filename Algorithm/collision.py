import numpy as np
from shapely.geometry import Point, Polygon

o = Polygon(((5,5),(10,5),(10,10),(5,10),(5,5)))

def Node_collision_check(x,y):

    p = Point(x,y)

    if p.within(o):
        return False

    else:
        return True

def Edge_collision_check(x_nearest, x_new):

    seg_length = 0.1
    seg_point = int(np.ceil(np.linalg.norm(x_new - x_nearest) / seg_length))
    v = (x_new - x_nearest)/seg_point

    for i in range(seg_point + 1):
        seg = x_nearest + i * v
        if Node_collision_check(seg[0],seg[1]) :
            continue
        else:
            return False
    return True



