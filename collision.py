import math
from vec2d import *



def collision(dx, dy, touch, center):
    v1 = Vec2d(dx, dy)
    v2 = Vec2d(touch[0] - center[0], touch[1] - center[1])
    sq = (touch[0] - center[0])**2 + (touch[1] - center[1])**2
    v3 = v1 - 2*v1.dot(v2)*v2/sq
    return (v3.x, v3.y)

print collision(1, 1, (0, 1), (1,1))
