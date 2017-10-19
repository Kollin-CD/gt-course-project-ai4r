import turtle
import math
import random
import sys
from robot import *
from matrix import *
from vec2d import *

def finalproject():

    filename = sys.argv[1]
    
    #information collected from training_data.txt   
    x_min = 240
    y_min = 105
    x_max = 1696
    y_max = 974
    center = (968, 539)
    radius = 66

    data = []
    predict = []

    fo = open(filename, 'r')
    line = fo.readline().rstrip()
    while not line == '':
        x, y = int(line.split(',')[0]), int(line.split(',')[1])
        data.append((x, y))
        line = fo.readline().rstrip()

        
    #initialize kalman filter parameters
    P =  matrix([[0., 0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0., 0.],
                 [0., 0., 1000., 0., 0., 0.],
                 [0., 0., 0., 1000., 0., 0.],
                 [0., 0., 0., 0., 1000., 0.],
                 [0., 0., 0., 0., 0., 1000.]])

    F =  matrix([[1., 0., 1., 0., 0., 0.],
                [0., 1., 0., 1., 0., 0.],
                [0., 0., 1.0, 0., 1.0, 0.],
                [0., 0., 0., 1.0, 0., 1.0],
                [0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 1.]])
    
    H =  matrix([[1., 0., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0., 0.]])
    R =  matrix([[0.01, 0.],
                 [0., 0.01]])
    I =  matrix([[1., 0., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0., 0.],
                 [0., 0., 1., 0., 0., 0.],
                 [0., 0., 0., 1., 0., 0.],
                 [0., 0., 0., 0., 1., 0.],
                 [0., 0., 0., 0., 0., 1.]])
    u = matrix([[0.], [0.], [0.], [0.], [0.], [0.]])

    #update kalman filter at last 30 steps
    x = matrix([[data[-30][0]], [data[-30][1]], [0.], [0.], [0.], [0.]])
    
    for i in range(-30, 0):
        x0 = x.value[0][0] + x.value[2][0]
        x1 = x.value[1][0] + x.value[3][0]

        #bounce at walls
        if x0 < x_min:            
            x = matrix([[x_min], [x1], [-x.value[2][0] * 0.3], [x.value[3][0]], [-x.value[4][0] * 0.3], [x.value[5][0]]])
            predict.append((x_min, x1))
            continue
        elif x0 > x_max:            
            x = matrix([[x_max], [x1], [-x.value[2][0] * 0.3], [x.value[3][0]], [-x.value[4][0] * 0.3], [x.value[5][0]]])
            predict.append((x_max, x1))
            continue
        elif x1 < y_min:            
            x = matrix([[x0], [y_min], [x.value[2][0]], [-x.value[3][0] * 0.3], [x.value[4][0]], [-x.value[5][0]* 0.3]])
            predict.append((x0, y_min))
            continue
        elif x1 > y_max:            
            x = matrix([[x0], [y_max], [x.value[2][0]], [-x.value[3][0] * 0.3], [x.value[4][0]], [-x.value[5][0]* 0.3]])
            predict.append((x0, y_max))
            continue

        #bounce at center
        elif distance((x0, x1), center) < radius + 20:
            dxy = collision(x.value[3][0], x.value[4][0], (x0, x1), center)
            
            x = matrix([[x0], [x1], [dxy[0] * 0.8], [dxy[1] * 0.8], [-x.value[4][0]* 0.3], [-x.value[5][0]* 0.3]])
            predict.append((x0, x1))
            continue

        #measurements
        m = matrix([[data[i][0], data[i][1]]])
        #update KF
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((x0, x1))
        
    
    #predict last 60 steps
    for j in range(60):
        a = int(x.value[0][0] + x.value[2][0])
        b = int(x.value[1][0] + x.value[3][0])
        
        if a < x_min:            
            x = matrix([[x_min], [b], [-x.value[2][0] * 0.3], [x.value[3][0] ], [-x.value[4][0] * 0.3], [x.value[5][0]]])
            predict.append((x_min, b))
            continue
        elif a > x_max:            
            x = matrix([[x_max], [b], [-x.value[2][0] * 0.3], [x.value[3][0] ], [-x.value[4][0]* 0.3], [x.value[5][0]]])
            predict.append((x_max, b))
            continue
        elif b < y_min:            
            x = matrix([[a], [y_min], [x.value[2][0]], [-x.value[3][0] * 0.3], [x.value[4][0]], [-x.value[5][0]* 0.3]])
            predict.append((a, y_min))
            continue
        elif b > y_max  :            
            x = matrix([[a], [y_max], [x.value[2][0]], [-x.value[3][0] * 0.3], [x.value[4][0]], [-x.value[5][0]* 0.3]])
            predict.append((a, y_max))
            continue
        elif distance((a, b), center) < radius + 20:            
            dxy = collision(x.value[3][0], x.value[4][0], (a, b), center)            
            x = matrix([[a], [b], [dxy[0] * 0.8], [dxy[1] * 0.8], [-x.value[4][0]* 0.3], [-x.value[5][0]* 0.3]])
            predict.append((a, b))
            continue

        
        m = matrix([[a , b]])
       
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((a, b))  

    #export data to files
    result = open("prediction.txt", "w")
    real = open("real.txt", "w")
    for i in range(-60, 0):
        result.write(str(predict[i][0]) + ',' + str(predict[i][1]) + '\n')
        real.write(str(data[i][0]) + ',' + str(data[i][1]) + '\n')
        
    
    
   
    
        

    '''with open('prediction.txt', 'w') as f:
        for _ in range(60):
            print >> f, '%s,%s' % (x.strip(), y.strip())'''

def distance(start, end):
    return math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)

def KFfilter(x, m, P, F, H, R, I, u):

    # prediction
    x = (F * x) + u
    P = F * P * F.transpose()
    
    # measurement update
    Z = m
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    return [x, P]
    
def collision(dx, dy, touch, center):
    v1 = Vec2d(dx, dy)
    v2 = Vec2d(touch[0] - center[0], touch[1] - center[1])
    sq = (touch[0] - center[0])**2 + (touch[1] - center[1])**2
    v3 = v1 - 2*v1.dot(v2)*v2/sq
    return (v3.x, v3.y)   

    
finalproject()
