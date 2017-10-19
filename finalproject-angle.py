import turtle
import math
import random
import sys
from robot import *
from matrix import *
from vec2d import *

def finalproject():

    #filename = sys.argv[1]
    tdata = []
    data = []
    predict = []
    fo = open("training_data.txt", 'r')
    line = fo.readline().rstrip()
    while not line == '':
        x, y = int(line.split(',')[0]), int(line.split(',')[1])
        tdata.append((x, y))
        line = fo.readline().rstrip()

    x_min = tdata[0][0]
    y_min = tdata[0][1]
    x_max = 0
    y_max = 0
    for i in range(len(tdata)):
        if tdata[i][0] < x_min:
            x_min = tdata[i][0]
        if tdata[i][1] < y_min:
            y_min = tdata[i][1]
        if tdata[i][0] > x_max:
            x_max = tdata[i][0]
        if tdata[i][1] > y_max:
            y_max = tdata[i][1]

    print x_min, y_min, x_max, y_max
    center = ((x_min + x_max)/2, (y_min + y_max)/2)
    radius = min((x_max - x_min)/2, (y_max - y_min)/2)

    for i in range(len(tdata)):
        dist = distance(tdata[i], center)
        if dist < radius:
            radius = dist

    print radius

    fo = open("test01.txt", 'r')
    line = fo.readline().rstrip()
    while not line == '':
        x, y = int(line.split(',')[0]), int(line.split(',')[1])
        data.append((x, y))
        line = fo.readline().rstrip()

        
    #kalman filter
    P =  matrix([[0., 0., 0., 0., 0., 0.],
                 [0., 0., 0., 0., 0., 0.],
                 [0., 0., 1000., 0., 0., 0.],
                 [0., 0., 0., 1000., 0., 0.],
                 [0., 0., 0., 0., 1000., 0.],
                 [0., 0., 0., 0., 0., 1000.]])
    F =  matrix([[1., 0., 1., 0., 0., 0.],
                [0., 1., 0., 1., 0., 0.],
                [0., 0., 1., 0., 1., 0.],
                [0., 0., 0., 1., 0., 1.],
                [0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 1.]])
    
    H =  matrix([[1., 0., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0., 0.]])
    R =  matrix([[0.1, 0.],
                 [0., 0.1]])
    I =  matrix([[1., 0., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0., 0.],
                 [0., 0., 1., 0., 0., 0.],
                 [0., 0., 0., 1., 0., 0.],
                 [0., 0., 0., 0., 1., 0.],
                 [0., 0., 0., 0., 0., 1.]])
    u = matrix([[0.], [0.], [0.], [0.], [0.], [0.]])

    
    x = matrix([[data[0][0]], [data[0][1]], [0.], [0.], [0.], [0.]])
    
    for i in range(len(data) - 60):
        x0 = x.value[0][0]
        x1 = x.value[1][0]
        angle = x.value[2][0]
        
        
        if x0 < x_min + 5  or x0 > x_max - 5  :            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [x.value[2][0] * 0.8], [ x.value[3][0] * 0.8], [x.value[4][0] * 0.5], [x.value[5][0] * 0.5]])
            
        elif x1 < y_min + 5  or x1 > y_max - 5 :            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [x.value[2][0] * 0.8], [x.value[3][0] * 0.8], [x.value[4][0] * 0.5], [x.value[5][0] * 0.5]])
            
        elif distance((x0, x1), center) < radius + 50:
            dxy = collision(x.value[3][0], x.value[4][0], (x0, x1), center)
            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [dxy[0] * 0.8], [dxy[1] * 0.8], [x.value[4][0] * 0.5], [x.value[5][0] * 0.5]])
            
        
        m = matrix([[data[i][0], data[i][1]]])
       
        
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((x.value[0][0], x.value[1][0]))
        
    
    
    for j in range(60):
        a = x.value[0][0] + x.value[2][0]
        b = x.value[1][0] + x.value[3][0]
        angle = x.value[2][0] + x.value[4][0]
        if a < x_min + 5  or a > x_max - 5 :            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [-x.value[2][0] * 0.8], [x.value[3][0] * 0.8], [-x.value[4][0] * 0.5], [x.value[5][0] * 0.5]])
            
        elif b < y_min + 5  or b > y_max - 5 :            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [x.value[2][0] * 0.8], [-x.value[3][0] * 0.8], [x.value[4][0] * 0.5], [-x.value[5][0] * 0.5]])
           
        elif distance((a, b), center) < radius + 50 :            
            dxy = collision(x.value[3][0], x.value[4][0], (a, b), center)            
            x = matrix([[x.value[0][0]], [x.value[1][0]], [dxy[0] * 0.8], [dxy[1] * 0.8], [-x.value[4][0] * 0.5], [x.value[5][0] * 0.5]])
            

        m = matrix([[a, b]])
       
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((x.value[0][0], x.value[1][0]))    

    
    #show movements
    window = turtle.Screen()
    window.reset()
    window.setworldcoordinates(x_min - 500, y_min - 500, x_max + 500, y_max + 500)
    window.bgcolor('white')
    #circle = turtle.Turtle()
    #circle.shape('circle')
    #circle.penup()
    #circle.setpos(center)
    #circle.pendown()
    
    robot = turtle.Turtle()
    robot.shape('square')
    robot.color('blue')
    robot.shapesize(.5, .5, .5)
    robot.penup()
    robot.setpos(data[0])

    probot = turtle.Turtle()
    probot.shape('square')
    probot.color('red')
    probot.shapesize(.5, .5, .5)
    probot.penup()
    probot.setpos(predict[0])
    
    robot.pendown()
    probot.pendown()
    for i in range(len(data) -60):
        robot.goto(data[i])
        probot.goto(predict[i])
        
    robot.pensize(3)
    probot.pensize(3)
    for i in range(-60, 0):
        robot.goto(data[i])
        probot.goto(predict[i])
    
   
    
        

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
