import turtle
import math
import random
import sys
from robot import *
from matrix import *
from vec2d import *

def finalproject():
    data = []
    predict = []
    #information collected from training_data.txt   
    x_min = 260
    y_min = 125
    x_max = 1716
    y_max = 994
    center = (988, 560)
    radius = 100

    
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

    
    x = matrix([[data[0][0]], [data[0][1]], [0.], [0.], [0.], [0.]])
    
    for i in range(len(data) - 60):
        x0 = x.value[0][0] + x.value[2][0]
        x1 = x.value[1][0] + x.value[3][0]               

        #measurements
        m = matrix([[data[i][0], data[i][1]]])
        #update KF
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((x0, x1))
        
    
    #predict last 60 steps
    for j in range(60):
        a = x.value[0][0] + x.value[2][0]
        b = x.value[1][0] + x.value[3][0]
        
               
        m = matrix([[a , b]])
       
        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((int(a), int(b)))     

    print data[-1]
    #show movements
    window = turtle.Screen()
    window.reset()
    window.setworldcoordinates(x_min - 500, y_min - 500, x_max + 500, y_max + 500)
    window.bgcolor('white')

    bd = turtle.Turtle()
    bd.shape('square')
    bd.color('black')
    bd.shapesize(.01, .01, .01)
    bd.penup()
    bd.setpos((260, 125))
    bd.pendown()
    bd.goto((1716, 125))
    bd.goto((1716, 994))
    bd.goto((260, 994))
    bd.goto((260, 125))
    
   

    cl = turtle.Turtle()
    cl.shape('square')
    cl.color('black')
    cl.shapesize(.01, .01, .01)
    cl.penup()
    cl.setpos((988, 460))
    cl.pendown()
    cl.circle(100)
    
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
    for i in range(200):
        robot.goto(data[i])
        probot.goto(predict[i])
    robot.pensize(3)
    probot.pensize(3)
    #for i in range(-60, 0):
        #robot.goto(data[i])
        #probot.goto(predict[i])
    
   
    
        

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

def interaction(x, y, dx, dy, center, radius):
    x = x
    y = y
    for i in range(50):
        x = x - dx / 50.0
        y = y - dy / 50.0
        if abs (distance((x, y), center) - radius) < 2:
            return (x, y)
    return (x, y)

def collision(dx, dy, touch, center):
    v1 = Vec2d(dx, dy)
    v2 = Vec2d(touch[0] - center[0], touch[1] - center[1])
    sq = (touch[0] - center[0])**2 + (touch[1] - center[1])**2
    v3 = v1 - 2*v1.dot(v2)*v2/sq
    return (v3.x, v3.y)   

    
finalproject()
