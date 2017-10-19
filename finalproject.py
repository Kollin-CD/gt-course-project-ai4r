import turtle
import math
import random
import sys
from robot import *
from matrix import *

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

    fo = open("test.txt", 'r')
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

    
    x = matrix([[data[-90][0]], [data[-90][1]], [0.], [0.], [0.], [0.]])
    for i in range(-90, -60):
        if data[i][0] < x_min  or data[i][0] > x_max :
            drag = 0.5
            ddx = 1.0
            ddy = 0.0
            
        elif data[i][1] < y_min  or data[i][1] > y_max :
            drag = 0.5
            ddx = 0.
            ddy = 1.
            
        elif distance(data[i], center) < radius:
            drag = 0.5
            ddx = 1.
            ddy = 1.
            
        else:
            drag = 1.
            ddx = 0.
            ddy = 0.
            
        
        F =  matrix([[1., 0., 1., 0., 0., 0.],
                     [0., 1., 0., 1., 0., 0.],
                     [0., 0., drag, 0., ddx, 0.],
                     [0., 0., 0., drag, 0., ddy],
                     [0., 0., 0., 0., 1., 0.],
                     [0., 0., 0., 0., 0., 1.]])
        
        m = matrix([[data[i][0], data[i][1]]])
               
        

        update = KFfilter(x, m, P, F, H, R, I, u)
        x = update[0]
        P = update[1]
        predict.append((x.value[0][0], x.value[1][0]))
        
    
        
    for j in range(60):
        a = x.value[0][0] + x.value[2][0]
        b = x.value[1][0] + x.value[3][0]

        if a < x_min + 20 or a > x_max - 20:
            drag = 0.5
            ddx = 1.
            ddy = 0.0
            
        elif b < y_min + 20 or b > y_max - 20:
            drag = 0.5
            ddx = 0.
            ddy = 1.
            
        elif distance((a, b), center) < radius + 20:
            drag = 0.5
            ddx = 1.
            ddy = 1.
            
        else:
            drag = 1.
            ddx = 0.
            ddy = 0.
            
        
        F =  matrix([[1., 0., 1., 0., 0., 0.],
                     [0., 1., 0., 1., 0., 0.],
                     [0., 0., drag, 0., ddx, 0.],
                     [0., 0., 0., drag, 0., ddy],
                     [0., 0., 0., 0., 1., 0.],
                     [0., 0., 0., 0., 0., 1.]])
        
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
    circle = turtle.Turtle()
    circle.shape('circle')
    circle.penup()
    circle.setpos(center)
    circle.pendown()
    
    robot = turtle.Turtle()
    robot.shape('square')
    robot.color('blue')
    robot.shapesize(.2, .2, .2)
    robot.penup()
    robot.setpos(data[0])

    probot = turtle.Turtle()
    probot.shape('square')
    probot.color('red')
    probot.shapesize(.2, .2, .2)
    probot.penup()
    probot.setpos(predict[0])
    
    robot.pendown()
    probot.pendown()
    for i in range(len(data)):
        robot.goto(data[i])

    for i in range(len(predict)):
        
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
    
   

    
finalproject()
