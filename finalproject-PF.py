import turtle
import math
import random
import sys
from matrix import *
from vec2d import *

class hexbug:

    # --------
    # init: 
    #    creates hexbug and initializes location/orientation 
    #

    def __init__(self, length = 44.0):
        self.x = random.randint(x_min, x_max) # initial x position
        self.y = random.randint(y_min, y_max) # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of hexbug
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero

    # --------
    # set: 
    #    sets a hexbug coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set_noise: 
    #    sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #  

    def measurement_prob(self, measurements):

        # calculate the correct measurement
        predicted_measurements = self.sense(0) # Our sense function took 0 as an argument to switch off noise.


        # compute errors
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            

            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / (bearing_noise ** 2) / 2.0) /  
                      sqrt(2.0 * pi * (bearing_noise ** 2)))

        return error
    
    def __repr__(self): #allows us to print hexbug attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientation))
    
   
    def move(self, motion): # Do not change the name of this function
        steer = motion[0] + random.gauss(0, self.steering_noise)
        dist = motion[1] + random.gauss(0, self.distance_noise)
        turningAngle = (dist / self.length) * tan(steer)
        if abs(turningAngle) < 0.00001:
            x = self.x + dist * cos(self.orientation)
            y = self.y + dist * sin(self.orientation)
            rbug = hexbug(self.length)
            rbug.set(x, y, self.orientation)
            return rbug
        turningRadius = dist / turningAngle
        cx = self.x - sin(self.orientation) * turningRadius
        cy = self.y + cos(self.orientation) * turningRadius
        orientation = (self.orientation + turningAngle) % (2 * pi)
        x = cx + sin(self.orientation + turningAngle) * turningRadius
        y = cy - cos(self.orientation + turningAngle) * turningRadius
        rbug = hexbug(self.length)
        rbug.set(x, y, orientation)
              
        return rbug

    def sense(self, noiseSet = None): #do not change the name of this function
        Z = []
        for i in range(len(landmarks)):
            bearing = (pi - atan2((landmarks[i][0] - self.y), (self.x - landmarks[i][1])) - self.orientation) % (2 * pi)
            if (noiseSet == 0) :
                Z.append(bearing)
            else:
                Z.append(bearing + random.gauss(0, self.bearing_noise))
        return Z 
     

def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]




    







def distance(start, end):
    return math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)

def particle_filter(p, motions, measurements): # I know it's tempting, but don't change N!
    
    # --------
    #
    # Update particles
    #     

    N = len(p)
    
    # motion update (prediction)
    p2 = []
    for i in range(N):
        bug = p[i].move(motion)
        if bug.x < x_min or bug.x > x_max:
            p[i].set(p[i].x, p[i].y, (pi - p[i].orientation) % (2 * pi))
            bug = p[i]
        elif bug.y < y_min or bug.y > y_max:
            p[i].set(p[i].x, p[i].y, 2 * pi - p[i].orientation)
            bug = p[i]
        p2.append(bug)
    p = p2

    # measurement update
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(measurement))

    # resampling
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3
    
    return p
    

data = []
predict = []

#information collected from training_data.txt   
x_min = 240
y_min = 105
x_max = 1696
y_max = 974
center = (968, 539)
radius = 100

landmarks = [(240, 105), (1696, 105), (1696, 974), (240, 974)]
bearing_noise  = 0.5
steering_noise = 0.5
distance_noise = 5.0

fo = open("test01.txt", 'r')
line = fo.readline().rstrip()
while not line == '':
    x, y = int(line.split(',')[0]), int(line.split(',')[1])
    data.append((x, y))
    line = fo.readline().rstrip()

stepDist = []
for i in range(len(data) - 1):
    stepDist.append(distance(data[i], data[i + 1]))
     
dist = sum(stepDist)/len(stepDist)

motion = [0.0, dist]


p = []
N = 100
for i in range(N):
    r = hexbug()
    r.set_noise(bearing_noise, steering_noise, distance_noise)
    p.append(r)

for i in range(len(data)):
    measurement = []
    for j in range(len(landmarks)):
        measurement.append(atan2(landmarks[j][1] - data[i][1], landmarks[j][0] - data[i][0]) )
         
    p = particle_filter(p, motion, measurement)
    
     
    p_pos = get_position(p)
    predict.append((p_pos[0], p_pos[1]))
    
   


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
cl.setpos((968, 439))
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

for i in range(len(data)):
    robot.goto(data[i])
    probot.goto(predict[i])
    
'''robot.pensize(3)
probot.pensize(3)
for i in range(-60, 0):
    robot.goto(data[i])
    probot.goto(predict[i])'''
