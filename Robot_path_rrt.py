#!/usr/bin/env python

import sys,math,random,pygame
from pygame.locals import *



import matplotlib.pyplot as pl
import numpy as np

#Initialization of constants 

PHI = 45
s = 0.5 
PG = 0.05
d = 10000
STEP_SIZE = 5
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,255,255
Nmax = 1000
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
GAME_LEVEL = 2
rectlist = []
cnt = 0
XDIM = 720
YDIM = 500
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('Rapidly Exploring Random Tree')
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0



def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


class Node(object):
    """Node in a tree"""
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


#Checking collision between the circular initial and goal configuration with a point
def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False

#Checking if a point collides with an obstacle
def collides(p):
    for rect in rectlist:
        if rect.collidepoint(p) == True:
            # print ("collision with object: " + str(rect))
            return True
    return False

#Generates a random node
def get_random():
    return random.random()*XDIM, random.random()*YDIM

def rand_config():
    p = [-1,-1]
    while p[0] < 0 or p[0] >100:
        p[0] = random.random()*XDIM
        p[0] *= 10
    while p[1] < 0 or p[1] >100:
        p[1] = random.random()*YDIM
        p[1] *= 10
    return p

def get_random_biased():
    while True:
        p = get_random()
        #p = get_random(goalPointCoord.point,PG,d)
        noCollision = collides(p)
        if noCollision == False:
            return p
        return p
#Steps from a point to another
def step(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

#Initializes the "world"
def init_obstacles(config):
    global rectlist
    rectlist = []
    print("config "+ str(config))
    if (config == 0):
        rectlist.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100),(100,200)))
    if (config == 1):
        rectlist.append(pygame.Rect((20,10),(100,200)))
        rectlist.append(pygame.Rect((500,200),(500,200)))
    if (config == 2):
        rectlist.append(pygame.Rect((20,10),(100,200)))
	rectlist.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100),(100,200)))
	rectlist.append(pygame.Rect((300,40),(350,100)))
    if (config == 3):
        rectlist.append(pygame.Rect((20,10),(100,200)))

    for rect in rectlist:
        pygame.draw.rect(screen, red, rect)

#Resets the "World"
def reset():
    global cnt
    screen.fill(black)
    init_obstacles(GAME_LEVEL)
    cnt = 0

#Main function
def main():
    global cnt
    initPoseSetFlag = False
    initialPoint = Node(None, None)
    goalPoseSetFlag = False
    global goalPointCoord
    goalPointCoord = Node(None,None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        if currentState == 'init':
            print('goal point not yet set')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            #traceback from the goal to init
            currNode = goalNode.parent
            while currNode.parent != None:
                pygame.draw.aaline(screen,cyan,currNode.point,currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'buildTree':
            cnt = cnt+1
            if cnt < Nmax:
                foundNext = False
                while foundNext == False:
                    rand = get_random_biased()
                    parentNode = nodes[0]
                    #Extend functionality of the RRT Algorithm
                    for p in nodes: #find nearest vertex
                    #checking to see if this vertex is closer than the previously selected closest vertex
                        if dist(p.point,rand) <= dist(parentNode.point,rand): 
                            newPoint= step(p.point,rand)
                            # checking if a collision would occur with the newly selected vertex
                            if collides(newPoint) == False: 
                                parentNode = p #the new point is not in collision, so update this new vertex as the best
                                foundNext = True #return True if not in collision

                newnode=step(parentNode.point,rand)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.aaline(screen,white,parentNode.point,newnode)

                if point_circle_collision(newnode, goalPointCoord.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    print "Goal Found!"
                    goalNode = nodes[len(nodes)-1]

                if cnt%100 == 0:
                    print("node: " + str(cnt))
            else:
                print("Ran out of nodes... :(")
                return;

        #handle mouse events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSetFlag == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initial pose set: '+str(e.pos))
                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint)
                            initPoseSetFlag = True
                            pygame.draw.circle(screen, blue, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSetFlag == False:
                        print('goal pose set: '+str(e.pos))
                        if collides(e.pos) == False:
                            goalPointCoord = Node(e.pos,None)
                            goalPoseSetFlag = True
                            pygame.draw.circle(screen, green, goalPointCoord.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSetFlag = False
                    goalPoseSetFlag = False
                    reset()

        pygame.display.update()
        fpsClock.tick(10000)

if __name__ == '__main__':
    main()
    input("press Enter to quit")