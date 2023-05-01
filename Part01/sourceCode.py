#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 25 17:09:37 2023

@authors: mothishraj satish
"""
import heapq
import numpy as np
import math
import cv2
import pygame
import time

import itertools
import threading
import sys

import matplotlib.pyplot as plt





"""
----------------------------------------------------------------
Loading animation 
---------------------------------------------------------------- 
"""


is_loading_a_star = False
is_loading_backtrack = False


def animate_A_star():
    for c in itertools.cycle(["⢿", "⣻", "⣽", "⣾", "⣷", "⣯", "⣟", "⡿"]):
        if is_loading_a_star:
            break
        sys.stdout.write('\rRunning A* Algorithm ' + c)
        sys.stdout.flush()
        time.sleep(0.1)


def animate_Backtrack():
    for c in itertools.cycle(["⢿", "⣻", "⣽", "⣾", "⣷", "⣯", "⣟", "⡿"]):
        if is_loading_backtrack:
            break
        sys.stdout.write('\rBacktracking ' + c)
        sys.stdout.flush()
        time.sleep(0.1)


"""
----------------------------------------------------------------
 Creating obstacles in the map
---------------------------------------------------------------- 
"""


def obstacle_space(mapWidth,mapHeight,clearance):
    obstacle_cord=[]
    offset = clearance
    for x in range(mapWidth):
        for y in range(mapHeight):
            # Clearance plus radius around the boundaries
            if (x<=offset) or (x>= 600-offset) or (y<=offset) or (y>= 200-offset):
                #map[y][x] = [0,0,255]
                obstacle_cord.append((x,y))
            # Rectangle of size 150 by 1250 with offset
            if ((x>=150-offset) and (x<=165+offset)) and ((y>=75-offset) and (y<=mapHeight-offset)):
                #map[y][x] = [0, 255, 255] 
                obstacle_cord.append((x,y))
            # Rectangle of size 150 by 1250 with offset
            if ((x>=250-offset) and (x<=265+offset)) and ((y>=offset) and (y<=125+offset) ):
                #map[y][x] = [255, 255, 0]
                obstacle_cord.append((x,y))
            # Circle with offset
            if ((x-400)**2 + (y-110)**2 - (50+offset)**2 )<=0:
                #map[y][x] = [255,0,255]
                obstacle_cord.append((x,y))
    return obstacle_cord,obstacle_cord


"""
----------------------------------------------------------------
Action set
---------------------------------------------------------------- 
"""
def actionSet(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038*100
    L = 0.34*100
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180 
    UL = ( 2*np.pi * UL)/60
    UR = (2*np.pi * UR)/60

    
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xn +=  0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    #Thetan = int(180 * (Thetan) / 3.14)
    Xn, Yn = int(Xn), int(Yn)
    Thetan = 180 * (Thetan) / 3.14
    return (Xn, Yn, Thetan), D

'''
------------------------------------------------------------------
Get Neighbours
------------------------------------------------------------------
'''
def getGraph(coord,orientation,map_width,map_height,rpm1,rpm2,ObstacleList): 
    
    obs_set = set(ObstacleList)
    costs = {}
    actions = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]
    cost=0
    rpm=[]
    for action in actions:
         coord_neighbor,cost =actionSet(coord[0],coord[1],orientation, action[0],action[1])     
         if coord_neighbor[0]>=map_width or coord_neighbor[1]>=map_height or (coord_neighbor[0],coord_neighbor[1]) not in obs_set:
             costs[(coord_neighbor[0],coord_neighbor[1],coord_neighbor[2])]=cost
             rpm.append((action[0],action[1]))
         
        

        
    return costs,rpm     





def A_Star(start,goal,rpm1,rpm2,map_width,map_height,ObstacleList):
    
        
    cost_list = {}
    closed_list = []
    
    #Contains the parent node and the cost taken to reach the current node
    parent_index = {}
    print("start Point :",start)
    print("Goal Point : ",goal)
    visited_regions = np.zeros((map_width*2,map_height*2))
    cost_list[start]=0
    open_list = [(0,start)]
    Goal_Reached = False
    count=0
    rpm_closed_list=[]
    loading = threading.Thread(target=animate_A_star)
    loading.start()
    global is_loading_a_star
    #Converting to set for faster checking 
    obs_set = set(ObstacleList)
    
    while len(open_list)>0 and Goal_Reached == False:
        #1print("Open List : ",open_list)
        count = count+1
        totalC, parent_coord = heapq.heappop(open_list) 
        parent_position = (parent_coord[0],parent_coord[1])
        orientation = parent_coord[2]
        
        
        neighbours,rpm = getGraph(parent_position,orientation,map_width,map_height,rpm1,rpm2,obstacle_scaled)
        rpm_count=-1
        #print(neighbours)
        if parent_position not in obs_set:
            for key, cost in neighbours.items():
                cost_list[key]=math.inf
                
            for coord, cost in neighbours.items():
                rpm_count+=1
                if(coord not in  closed_list) and (coord not in ObstacleList) and visited_regions[round(coord[0])*2][round(coord[1]*2)]==0:
                    visited_regions[round(coord[0])*2][round(coord[1]*2)]=1
                    coord_round = (round(coord[0]),round(coord[1]),coord[2])
                    closed_list.append(coord_round)
                    rpm_closed_list.append(rpm[rpm_count])
                    Cost2Come = cost 
                    Cost2Go = math. dist((coord[0],coord[1]),(goal[0],goal[1]))  # h(n)
                    TotalCost = Cost2Come + Cost2Go   # f(n)
                    
                    if TotalCost < cost_list[coord] or coord not in open_list :
                        
                        parent_index[coord_round]={}
                        parent_index[coord_round][TotalCost] = (parent_coord,rpm[rpm_count])
                        
                       
                        
                        cost_list[coord_round]=TotalCost
                        heapq.heappush(open_list, (TotalCost, coord_round))
                    #The thersold is set according to the step size to reach closest to goal with few steps
                    if ((coord_round[0]-goal[0])**2 + (coord_round[1]-goal[1])**2 <= (1.5)*2) and (coord_round[2]>goal[2]-30 and coord_round[2]<goal[2]+30) :
                        print("\nFinal Node :",coord_round)
                        print('GOAL  Reached !!')
                        print("Total Cost :  ",TotalCost)
                        Goal_Reached = True
                        time.sleep(0)
                        is_loading_a_star = True
                        return parent_index,closed_list,coord_round,rpm_closed_list,True

                    
    return parent_index,closed_list,coord_round,rpm_closed_list,False
                    
"""
 ----------------------------------------------------------------
Backtracking
 ---------------------------------------------------------------- 
 """               

def get_Backtrack(parent_index,goal,start):
    back_track = []
    current= start
    back_track.append(current)
    is_goal_reached = False
    rpm_backtrack_list=[]
    #For loading icon
    global is_loading_backtrack
    loading = threading.Thread(target=animate_Backtrack)
    loading.start()
    while is_goal_reached == False:
        for coord,parent_cost in parent_index.items():
            for cost,parent in parent_cost.items():
                if coord==current:
                    #rint("parent: ",parent[0])
                    if parent[0] not in back_track:
                        back_track.append(current)
                        rpm_backtrack_list.append(parent[1])
                        
                    current = parent[0]
                    if parent[0] == goal:
                        is_goal_reached = True
                        time.sleep(1)
                        is_loading_backtrack = True
                        
                        break
    back_track.append(goal)
    return back_track,rpm_backtrack_list
    
    



"""
----------------------------------------------------------------
Visualization 
---------------------------------------------------------------- 
"""

def visualize_map(map_width,map_height,obstacle_scaled,obstacle_cord,closed_list,back_track_coord,rpm_backtrack_list):

    obstacle_map = np.zeros((map_width*2+1,map_height*2+1,3),np.uint8) 
    obstacle_map[obstacle_scaled*2]=255
    obs_set = set(obstacle_scaled)
       
    pygame.init()
    gameDisplay = pygame.display.set_mode((map_width*2,map_height*2))
    pygame.surfarray.make_surface(obstacle_map)
    pygame.display.set_caption('A Star Algorithm')
    
    gameDisplay.fill((0,0,0))
    
    #Adding obstacle to animation
    for coords in obs_set:
        pygame.draw.rect(gameDisplay, (255,0,255), [coords[0]*2 ,abs(250-coords[1])*2,1,1])
        pygame.display.flip()
      
    

    fig, ax = plt.subplots()
    
    
    for child,costdict in parent_index.items():
    
        for cost, parent_and_rpm in costdict.items():
            Xi = parent_and_rpm[0][0]
            Yi = parent_and_rpm[0][1]
            Thetai = parent_and_rpm[0][2]
            UL = parent_and_rpm[1][0]
            UR = parent_and_rpm[1][1]
            t = 0
            r = 0.038*100
            L = 0.34*100
            dt = 0.1
            Xn=Xi
            Yn=Yi
            Thetan = 3.14 * Thetai / 180
            UL = (2*np.pi* UL)/60
            UR = ( 2*np.pi * UR)/60

            while t<1:
                t = t + dt
                Xs = Xn
                Ys = Yn
                Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
                Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
                Thetan += (r / L) * (UR - UL) * dt
                if ((int(Xn),int(Yn)) not in obs_set):
                    plt.plot([Xs, Xn], [Ys, Yn], color="blue")
                    pygame.draw.rect(gameDisplay, (0,255,255), [int(Xn)*2 ,abs(250-int(Yn))*2,1,1])
                    pygame.display.flip()
            Thetan = 180 * (Thetan) / 3.14
              
    optimal = []  
    for coord,rpm in zip(back_track_coord[::-1],rpm_backtrack_list[::-1]):
    
        Xi = coord[0]
        Yi = coord[1]
        Thetai = coord[2]
        UL = rpm[0]
        UR = rpm[1]
        t = 0
        r = 0.038*100
        L = 0.34*100
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
        UL = (2 * np.pi * UL)/60
        UR = (2 * np.pi * UR)/60

        while t<1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            if ((int(Xn),int(Yn)) not in obs_set):
                plt.plot([Xs, Xn], [Ys, Yn], color="red")
            optimal.append((Xn,Yn))
    
            
        Thetan = 180 * (Thetan) / 3.14
    
        
    for X,Y in optimal[::-1]: 
        pygame.draw.rect(gameDisplay, (0,0,255), [int(X)*2 ,abs(250-int(Y))*2,1,1])
        pygame.display.flip()
    
        
    pygame.quit()        
    plt.grid()
    
    ax.set_aspect('equal')
    
    
    plt.title('Plot of path',fontsize=10)
    
    plt.show()
    plt.close()   
 
    


"""
----------------------------------------------------------------
Main Function
---------------------------------------------------------------- 
"""




map_width=600
map_height=250
start = (11,11,0)
goal = (400,100,60)
rpm1=16
rpm2 =16
rpm_backtrack_list=[]
back_track_coord=[]
x_s=0
y_s=0

orientation_s=0
isGoal = False

try:

    while True: 
        Clearance = int(input("Please Enter Clearance :"))
        
        obstacle_scaled,obstacle_cord =obstacle_space(600,200,Clearance)
        #Showing the map
        x, y = [], []
        for i in obstacle_scaled:
            x.append(i[0])
            y.append(i[1])
        plt.scatter(x,y,s=0.1,c='red')
        plt.axis([0,600,0,200])
        plt.title('Obstacles Map')
        plt.grid(which='both')
        plt.show()

        x_s = int(input("Please enter the x coordinate of start  : "))
        y_s = int(input("Please enter the y coordinate of start  : "))
        orientation_s =int(input("Please enter the orientation of start  : "))
        x_g = int(input("Please enter the x coordinate of goal  : "))
        y_g = int(input("Please enter the y coordinate of goal  : "))
        orientation_g =int(input("Please enter the orientation of goal  : "))
        rpm1 = int(input("Please enter the left wheel rpm  : "))
        rpm2 = int(input("Please enter the right wheel rpm  : "))
     
        #print("RPM is fixed to be 17 in this case for smooth movement of Turtlebot in Gazebo")
        

       
        if(x_s>=map_width or x_g>=map_width or y_g>=map_height or y_g>=map_height or x_s<0 or x_g<0 or y_g<0 or y_g<0):
            print("Please enter a value for x betweenn 0-599 and y between 0-200")
            continue
        
        elif((x_s,y_s)  in obstacle_cord ) or ((x_g,y_g) in obstacle_cord ):
            print("The input entered is on the obstacle point, Please enter a valid input")
            continue

        else: 
            start = (x_s,y_s,orientation_s) 
            goal =  (x_g,y_g,orientation_g)
            break
    #since the triangle is obstructing the path to reach we are keeping a threshold for x (Please refer the map )     
  
    start_time = time.time()
    parent_index,closed_list,goal_new,rpm_closed_list,isGoal = A_Star(start,goal,rpm1,rpm2,map_width,map_height,obstacle_scaled)
    
    
    if(isGoal):
        
        print("\nTime to Find Path: ",time.time() - start_time, "seconds (Warning : Actual time can be less since the loading animation can add more time" )
        back_track_coord,rpm_backtrack_list =get_Backtrack(parent_index,start,goal_new)
        visualize_map(map_width, map_height, obstacle_scaled, obstacle_cord, closed_list, back_track_coord,rpm_backtrack_list)

    else:
        print("Bactracking cannot be done")

    

    
except Exception as e:
    print("You have entered an invalid output please Run the program again")
    print(e)
print("Program Executed ")

