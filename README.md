# Path Planning for a mobile Robot and turtlebot3 with obstacle avoidance and non holonomic constraints using A-star Search Algorithm
Project - 03 (Phase 02) for the course ENPM661 - Planning for Autonomous Robots

## Team Members
- Mothish Raj Venkatesan Kumararaj (mr2997@umd.edu)   Directory ID: mr2997    UID: 117553727 


- Satish Vennapu (satish@umd.edu)    Directory ID : satish     UID : 118306759

## Project Description
Implement A* Algorithm to find a path between the start and end point on a given map for a mobile
robot and Turtlebot3 with non-holonomic constraints in ROS, Gazebo. 

* **A-star** : A* Search algorithm is one of the best and popular technique used in path-finding and graph traversals.




### Dependencies

* NumPy
* cv2
* Matplotlib
* math
* heapq
* pygame
* time
* itertools
* threading
* sys
* rospy
* geometry_msgs
* rospy
* geometry_msgs
* tf
* roslaunch
* os
* subprocess




### Steps to run the implementation


#### Part1:
1. Run the python file
2. The program prompts for user inputs and takes the user inputs from the console

#### Part2:
1. Clone the repository
2. Install latest version of Python and the required libraries mentinoned above prior to running the code
3. Add the ros package into the src folder of the catkin_ws and run the "catkin_make" command 
4. run "source devel\setup.bash"
5. Navigate to "Project 3 Phase2 part1\A_star_differential_drive\Part02\proj3_phase2_sample\src\src" and run the sourceCode.py
5. The program prompts for user inputs and takes the user inputs from the console
6. Exploration of nodes starts and the optimal path is displayed in 2D map and gazebo



 ```
 git clone  https://github.com/Mothish97/A_star_differential_drive.git

 catkin_make
 
 source devel\setup.bash

 python3 sourceCode.py
 ```
## Results

#### Part1:
https://user-images.githubusercontent.com/86384730/230561903-ae306d60-d9a3-43e7-9327-bb372613fedc.mp4

#### Part2:
https://user-images.githubusercontent.com/86384730/230561713-5957c35d-34fa-4075-b242-e64caf388cba.mp4




### Test Case for Part 1: 
Please enter the clearance: 10

Please enter the x-coordinate of start: 11

Please enter the y-coordinate of start: 11

Please enter the orientation of start: 0

Please enter the x-coordinate of goal: 400

Please enter the y-coordinate of goal: 25

Please enter the orientation of goal: =30

Please enter the left wheel rpm: 30

Please enter the right wheel rpm: 40

### Test Case  for Part 2: 

Please enter the clearance: 22

Please enter the x-coordinate of start: 100

Please enter the y-coordinate of start: 24

Please enter the orientation of start: 0

Please enter the x-coordinate of goal: 400

Please enter the y-coordinate of goal: 24

Please enter the orientation of goal: 0

Please enter the left wheel rpm: 50

Please enter the right wheel rpm: 50



### Constraints in the program
1. The sum of the clearance and the robot always equals to 22  for part 2
2. The map is scaled down by 10 and its in cms
3. Gazebo origin is located at the center and the robot spawning in our case is shifted to the bottom left corner



### Video URL

#### Part 1:

https://drive.google.com/file/d/1n5SmnlW9Tq08KdRU7UKI8Rp8X4Ycr2td/view?usp=share_link

#### Part 2:

https://drive.google.com/file/d/1yGhIJzva34Wc6-rlgw7rLKRfKO-JejxS/view?usp=share_link

