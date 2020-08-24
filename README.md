--------------------------------------------------------------------------------------
			ASSIGNMENT 5: Final Project
--------------------------------------------------------------------------------------
Group members: 

1. Shubham Horane
2. Rahil Modi
3. Manikanda Balaji
4. Chase Gurcan
5. Akshay Hole
--------------------------------------------------------------------------------------
Three videos describing each task of this assignment can be found in the following path:
/src/turtlebot3_auefinals_pkg/videos

The launch files can be found in the path: 
/src/turtlebot3_auefinals_pkg/launch

The python script files can be found in the path: 
/src/turtlebot3_auefinals_pkg/nodes

--------------------------------------------------------------------------------------
NOTE: CAMERA ADDITION TO TURTLEBOT3 BURGER IN GAZEBO

Camera is not available by default in the TurtleBot burger model defined in Gazebo. The process to add camera to this model has been outlined in the readme.md file of the assignment 5 package in this same repository.

--------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------
This assignment package has one launch file for all required tasks:
----------------------------------------------------------------------------------------

filename: "final_launch.launch"

- This file initializes the following nodes:

1. The auefinal world
2. The standing person which will be used for leg tracking
3. The person teleop window
4. Leg detection package
5. Tiny yolo package
6. Main integrated python script for the requirements of the final project: 
NOTE: This componenet of the launch file has been commented to allow for the world and the several nodes load up correctly. This node can be run manually after everything has loaded to ensure that the bot starts moving only after it can be seen in the world.
7. hector slam navigation package (arg value can be changed to run the other packages if they are installed in your system)


- The command for executing the above launch file is:

$ roslaunch turtlebot3_auefinals_pkg final_launch.launch 

- The command to run the integrated node is : 

$ rosrun turtlebot3_auefinals_pkg aue_final.py 

*** - The two videos in the videos folder show the overall functionality of the code. 
----------------------------------------------------------------------------------------

Work was divided between two sub-groups within our team:

sub-group #1: Shubham Horane & Rahil Modi
- Stop Sign Detection
- World setup
- Line following 


sub-group #2: Chase Gurcan, Manikanda Balaji, & Akshay Hole.
- Leg tracking
- Wall following and obstacle avoidance 
- Slam methods (hector and gmapping)

The entire team worked on code integration and final tuning of parameters of the code.
----------------------------------------------------------------------------------------












