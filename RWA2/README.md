# RWA 2 #

## Introduction 
  	This file includes python program solution for Real World Application 1 for
	UMD ENPM 809E Spring 2021 batch. The folders consists of python based node and launch file. 
	
	These files are executable:
  rwa2_shah.launch
  my_bot_controller
  
## Requirements
       ***Important************************************************
       *It is required that rwa2 module is in the catkin workspace*
       ************************************************************
       
	
### Running code in ubuntu
After making sure that the path of the rwa2 module is compiled using catkin build command
Make sure that current working derectory is same as the directory of program
You can change the working derectory by using **cd** command
* Run the following command which will compile the package
````
catkin build rwa2_shah
````
* Run folloing line to launch the bot at (-2, 0) location
````
roslaunch rwa2_shah rwa2_shah.launch
````
* Run following command to move robot to specified node
````
rosrun rwa2_shah my_bot_controller {NODE}
````

### Troubleshooting ###
	Most commonly the issue will be with the Gazebo simulation Close all the terminals of ROS and restart them to solve majority of issues.
	For issues that you may encounter which can't be solved with simple restart, create an issue on GitHub.
  
### Maintainers ###
	Mrugesh Shah (mrugesh.shah92@gmail.com)
