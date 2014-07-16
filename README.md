skynav
======

container of the skynav navigation system

[![Build Status](https://travis-ci.org/SaxionLED/skynav.svg?branch=master)](https://travis-ci.org/SaxionLED/skynav)


Dependencies

- Install ros-hydro-desktop-full, and this package will build.





-------------------
quick install notes:
-------------------
- Install ubuntu 12.04-LTS  
_http://www.ubuntu.com/_

- Install ros hydro  
_http://www.ros.org/install/_  
_http://wiki.ros.org/hydro/Installation/Ubuntu_

- Setup catkin workspace _/home/[USER]/catkin_ws_  
_http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment_  

- Setup .bashrc  
Source the workspace  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;___source ~/catkin_ws/devel/setup.bash___  

- Git clone skynav and x80sv in the catkin_ws/src folder,  
	_(or somewhere else and create a symbolic link to the files in the catkin_ws/src folder.)  
  
	for the navigation software: _$ git clone https://github.com/SaxionLED/skynav.git_  
	for the x80sv robot drivers: _$ git clone https://github.com/SaxionLED/x80sv.git_  
  
- Setup serial ports to right setup and add useraccount to 'dialout' group   
 	_$ useradd -G dialout [USER]_    
  
- Export the files from __x80sv/x80sv_driver/udev__ folder to __/etc/udev/rules.d__   
	  for resolving the robot specific USB connections

- Run _:~/catkin_workspace $ catkin_make_ to build the project  
- Run _:~/catkin_workspace $ catkin_make run_tests_ to build and run the gtests and rostests  

- Connect the robot and laser via usb,  or run a robot simulator in gazebo  

- Run _:~/catkin_workspace $ roslaunch x80sv_bringup skynav_real_robot.launch_ 
	to launch the full skynav navigation package including gmapping and x80 drivers
- or run _:~/catkin_workspace $ roslaunch skynav_bringup skynav_launch_ for running only the skynav stack
