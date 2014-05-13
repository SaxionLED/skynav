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

- Install ros-hydro-joy  
 	_$ sudo apt-get ros-hydro-joy_  

- Setup .bashrc  
Source the workspace  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;___source ~/catkin_ws/devel/setup.bash___  
Set gazebo model path for offline gazebo models in the model database  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;___export GAZEBO_MODEL_PATH=~/catkin_ws/src/x80sv/x80sv_simulation/models/___  

- Setup serial ports to right setup and add useraccount to 'dialout' group   
 	_$ useradd -G dialout [USER]_  

- Git clone skynav and x80sv in /home/[USER]/Git/  
	for the navigation software: _$ git clone https://github.com/SaxionLED/skynav.git_  
 	for the x80sv robot drivers: _$ git clone https://github.com/SaxionLED/x80sv.git_  


- Create symbolic link of skynav and x80sv folder in /home/[USER]/catkin_ws/src-  

- Run _$ catkin_make_  

- Connect robot via ttyUSB0 and connect laser,  or run simulator in gazebo  

- Launch __skynav.launch__  
