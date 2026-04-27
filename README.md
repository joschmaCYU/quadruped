# Quadruped
The code base for my quadruped robot. I build this with ros2 jazzy. On an esp32 and raspberry pi.  
My goal is to show example on how to make your own autonomus quadruped with ROS2.
  
Here is how I did it (work in progress) :
### 0 - What will my robot do ?
The first question you have to ask your self and the most important one is : what will my robot do ? For me, I want my robot to autonomously navigate a semi-controlled environement.
Then you have to pick your parts. I went with : a lidar (to see the world around me (for autonomus navigation), an esp32 (to communicate with the servos), a rasperrypi 5 (the main pc), 8 MG90S servos (2 in each leg), 2 ubec for power management (one 6a for all my servos and anotherone 3a 5v for the raspberry pi), a 2200 lipo mAh battery.  
  
With all of that i mind we can begging with
### 1 - Making the robot in FreeCAD
#### 1.1 - Getting the idea
Now that you know what we have to fit in our robot lets design it. Use your favorite CAD software. Because I took great inspiration of (sesame-robot)[https://github.com/dorianborian/sesame-robot/tree/main]. But I did not just copy and paste. I beggun by (and I strongly advise you to do so) taking a pencile and a sheet of paper and draw my robot. This will help you refine your idea. You will have to ask your self many question about how it will move it's height, it's length ect...   
#### 1.2 - The design
Now this is the part where you have to take your sketchs and make them in a CAD. I would begging with the main body but feel free to do what you want. Iterate as many time until you are satisfied  
### 2 - Creating my urdf file
#### 2.1 - What is an urdf file ?
It's simple ! It's a file that describes your robot. This file will be used by your simulation software to make the robot move. Rather then having just a fixed 3d the urdf file specifies how parts move/rotate along side each other. 
#### 2.2 - How to make it
Either you rebuild your robot in an urdf software like (1)[https://urdf.d-robotics.cc] or (2)[https://lever-robotics.github.io/URDF_creator/]. This can be done quick and dirty but is often less precise.    
Or you use your newly modeled robot to genereate it. As we will be using ROS (here)[https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Exporting-an-URDF-File.html] is a tutorial to export your CAD file to an URDF file. You will have to specifyl how each part move/rotates this can be tidius but you will have the exact replicate of your robot in the sim ! Speaking of which :  
### 3 - Simulating the robot (ROS)
#### 3.1 - What is ROS and what will we be using ROS for ?
The most simple explanation I can give is : ROS is like whatsapp a messaging app but the messages are information. We will be using ROS because it have a greate echo system of apps (simulators, autonomus navigation...)
#### 3.1 - Making the robot move with simple cos and sin
#### 3.2 - Autonomus navigation
##### 3.2.1 - AMCL
##### 3.2.2 - SLAM
### 4 - Bulding the robot
### 5 - Sim to life

Here is it walking and turning :

https://github.com/user-attachments/assets/2a4f5992-fd1c-4fa5-9b9c-b5832c5c24f9

