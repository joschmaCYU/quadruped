# My quadruped project (find a name ?)
(Photo of the robot)

## Short presentation
This repo is for my quadruped robot (code, 3D files, etc.). This walking robot can autonomously navigate thanks to Nav2 and SLAM. I built this with ROS 2 Jazzy, running on an ESP32 and a Raspberry Pi.  
My goal is to show examples of how to make your own autonomous quadruped with ROS 2.
## What it can do
https://github.com/user-attachments/assets/2a4f5992-fd1c-4fa5-9b9c-b5832c5c24f9
## The chalenge of making a walking robot
It's friction and foot slippage. Even in simulation, the robot rarely moves exactly as commanded, causing odometry to not represent the real robot position.

# Tutorial (🚧 work in progress 🚧):
Follow these steps to build your own quadruped !  

### 0 - What will my robot do ?
The first question you have to ask your self and the most important one is: *what will my robot do ?*  
For me, I want my robot to autonomously navigate a semi-controlled environement.

### 1 - Parts
Now that you have defined your goals you need to pick your parts.
- Compute: Raspberry Pi 5 (Main ROS 2 brain) and ESP32 (Servo controller)
- Actuators: 8x MG90S Micro Servos
- Sensors: 2D LiDAR (for SLAM/Navigation)
For more details see (parts)[https://github.com/joschmaCYU/quadruped/blob/main/PartsREADME.md]

> (PartsREADME.md file):
### 1 - Parts
As stated before we will be using ROS 2 but it needs power to run ! Thats why you will need something powerfull like a raspberry pi 5. To communicate with the servos and other actuators I choose an ESP32. You could plug everything but the rasp has only 4 PWM pins and you can't just plug the servos to any GPIO pins else they will not act as you want.  
To sens the world I choose a 2D lidar (for SLAM/Navigation). This will mesure how far away the obstacles are.
> [!TIP]
> You don't need to place the lidar low to the ground because you will be able to move your legs up, so even if the lidar doesn't detect the obstacle your robot will still be able to pass over it.

The power side is much more strait forward. If you want to move multiple servos at the same time your boards will not provide sufficiant power you will need a battery. I choose a small 2200mAh LiPo Battery. The servos need 5v to operate you need to make sure to provide these 5v to much and your servo will burn and to little they will not move. So you will need a 5V/6A UBEC which regulates the voltage and has a max current of 6A
> [!WARNING]
> If your servo use more then 6A be sure to take a more powerfull UBEC
And the rasp needs also 5V but will not pull more then 3A so I took a 5V/3A UBEC for it!

- Sketch of my electronic (TODO)

> [!TIP]
> You can add some other sensors like an IMU

With all of that in mind we can begging with
### 2 - Print & Assemble
Now that you know what we have to fit in our robot lets design it. Use your favorite CAD software and create your URDF file.  
For more help see (print)[https://github.com/joschmaCYU/quadruped/blob/main/PrintREADME.md]

> (PrintREADME.md file):
### 2 - Bringing it to your computer

#### 2.1 - Getting the idea
So you want to design your robot, I have a few tips for you.  
Beggin by (and I strongly advise you to do so) take a pencil and a sheet of paper and try to draw your robot!  

- Sktech of my robot (TODO)

This will help you refine your idea. You will have to ask yourself many questions about how it will move, its height, its length, etc.  
You can take inspiration from other robots! I took great inspiration of (sesame-robot)[https://github.com/dorianborian/sesame-robot/tree/main].  
> [!WARNING]
> There are multiple types of walking robots, bipedal (2 legs), qudruped, hexapod ect...
> There are 2 general type of leg position: mammalian and reptilian.
For mammalian the legs are under the body, like a horse or a dog.  
For reptilian the legs are on the side, like a spider or a crocodile (that's what I went with)  
> [!TIP]
> Because the MG90S servos are weak, the chassis must be as lightweight as possible.
My robot walks like a spider robot. It has 8-DOF.  

#### 2.2 - The design
Now this is the part where you have to take your sketchs and make them in a CAD. Iterate as many time until you are satisfied. *No need to print it yet.*  

- Image of my robot in CAD (TODO)

#### 2.3 - Creating my urdf file
##### 2.3.1 - What is an urdf file ?
It's simple! It's a file that describes your robot. This file will be used by your simulation software to make the robot move. Rather then having just a fixed 3d the urdf file specifies how parts move/rotate along side each other.  
##### 2.3.2 - How to make it
Either you rebuild your robot in an urdf software like (D-Robotics)[https://urdf.d-robotics.cc] or (Lever Robotics)[https://lever-robotics.github.io/URDF_creator/]. This can be done quick and dirty but can be less precise.  
Or you use your newly modeled robot to genereate it. (here)[https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Exporting-an-URDF-File.html] is a tutorial for ROS on how to export your CAD file to an URDF file.  
> [!TIP]
> You will have to specifyl how each part move/rotates this can be tidius but you will have the exact replicate of your robot in the sim !

Speaking of which :  
### 3 - Simulating the robot and let's use ROS
The robot runs on ROS 2 Jazzy, handling the communication between the sensors, the Pi, and the ESP32. (Let's bring your robot to sim)[https://github.com/joschmaCYU/quadruped/blob/main/SimREADME.md]  

> (SimREADME.md file):
### 3 - Simulating the robot
#### 3.1 - What is ROS and what will we be using ROS for ?
The most simple explanation I can give is : ROS is like whatsapp a messaging app but the messages are information. We will be using ROS to benefit from it's great echo system (simulators, autonomus navigation, mapping...).  
> [!TIP]
> If you have never used ROS you should begging with getting familiar to it with (tutorials)[https://docs.ros.org/en/jazzy/Tutorials.html] !
#### 3.2 - Set up urdf + sim
You can find a tutorial to do so (here)[https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics]. I will not detail this part which is outside of this tutorial scope.  

#### 3.2 - Making the robot move
To make the robot move we will use inverse kinematics and then use 3d IK to move over obstacles.
If you don't want this I am sure you can find some pre-built frameworks like ros2_control walking plugins to do the job for you but here we will create our own !  
1) The upper leg (L1​) is permanently sticking straight out horizontally.
2) The knee joint tilts the lower leg (L2​) outward to control the robot's height.
3) The shoulder joint acts as a "Yaw" hinge, sweeping the entire leg forward and backward like a door to control the stride.
(sketch how the robot will move to explain the math)  

```
def calculate_ik(self, x, z):
        # 1. Physical Leg Lengths
        L1 = 0.206  # length of upper leg
        L2 = 0.250  # length of lower leg

        # 3. KNEE MATH (Controls Height)
        # Because your URDF draws the calf pointing straight down when angle is 0. A larger angle bends it outwards.
        cos_knee = abs(z) / L2
        cos_knee = max(0.0, min(1.0, cos_knee))
        knee_angle = math.acos(cos_knee)

        # 4. SHOULDER MATH (Controls Forward Stride)
        # Calculate how far out the foot currently is due to the knee bend
        horizontal_reach = L1 + (L2 * math.sin(knee_angle))

        # Calculate the sweep angle required to move 'x' meters forward
        step_reach = x / horizontal_reach
        step_reach = max(-1.0, min(1.0, step_reach))
        shoulder_angle = math.asin(step_reach)

        return shoulder_angle, knee_angle
```
The math isn't very advanced but you need to take your time to assimilate it  

#### 3.3 - IK gait
```
 def get_ik_gait(self, t, phase_offset, step_scale):
        T = 0.80
        duty_factor = 0.60
        cycle_progress = ((t / T) + phase_offset) % 1.0

        # --- SPIDER GAIT SETTINGS ---
        stride_length = 0.25  # 15cm max steps
        step_height = 0.08    # Lift foot 8cm into the air
        stand_height = -0.25  # Keep the hip 20cm off the floor (Safe for L2=25cm)

        if cycle_progress < duty_factor:
            # STANCE PHASE
            stance_p = cycle_progress / duty_factor
            target_x = (stride_length / 2.0) - (stride_length * stance_p)
            target_z = stand_height
        else:
            # SWING PHASE
            swing_p = (cycle_progress - duty_factor) / (1.0 - duty_factor)
            target_x = -(stride_length / 2.0) + (stride_length * swing_p)
            target_z = stand_height + (step_height * math.sin(swing_p * math.pi))

        # SCALE THE PHYSICAL STEP (Not the joint angle!)
        target_x = target_x * step_scale

        return self.calculate_ik(target_x, target_z)

```

#### 3.4 - Odom
```
        speed_multiplier = 0.08247
        turn_multiplier = 0.6

        self.odom_yaw += (self.cmd_w * turn_multiplier) * self.dt
        self.odom_yaw = math.atan2(math.sin(self.odom_yaw), math.cos(self.odom_yaw))
        self.odom_x += (self.cmd_x * speed_multiplier * math.cos(self.odom_yaw)) * self.dt
        self.odom_y += (self.cmd_x * speed_multiplier * math.sin(self.odom_yaw)) * self.dt
```


### 4 - Autonomus navigation
#### 4.1 - AMCL
#### 4.2 - SLAM
### 5 - Bulding the robot
### 6 - Sim to life

<details>
<summary>FAQ</summary>
Should I program in Python or C++ ? <br> At the start for quick iteration you can use python and if the performance requires it switch to c++. <br><br>
</details>
