## Start docker
docker-compose up -d quadruped
docker-compose up -d 

## Stop
docker-compose stop quadruped

## Execute commands in docker
docker-compose exec -it quadruped bash

## Activate GUI
xhost +

## Build my docker
bash build.sh

## Remove it 
docker rmi quadruped:latest

# For freecad :
conda activate freecad_1_0_312 && freecad
conda deactivate

Pair A (These legs move together):
    Front Left Shoulder: Pin 13
    Front Left Knee: Pin 14
    Back Right Shoulder: Pin 15
    Back Right Knee: Pin 16

Pair B (These legs move together):
    Front Right Shoulder: Pin 17
    Front Right Knee: Pin 18
    Back Left Shoulder: Pin 19
    Back Left Knee: Pin 21


ros2 run quadruped_basics dashboard.py

# Launch robot :
ros2 launch quadruped_basics display.launch.py
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0
RESET esp32
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true

## If robot not connecting :
1) ros2 daemon stop && ros2 daemon start && ros2 topic list
2) sudo fuser -v /dev/ttyUSB0 && sudo killall -9 micro_ros_agent


# Launch seq for autonomus nav :
ros2 launch quadruped_basics sim.launch.py
ros2 run quadruped_basics ik_node.py --ros-args -p use_sim_time:=true
nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/ros/ros2_ws/src/quadruped_basics/maps/third_better_map.yaml params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_nav2.yaml
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
(Don't forget to put origin 2D pose)

# Launch for mapping :
ros2 launch quadruped_basics sim.launch.py
ros2 run quadruped_basics ik_node.py --ros-args -p use_sim_time:=true
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/ros/ros2_ws/src/quadruped_basics/config/my_slam_params.yaml use_sim_time:=true
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
## Save map :
ros2 run nav2_map_server map_saver_cli -f /home/ros/ros2_ws/src/quadruped_project/src/quadruped_basics/maps/my_first_map


Weight :
Foot : 7
Leg : 4
MG 90s : 14
Body : 73
Lidar : 46
esp32 : 10
Battery : 140
UBEC 6A : 15
UBEC 3A : 10

7 * 4 + 4 * 4 + 14 * 8 + 73 + 46 + 10 + 140 + 15 + 10 = 450
Main body : 14 * 4 + 73 + 10 + 140 + 15 + 10 = 304 + lidar ?
