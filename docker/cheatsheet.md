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


If ghost topic :
ros2 daemon stop
ros2 daemon start
ros2 topic list

If esp32 not connecting plug the battery after esp32 start

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
