author Boyu Zhang & Xiaoyu Zhang

all the work in done in docker tiago



1. put the models and world file from tmc_wrs_gazebo_world into ./workspace/src/pal_gazebo_worlds (due to the size limitation, i didn't upload this package in my zip file)

1. 1. put the world and map from file worlds_and_maps into ./workspace/src/pal_gazebo_worlds

2. put the map folder into $(env HOME)/.pal/tiago_maps/configurations/
   alternative: rosrun map_server map_server src/t5_navi/maps/wrs2020_cola3/map.yaml

3. build the packages and source
    object_detection
    object_labeling
    plane_segmentation
    tiago_move
    t5_navi
    t5_grasp_drop
    t5_task_manager


before starting the program, i commited out the launch


4. please prepare 

    4. 1. roscore

    4. 2. roslaunch t5_navi t5_navi.launch 
    4. 2. 1. incase the map doesn't topup properly, and the rviz doesn't run properly, use the map_server, use the command below
             rosrun map_server map_server src/t5_navi/maps/wrs2020_cola3/map.yaml

    4. 3. roslaunch object_detection object_detection.launch 

    4. 4. rosrun t5_task_manager task_manager.py

    to show the status: 
    4. 5. rosrun smach_viewer smach_viewer.py





5. if the procedure doesn't run properly, alternative: 


5. 1. video


5. 2. pull the images

I uploaded the image into docker hub, all procedure are set properly, don't need to build the workspace and souce, you can run the command directly without any trouble

foode258/tiago_yolo:t5test
size: ~11.5G
this one includes all the packages that needed, but without nvidia cuda support
run t5_navi t5_task_manager as menshened above


foode258/yolo_ros:v3
size:~9G
this image is specific for yolo, run object_detection in this image



docker pull foode258/tiago_yolo:t5test
docker pull foode258/yolo_ros:v3



rocker --nvidia --x11 --privileged \
    --volume <~>
    --network host \
    --name core_image \
    tiago_yolo:t5test


rocker --nvidia --x11 --privileged \
    --volume <~>
    --network host \
    --name yolo_image \
    foode258/yolo_ros:v3


in core_image:
run 4. 1. ~ 4. 5. 

for fast yolo detection
in yolo_image:
run 4. 3. 