# UR5 with robotiq-2f-85 and realsense in Gazebo
This project is a simulation environment for UR5, robotiq-2f-85, and realsense D435.     
- ROS Distribution: noeitc(Ubuntu focal-20.04)    
- Gazebo version: 11     
## Requriments:   
Before you use this project, you need to install the following ROS package in advance.   
- Install MoveIt!:    
    ```
    sudo apt updata
    sudo apt install ros-noetic-moveit
    ```
- Install RViz   
    ```
    sudo apt install ros-noetic-rviz
    ```
- Install Controller_manager
    ```
    sudo apt install ros-noetic-controller-manager
    ```
## How to use this repository    
- Create a workspace
    ```
    mkdir -p ~/ur_ws/src
    cd ~/ur_ws/src
    git clone https://github.com/chpppy/UR5_with_Robotiq_and_RealsenseD435_in_Gazebo.git    
    ```
- Build the code under `~/ur_ws/`    
    ```
    cd ~/ur_ws/
    catkin_make
    source devel/setup.bash
    ```

- Run the code with ROS and Gazebo.     
    ```
    roslaunch hyrobot main.launch use_rviz:=true use_moveit:=true
    ```
## How to achieve visual grasping
### Use MoveIt! to manipulate the UR5
- Use [MoveIt!](https://moveit.github.io/moveit_tutorials/) to plan and execute the trajectory.
- Use [C++ Interface](https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp) or [Python Interface](https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py)
### Use realsense D435
- Subscribe the color topic and the depth topic of D435 to get image raw.
- Align the depth image with the color image.   
    ![Method](Align.png)     
(Depth_x, Depth_y) means the pixel position in depth image raw, (Color_x, Color_y) means the pixel position in color image raw.
