# Autonomous-Delivery-Bot
ROS and Raspberry Pi based autonomous delivery robot which can be used inside an controlled environment. Status - Alpha

## Software requirements

### Operating system
Tested on Ubuntu 18.04

### Robotic Operating System
Ros Melodic

### Webots
Webots is a free and open-source 3D robot simulator.
Version used: 2021a

#### Installation procedure
1. Download **webots_2021a_amd64.deb** from [here.](https://github.com/cyberbotics/webots/releases/tag/R2021a)
2. In the terminal execute  
    ```
    sudo apt install ./webots_2022a_amd64.deb
    ```

## Setting up a local repository
1. Create a ros package in the catkin workspace
    ```
    cd ~/<catkin_workspace_name>/src
    ```
    ```
    catkin_create_pkg autonomous-delivery-bot std_msgs rospy roscpp
    ```
    ```
    cd ~/<catkin_workspace_name>
    ```
    And then execute ```catkin build``` or ```catkin_make``` command.

2. Initialise git <br />
Make sure you have configured git using the commands.
    ```
    git config --global user.email "you@example.com"
    git config --global user.name "your name"
    ```
    Initialise git in the local repository by executing the following commands.
    ```
    cd ~/<catkin_workspace_name>/src/autonomous-delivery-bot
    ```
    ```
    git init
    ```
    ```
    git add .
    ```
    ```
    git commit -m "Created ros package" .
    ```
    Rename master branch to main
    ```
    git branch -m main
    ```

3.  Connect local repository to remote
    ```
    git remote add origin https://github.com/MIT-Mentors/autonomous-delivery-bot.git
    ```
    ```
    git pull origin main --allow-unrelated-histories
    ```
    
## To run the code
Open the terminal and execute
```
roscore
```
In another terimnal window execute the following command to run the webots simulation
```
roscd autonomous-delivery-bot
webots worlds/city.wbt
```
To run the ros node, in an another terminal execute  
```
rosrun autonomous-delivery-bot main.py
```