# itri_driver_tutorial
The tutorial about how to use ITRI ROS driver.

## Dependencies
- [ROS Kinetic or Melodic](http://wiki.ros.org/ROS/Installation)
- [MoveIt!](https://moveit.ros.org/install/)
- ...

## Usage
### RAS Setting
Set **"Port Number"** to **11000**.

Change to **"0: [Ethernet Server] 命令通訊模式"**.

Then press **"連線"**.

As shown in the following figure:

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/ras_setting.png "Logo 標題文字 1")

### ITRI Driver Setting
####

## Example
Run the following command to connect to ITRI robot:
### Use MoveIt! to do motion planning
```
roslaunch itri_ar607_moveit_config moveit_planning_execution.launch robot_ip:=<your_robot_ip>
```
### Plan trajectory by your own algorithms
```
roslaunch itri_driver itri_interface_streaming.launch robot_ip:=<your_robot_ip>
```
```<your_robot_ip>``` is the IP of your ITRI robot controller. You can enter **ipconfig** in cmd to search the IP.

## Troubleshooting
### ...
