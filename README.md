# itri_driver_tutorial
The tutorial about how to use ITRI ROS driver.

## Dependencies
- [ROS Kinetic or Melodic](http://wiki.ros.org/ROS/Installation)
- [MoveIt!](https://moveit.ros.org/install/)
- [industrial_core](https://github.com/ros-industrial/industrial_core)
- ...

## Usage
### RAS Setting
Set **"Port Number"** to **11000**.

Change to **"0: [Ethernet Server] 命令通訊模式"**.

Then press **"連線"**.

As shown in the following figure:

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/ras_setting.png "RAS interface")

### ITRI Driver Setting
#### Action name & Controller joint names & IP address & Port number
In ```itri_driver/config``` directory, create your own **configuration file**. It's **very important**!

Inside the file, specify the **namespace**, **controller name**, **controller joint names**, **IP address**, and **port number** as shown below.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/config_file.png "config file")

Remember to include this configuraion file in ```itri_driver/launch/itri_interface_streaming.launch```.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/load_rosparam.png "include config file")

#### Local ip
In ```itri_driver/src/ras_client.cpp```, there is a variable which specifies the ip of your local computer.

You can enter **ifconfig** in terminal to search.

Ensure it is set correctly, or the feedback of robot states from ITRI controller could not be received.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/local_ip.png "local ip")

If you forget to set the right local ip, you may see the following phenomenon.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/timeout_error.png "timeout err")

#### Update rate of robot states
If you want to change the rate which the commands are sent to robot controller to get robot states, adjust the argumet passed to the constructor of ```ros::Rate```.

The following code snippet is in ```itri_driver/src/itri_state_interface.cpp```. It can be up to **150 Hz**.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/update_states_rate.png "update rate")

Remember to **catkin_make** or **catkin build** before launching the driver if you do any modifications.

## Example
Run the following command to connect to ITRI robot:
### Use MoveIt! to do motion planning
#### AR605
```
roslaunch itri_ar607_moveit_config moveit_planning_execution.launch
```
#### SJ705
```
roslaunch dars_sj705_right_moveit_config moveit_planning_execution.launch
```
#### Dual SJ705
```
roslaunch dars_sj705_right_moveit_config moveit_planning_execution_multi.launch
```
### Plan trajectory by your own algorithms
```
roslaunch itri_driver itri_interface_streaming.launch
```

## Troubleshooting
### If you have any problems, feel free to open an issue.
