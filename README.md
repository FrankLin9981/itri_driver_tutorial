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

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/ras_setting.png "RAS interface")

### ITRI Driver Setting
#### Action name
In the main function of ```itri_driver/src/itri_trajectory/action.cpp```, initial the **action servers** with the correspoding action name.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/action_name.png "action name")

#### Update rate of robot states
If you want to change the rate which the commands are sent to robot controller to get robot states, adjust the argumet passed to the constructor of ```ros::Rate```.

The following code snippet is in ```itri_driver/src/itri_state_interface.cpp```.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/update_states_rate.png "update rate")

#### Local ip
In ```itri_driver/src/ras_client.cpp```, there is a variable which specifies the ip of your local computer.

You can enter **ifconfig** in terminal to search.

Ensure it is set correctly, or the feedback of robot states from ITRI controller could not be received.

![alt 文字](https://github.com/FrankLin9981/itri_driver_tutorial/blob/master/images/local_ip.png "local ip")

Remember to **catkin_make** or **catkin build** before launching the driver if you do any modifications.

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
