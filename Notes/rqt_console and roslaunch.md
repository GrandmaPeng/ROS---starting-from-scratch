**rqt_console** and **rqt_logger_level** are used for debugging
**roslaunch** is used for starting many nodes at ones
To follow this tutorial, rqt and turtlesim packages are required:

    $ sudo apt-get install ros-<distro>-rqt ros-<distro>-rqt-common-plugins ros-<distro>-turtlesim
# rqt_console and rqt_logger_level
**rqt_console** belongs to ROS logging framework. It prints the output from nodes. **rqt_logger_level** allows you to change the verbosity level (DEBUG, WARN, INFO and ERROR) of nodes.
Run these two commands in two new terminals:
```
$ rosrun rqt_console rqt_console
```
    $ rosrun rqt_logger_level rqt_logger_level
The two windows look like:
![rqt_console](http://ww1.sinaimg.cn/large/c2a9265fly1fsqvwmimtmj20ot0hyq4k.jpg)
![rqt_logger_level](http://ww1.sinaimg.cn/large/c2a9265fly1fsqvxfcjw4j20os07ct9p.jpg)    

Now, run a turtlesim in a new terminal:

    $ rosrun turtlesim turtlesim_node
    
Then the console window shows a record of "spawn" with the severity "INFO". By default, the logger level is INFO. It can be set in the rqt_logger_level by:
![logger_level](http://ww1.sinaimg.cn/large/c2a9265fly1fsqw2kcw1kj20wm09ktam.jpg)
Try running:

    $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

Check the console window, "Oh no! I hit the wall!" with severity "Warn" shows up.
**Explanations:**
Logger level priority:
```
Fatal
Error
Warn
Info
Debug
```
in which "Fatal" is the highest. When you set the level to be "Warn", in the console window you will get "Warn", "Error" and "Fatal" logging messages.
Press Ctrl-C. Next let's move to roslaunch.
# roslaunch
**roslaunch** starts nodes as defined in a launch file.

    roslaunch [package] [filename.launch]
Now we will use it to start mutiple turtlesim nodes, and make one node mimic the other. cd to **beginner_tutorials**:
    
    $ roscd beginner_tutorials
make a launch directory:
    
    $ mkdir launch
    $ cd launch
>Note: it is not necessary to name it as "launch" or even make a directory. roslaunch automatically looks in to the passed packages and detects available launch files.
## Launch file
Create a launch file named "turtlemimic.launch" as below:
```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```
**Explainations**:
- <launch>: this tag indentify a launch file
- <group>: here, 2 groups with namespace "turtlesim1" and "turtlesim2" start with a turtlesim node with a name of sim respectively. Two simulators can start without having a name conflicts.
    ```
      <group ns="turtlesim1">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>
    
      <group ns="turtlesim2">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>
    ```
- <node> and <remap>: here, we start a mimic node with the topis **input** and **output** renamed to turtlesim1 and turtlesim2. This will cause tsim2 to mimic tsim1
    ```
      <node pkg="turtlesim" name="mimic" type="mimic">
        <remap from="input" to="turtlesim1/turtle1"/>
        <remap from="output" to="turtlesim2/turtle1"/>
      </node>
    ```
## start roslaunch
roslaunch the launch file by:

    $ roslaunch beginner_tutorials turtlemimic.launch
Then two turtlesims start. In a new terminal, use rostopic to send a speed setting message only to turtlesim1:

    $ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
You will see two turtlesims move at the same time.
Open rqt_graph to have a better understanding:

    $ rqt_graph
    
![rqt_graph](http://ww1.sinaimg.cn/large/c2a9265fly1fsqyyp59bhj217l06i0td.jpg)
    
    
    
    
