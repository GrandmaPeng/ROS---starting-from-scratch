# Example: Use keyboard to drive the turtle around
    $ rosrun turtlesim turtlesim_node
    $ rosrun turtlesim turtle_teleop_key
now you can control the turtle by the keyboard. Let's have a look at how it works.
# ROS Topic
- The turtlesim_node and the turtle_teleop_key node are communicating with each other over a ROS Topic.
- turtle_teleop_key publishes the "key strokes" on a Topic, while turtlesim_node subscribes to the same Topic.
## rqt_graph
It creates a dynamic graph of the system. You will need to download rqt package first, as rqt_graph is a part of the rqt:

    $ sudo apt-get install ros-<distro>-rqt
    $ sudo apt-get install ros-<distro>-rqt-common-plugins
In a new terminal, run

    $  rosrun rqt_graph rqt_graph
and put the mouse over the Topic (which is /turtle1/command_velocity), colors shows to identify the node (blue/green) and the topic (red)
![SystemGraph](http://ww1.sinaimg.cn/large/c2a9265fly1fspqndiabyj20wf096taa.jpg)
## rostopic
Check the sub-commands for rostopic:

    $ rostopic -h
```
    rostopic bw     display bandwidth used by topic
    rostopic echo   print messages to screen
    rostopic hz     display publishing rate of topic
    rostopic list   print information about active topics
    rostopic pub    publish data to topic
    rostopic type   print topic type
```
### rostopic echo
- print the data published on a topic. Run
    ```
    # rostopic echo [topic]
    $ rostopic echo /turtle1/command_velocity
    ```
    and press keyboard to drive the turtle, data shows.
- now, run rqt_graph again, you'll see rostopic echo (red) subscribes to the cmd_vel Topic as well.

### rostopic list
- lists all the Topics currently subscribed to and published.
    use "verbose":
    ```    
    $rostopic list -v
    ```
# ROS Messages
- Communications on Topics happens by sending ROS messages beteem nodes.
- Publisher and subscriber must send and receive the same type of message. This means a Topic type is defined by the message type published on it.
## rostopic type
- Use ```rostopic type``` to check the message type of a certain Topic
    ```
    $ rostopic type [topic]
    ```
    run:
    ```
    $ rostopic type /turtle1/command_velocity
    ```
    you should get:
    ```
    geometry_msgs/Twist
    ```
    To find the details of the message, use rosmsg
    ```
    $ rosmsg show geometry_msgs/Twist
    ```
    get:
    ```
    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
    ```
Now we've found the type of message that the nodes expects. We can publish commands to the turtle.
# Publish commands
## rostopic pub
**rostopic pub** publishes data on the currently advertised Topic.

    # rostopic pub [topic] [msg_type] [args]
    $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
- rostopic pub： publish message to a given topic
- -1: causes rostopic to **only publish one message then exit**
- /turtle1/cmd_vel: name of the Topic that is published to
- geometry_msgs/Twist: the message type to use when publishing to the topic
- --(double dash): tells the parser that none of the following arguments is an option(command). e.g.minors(-)in the arguments
- arguments: following [YAML](http://wiki.ros.org/ROS/YAMLCommandLine) syntax

The turtle needs a steady stream of commands at 1Hz to keep moving. This can be done by ```rostopic pub -r```:

    $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
- -r: publishing rate(Hz)
## rostopic hz
**rostopics hz** reports the rate at which data is published.

    # rostopic hz [topic]
    $ rostopic hz /turtle1/pose
(turtlesim is publishing data to the turtle at the rate of about 60Hz)
# rqt_plot
**rqt_plot** displays a scrolling time plot of data published on Topics.
```
    $ rosrun rqt_plot rqt_plot  
```
A new window pops out. Input **/turtle1/pose/x** -> + and **/turtle1/pose/y** -> +, x-y plot shows.
- pose: position. (x,y,theta)
    