 Note:
When opening a new terminal, the environment is reset, and the ```~/.bashrc``` is sourced. You'll need to add some environment setup files to the ~/.bashrc:
```
    $  sudo gedit ~/.bashrc
    # Add "source /home/pyzhu/catkin_ws/devel/setup.bash" to the file 
```
or just manually re-source them
```
source /home/pyzhu/catkin_ws/devel/setup.bash
```

# Graph concepts
- Nodes: an executable file. Can communicate with other nodes by ROS
- Message: ROS data type. Used when subscribing or publishing to a topic
- Topics: Nodes can publish messages to a topic / subscribe to a topic to receive messages
- Master: Nodes manager. ROS name service (e.g.helps nodes to find each other)
- rosout: equivalent of stdout / stderr in ROS
- roscore: Master + rosout + parameter server
# Nodes
- executable file in ROS
- uses ROS client libray to communicate with other nodes
- can publish / subscribe to a Topic
- can provide or use a service
# Client libraries
- allows nodes written in different programming languages to communicate
    - rospy = python client library
    - roscpp = c++ client library
# roscore
- run before using ROS
    ```
    $  roscore
    ```
    If roscore does not initialize: probably network configuration issue ->  [Network Setup](http://wiki.ros.org/ROS/NetworkSetup#Single_machine_configuration)
    If roscore does not initialize & sends a message about lack of permissions: probably the ~/.ros folder is owned by root -> change recursively the ownership of that folder with
    ```
    $  sudo chown -R <your_username> ~/.ros
    ```
# Using rosnode

- rosnode displays the nodes information that are currently running.
    - rosnode list: lists active nodes. If running
        ```
        $  rosnode list
        ```
        the result shows
        ```
        /rosout
        ```
        this is because rosout is always running as it collects and logs debugging output.
    - rosnode info <node name>: returns information about a specific node.
        ```
        $  rosnode info /rosout
        ```
        ```
        ------------------------------------------------------------------------
        Node [/rosout]
        Publications:
         * /rosout_agg [rosgraph_msgs/Log]
        
        Subscriptions:
         * /rosout [unknown type]
        
        Services:
         * /rosout/set_logger_level
         * /rosout/get_loggers
        
        contacting node http://machine_name:54614/ ...
        Pid: 5092
        ```
# rosrun
- Allows you to run a node directly with the packge name.
    ```
    $  rosrun [package_name] [node_name]
    ```
- ROS can use [Remapping Argument](http://wiki.ros.org/Remapping%20Arguments) to change the node's name:
    ```
    $  rosrun turtlesim turtlesim_node __name:=my_turtle
    ```
    then, when running ```rosnode list```, if you still see /turtlesim in the list, it means 
    - you closed it by Ctrl-C rather than close the window
    - or, you don't have $ROS_HOSTNAME environment variable [Network Setup](http://wiki.ros.org/ROS/NetworkSetup#Single_machine_configuration)

    you can clean the rosnode list by
    ```
    $  rosnode cleanup
    ```