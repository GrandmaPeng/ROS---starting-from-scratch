Services are another way that nodes can communicate with each other. It allows nodes to send a request and receive a response.
# ROS service
**rosservice** can easily use the services provided by ROS client/service framework.
```
rosservice list         list active services
rosservice call         call the services with provaided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```
## rosservice list
    $ rosservice list
turtlesim provides 9 services: clear, kill, reset, spawn, turtle1/set_pen, /turtle1/teleport_absolute, /turtle1/teleport_relative, turtlesim/get_loggers, and turtlesim/set_logger_level.
rosout provides /rosout/get_loggers and /rosout/set_logger_level.
## rosservice type
    rosservice type [service]
check the "clear" service type

    $ rosservice type clear

    std_srvs/Empty
"Empty" means the service needs no arguments when it is called.
## rosservice call
    rosservice call [service] [args]
try calling "clear"

    $ rosservice call clear
It clears the trace of the turtle. Now, have a look at the spawn service:

    $ rosservice type spawn | rossrv show
    
    float32 x
    float32 y
    float32 theta
    string name
    ---
    string name
the "name" is optional. If the name is not given, turtlesim creates one. If calling 

    $ rosservice call spawn 2 2 0.2 ""
a new turtle with the specified coordinate and orientation will be created.
# rosparam
**rosparam** allows you to store and manipulate data on ROS [Parameter Server](http://wiki.ros.org/Parameter%20Server). It uses YAML syntax.
```
examples:
integer:    1
float:      1.0
string:     one
boolean:    true
list:       [1, 2, 3]
dictionary: {a: b, c: d}
```
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           write parameters to file
rosparam delete         delete param
rosparam list           list params
```
## rosparam list
    $ rosparam list
    
    /background_b
    /background_g
    /background_r
    /roslaunch/uris/aqy:51932
    /run_id
## rosparam set/get
    rosparam set [param_name]
    rosparam get [param_name]
Change the red channel of the background color

    $ rosparam set /background_r 150
This command modified the param value. We need to call "clear" service to take effect:

    $ rosservice call clear

We can use **rosparam get** to get the parameter value:

    $ rosparam get /
    
    background_b: 255
    background_g: 86
    background_r: 150
    roslaunch:
      uris: {'aqy:51932': 'http://aqy:51932/'}
    run_id: e07ea71e-98df-11de-8875-001b21201aa8
## rosparam dump/load
    rosparam dump [file_name]
    rosparam load [file_name] [namespace]
Now let's write all the params into params.yaml file:

    $ rosparam dump params.yaml
You can load the yaml file into **new namespace**, e.g. "copy" namespace

    $ rosparam load params.yaml copy
    $ rosparam get copy/background_b
    
    255
