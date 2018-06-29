# Introduction of msg and srv
- msg: describes the ROS message fileds. They are used to generate source code for message in different languages.
- srv: one srv describes one service. It consists of **request** and **response**.
- msg files are stored in /msg in the package; srv files are stored in /srv.
- msg files declare a fieled type and field name at each line. Available field type:
    - int8, int16, int32, int64 (+ uint*)
    - float32, float64
    - string
    - time, duration
    - other msg files
    - variable-length array[] and fixed-length array[C]

There is a special field type: **Header**. It contains the timestamp and coordinate information. Frequently you will see declaration ```Header header``` at the first line in an msg file. Here is an example:
```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```
In srv file, request and response are divided by "- - -". Here is an example:
```
int64 A
int64 B
---
int64 Sum
```
# msg
## Creating a msg
Define a new message in the previous package:
```
$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```
Then an msg file is created with a line "int64 num".
*Important step!* we need to ensure the msg file is converted to the source code of C++, Python, etc. Check the package.xml, ensure there are:

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
At the build time, we need "message_generation"; at the runtime, we need "message_runtime".
In the CMakeLists.txt, add the "message_generation" dependency to the "find_package". You can add message_generation simply in the COMPONENTS list.
    
    find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
>Note: There can only be one find_package in the CMakeLists file.

Also the runtime dependency:
```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
and:
```
add_message_files(
  FILES
  Num.msg
)
```
Ensure the generate_messages() function is called:
```
generate_messages(
    DEPENDENCIES
    std_msgs
)
```
Now, you are ready to generate source files from your message definition.
## rosmsg
Use rosmsg to check if ROS can identify the message

    $ rosmsg show [message type]
```
$ rosmsg show beginner_tutorials/Num

int64 num
```
the package name here can be omitted.
# srv
## Creating an srv
```
$ roscd beginner_tutorials
$ mkdir srv
```
This time we copy a service from another package. It can be done by roscp:

    $ roscp [package_name] [file_to_copy_path] [copy_path]
```
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```
*Important step!* make sure srv file is converted to source code of C++, Python, etc.
Assume message_generation dependency has been added into CMakeLists (message_generation works both for srv and msg).
```
add_service_files(
  FILES
  AddTwoInts.srv
)
```
Now you can generate source files from your service definition.
## rossvr
Use **rosmsg show** to check if ROS identifies the service.

    # rossrv show <service type>
    
    $ rossrv show beginner_tutorials/AddTwoInts
    int64 a
    int64 b
    ---
    int64 sum

As it is in msg, you may not enter the package name:
    
    $ rossrv show AddTwoInts
    
    [beginner_tutorials/AddTwoInts]:
    int64 a
    int64 b
    ---
    int64 sum
    
    [rospy_tutorials/AddTwoInts]:
    int64 a
    int64 b
    ---
    int64 sum
    
# Rebuild the package
    
    $ catkin_make
All the .msg files will generate code for use in all supported languages.
The directories:
- C++ .h: ~/catkin_ws/devel/include/beginner_tutorials/
- Python: ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg 
- lisp: ~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/

# Review
Commands have been used so far:
- rospack = ros+pack(age) : provides information related to ROS packages
- rosstack = ros+stack : provides information related to ROS stacks
- roscd = ros+cd : changes directory to a ROS package or stack
- rosls = ros+ls : lists files in a ROS package
- roscp = ros+cp : copies files from/to a ROS package
- rosmsg = ros+msg : provides information related to ROS message definitions
- rossrv = ros+srv : provides information related to ROS service definitions
- rosmake = ros+make : makes (compiles) a ROS package



