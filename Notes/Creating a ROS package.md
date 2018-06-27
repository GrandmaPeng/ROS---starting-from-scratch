A catkin package:
- contains a catkin compliant package.xml file. It provides meta information about the package.
- contains a CMakeLists.txt.
- has its own folder.
# Creating a catkin package
## 1. Creating a workspace
## 2. Create a new package
    $ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
Use **catkin_create_pkg** script to create a new package named **beginner_tutorials** with the dependency **std_msgs**,  **rospy** and **roscpp**
Now, the beginner_tutorials folder should be created with a package.xml and a  CMakeList.txt in it.
## 3. Build the packages in the catkin workspace
    $ cd ~/catkin_ws
    $ catkin_make
## 4. Add the workspace to your ROS environment
    $ . ~/catkin_ws/devel/setup.bash
# Customizing your package
## 1. Customizing the package.xml
### (1) description tag 
    5   <description>The beginner_tutorials package</description>
### (2) Maintainer tags - allows others to contact with the package related staff
    7   <!-- One maintainer tag required, multiple allowed, one person per tag --> 
    8   <!-- Example:  -->
    9   <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
    10  <maintainer email="user@todo.todo">user</maintainer>
### (3) License tags
    12   <!-- One license tag required, multiple allowed, one license per tag -->
    13   <!-- Commonly used license strings: -->
    14   <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
    15   <license>TODO</license>
BSD is used in the tutorial because the rest of the core ROS components use it already
### (4)Dependencies tags - describe the dependencies of the package
In the previous operation, std_msgs, roscpp and rospy are passed as the arguments to catkin_create_pkg. Hence the dependencies are:

    38   <buildtool_depend>catkin</buildtool_depend>
    39   <build_depend>roscpp</build_depend>
    40   <build_depend>rospy</build_depend>
    41   <build_depend>std_msgs</build_depend>
Since we want all of the specified dependencies to be available at build and run time, we'll add them to ```run_depend``` tag

    18   <exec_depend>roscpp</exec_depend>
    19   <exec_depend>rospy</exec_depend>
    20   <exec_depend>std_msgs</exec_depend>