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