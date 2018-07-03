[roscpp overview](http://wiki.ros.org/roscpp/Overview)
# Writing a publisher node
We're going to write a publisher ("talker") node, which will continually broadcast a message.
cd to the beginner_tutorials package path.
## The code
Create a src folder:
    mkdir -p ~/catkin_ws/src/beginner_tutorials/src
Create a src/talker.cpp file and copy the code below:
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
```
## Code explanation
```cpp
#include "ros/ros.h"
```
It includes all the headers necessary to use the most common public pieces of the ROS system.
```cpp
#include "std_msgs/String.h"
```
It includes [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) message. It is a header generated automatically from the String.msg file.
```cpp
ros::init(argc, argv, "talker");
```
Initialize ROS. It allows name-remapping through the command line. Node name "talker" must be unique in a running system.
```cpp
ros::NodeHandle n;
```
Create a handle to this process' node. The first created NodeHandle will do the initialization of nodes, and the last destructed one will cleanup all the nodes-occupied resources.
```cpp
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
This tells the master that we will **publish** a message of type **std_msgs::String** on the topic **chatter**. It lets the master tell any nodes listening on chatter that we're going to publish data now. 
**1000** is the size of the publishing queue. In this case, if the messages are sent with too high frequency, the previous published message will be thrown away when the buffer has more than 1000 messages.
**NodeHandle::advertise()** returns a ros::Publisher object, which serves two purposes: (1) it contains *publish()* method, which allows you to publish messages on a topic; (2)if the message type is wrong, it prevents to publish a message (unadvertise) automatically. 
```cpp
ros::Rate loop_rate(10);
```
**ros::Rate** allows you to set the loop frequency. It tracks how long has been since the last call to **Rate::sleep()**, and sleep for the correct amount of time. In this example it runs at 10Hz.
```cpp
  int count = 0;
  while (ros::ok())
  {
```
By default roscpp installs a SIGINT handler. it handles the Ctrl-C which makes ros::ok() returns false.
ros::ok() returns false if:
- Ctrl-C is received
- It is kicked out of the ROS network by another node with the same name
- ros::shutdown() is called by another part of the application
- all ros::NodeHandles have been destroyed
Once ros::ok() returns false, all ROS calls will fail.
```cpp
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
```
Broadcast a messageon ROS using a **message-adapted** class, generated from a msg file. In this example, we use the standard String message, which has only one member:"data". You can publish more complicated datatypes as well.
```cpp
chatter_pub.publish(msg);
```
Here, we broadcast the msg to whoever has subscribed the topic "chatter".
```cpp
ROS_INFO("%s", msg.data.c_str());
```
ROS_INFO can replace for printf/cout. See[rosconsole document](http://wiki.ros.org/rosconsole) for more information.
```cpp
ros::spinOnce();
```
In this example, it is not necessary to use ros::spinOnce(), since we are not receiving any callbacks. If you were to add a subscription into this application without ros::spinOnce(), your callbacks would never get called.
```cpp
loop_rate.sleep();
```
Use this to sleep for time remaining to let the publishing rate be 10 Hz.
### Summary
- Initialize the ROS system
- Advertise that we're going to be publishing **std_msgs/String** messages on the chatter topic to the master
- Loop while publishing message to chatter at the rate of 10 Hz
Now we need to write a node to receive the messages.
# Writing the subscriber node
## Code
Create the listener.cpp file and copy the code below:
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
```
## Code explanation
```cpp
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```
This is the callback function that will be called when a new message is received on the chatter topic. The message is passed in the form of [boost shared_ptr](https://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm), which means you can store it without copying.
```cpp
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```
Subscribe to the chatter topic with the master. ROS calls the callback function when a message is published to this topic. 
**1000** is the size of the queue.
**NodeHandle::subscribe()** returns a ros::subscriber object. You'll have to hold it on until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the chatter topic.
There are different types of NodeHandle::subscribe() function, which allows you to specify a class member function, even anything can bee called by Boost.Function object.
```cpp
ros::spin();
```
ros::spin enters a loop, calling message callbacks as fast as possible. If the no message arrives, it won't use much CPU. Once the ros::ok() returns false, ros::spin() will exit. 
### Summary
- Initialize the ROS system
- Subscribe to the chatter topic
- Spin, waiting for messages to arrive
- When a message arrives, the chatterCallback() function is called.
# Building the nodes
CMakeFileList.txt:
```cpp
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```
```cpp
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
```
This make sure the message headers are generated before use.

    $ catkin_make







