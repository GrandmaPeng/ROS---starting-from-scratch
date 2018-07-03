# Writing a service Node
## Code
Create the add_two_ints_server.cpp file.
```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```
## Code explanation
```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
```
The AddTwoInts header file is generated from the srv file.
```cpp
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
```
This function provides the service to add two ints. int values are acquired from request, and the retured value is loaded in to response. These datatype are defined in the srv file, and the function returns a boolean.
```cpp
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
```
Two ints are added and the result is stored in the response. Then some information are logged. The service returns true when it is completed.
```cpp
ros::ServiceServer service = n.advertiseService("add_two_ints", add);
```
Here the service is created and advertised over ROS.
# Writing the client node
## Code
Create an add_two_ints_client.cpp file.
```cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```
## Code explanation
```cpp
ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
```
This creates a client for the add_two_ints service. ros::ServiceClient object is used to call the service.
```cpp
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
```
Instantiate an auto generated service class with its request member value assigned. One service class contains two members, **request** and **response**. It also contains two class definitions, **Request** and **Response**.
```cpp
if (client.call(srv))
```
This calls the service. Since the service calls are blocking (模态过程) It will return once the call is done. If the service call succeeded, call() returns true, and the srv.response value is valid. Vice versa.

# Building nodes
Add the following into the CMakeList.txt:
```cpp
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
```
```
catkin_make
```
# Running the nodes
## Running the server
    $ rosrun beginner_tutorials add_two_ints_server
    
    [ INFO] [1530576564.842265967]: Ready to add two ints.
## Running the client
    $ rosrun beginner_tutorials add_two_ints_client 1 4
    
    [ INFO] [1530576631.699515904]: Sum: 5

