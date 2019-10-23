#include "ros/ros.h"                            // ROS Default Header File
#include <std_msgs/Int16.h>                     // ROS built-in msg header file to generate my own message, which is going to be published
#include "message_practice/number_oddeven.h"           // Include the header file in order to declare the user-defined message "number_oddeven". According to CMakeLists.txt, this header file is automatically created in the directory of ~/catkin_ws/devel/include/PKG_NAME after catkin_make.


// Declare 1) a message to be republished, 2) a subscriber for the message "number", 3) and a publisher for the message "number_oddeven". Note that all of them are defined as a global variable.
message_practice::number_oddeven send_data;
ros::Subscriber ros_number_sub;
ros::Publisher ros_number_oddeven_pub;


// Message callback function. This function is called when a message named "number" is received, as descibed in main function.
// As an input variable, the pointer of 'std_msgs::Int16' type message is received.
void msgCallback(const std_msgs::Int16::ConstPtr& msg_ptr)
{

  send_data.count = msg_ptr->data;              // Save the the data from the recieved message in the member variable called 'count'. In fact, the data delievered by 'msg_ptr' is the number of count, in the int type.

  // Indicate whether the recieved count number is even or odd. Then, save the result in the member variable called 'oddeven', as a type of string.
  // Then, print out whether the number is even or odd. Note that ROS_INFO almost works as same as printf().
  if(msg_ptr->data % 2 == 0){
      ROS_INFO("even");
      send_data.oddeven =  "even";
  }
  else if(msg_ptr->data % 2 == 1){
      ROS_INFO("odd");
      send_data.oddeven =  "odd";
  }

  ros_number_oddeven_pub.publish(send_data);                  // By using the global variable of the ros publisher 'ros_number_oddeven_pub', the message 'send_data' is published as named as "number_oddeven".

}

int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "Node2");                       // Initializes Node Name as "Node2"
  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system


  // Declare a publisher 'ros_number_oddeven_pub' which publishes message type of 'message_practice::number_oddeven', which is named "number_oddeven".
  // Also, declare a subscriber 'ros_number_sub' which subscribes message type of 'std_msgs::Int16'.
  // The subsciber hears the topic named "number", and the size of the message queue as 10.
  ros_number_oddeven_pub = nh.advertise<message_practice::number_oddeven>("number_oddeven", 10);
  ros_number_sub = nh.subscribe<std_msgs::Int16>("number", 10, msgCallback);

  ros::spin();                                          // ros::spin() will enter a loop, pumping callbacks. It will exit when Ctrl-C is pressed, or the node is shutdown by the master.

  return 0;

}
