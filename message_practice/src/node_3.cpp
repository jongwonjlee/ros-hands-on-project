#include "ros/ros.h"                            // ROS Default Header File
#include "me489_hw2/number_oddeven.h"           // Include header file in order to utilize the user-defined message 'number_oddeven'. According to CMakeLists.txt, this header file is automatically created in the directory of ~/catkin_ws/devel/include/PKG_NAME after catkin_make.


// Message callback function. This function is called when a message named "number_oddeven" is received, as descibed in main function.
// As an input variable, the pointer of 'me489_hw2::number_oddeven' type message is received.
void msgCallback(const me489_hw2::number_oddeven::ConstPtr& msg_ptr)
{
  // Print out the sentence to indicate the contents of the message "number_oddeven" has been recieved well. Note that ROS_INFO almost works as same as printf().
  // Since the subscribed message type of 'me489_hw2::number_oddeven' has two member variables: 'int16 count' and 'string oddeven', both of them should be printed out.
  // Keep in mind that string type message should be converted into char* type message to be printed out. The function c_str() does this.

  ROS_INFO("%d is an %s number.", msg_ptr->count, (msg_ptr->oddeven).c_str());
}


int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "Node3");                       // Initializes Node Name as "Node2"

  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system

  // Declare a subscriber 'ros_number_oddeven_sub' which subscribes message type of 'me489_hw2::number_oddeven' and pass it into the callback function named 'msgCallback'.
  // The subsciber hears the topic named "number_oddeven", and the size of the message queue as 10.
  ros::Subscriber ros_number_oddeven_sub = nh.subscribe<me489_hw2::number_oddeven>("number_oddeven", 10, msgCallback);

  ros::spin();                                          // ros::spin() will enter a loop, pumping callbacks. It will exit when Ctrl-C is pressed, or the node is shutdown by the master.

  return 0;
}
