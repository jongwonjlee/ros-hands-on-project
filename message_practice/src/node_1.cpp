#include "ros/ros.h"                        // ROS Default Header File
#include <std_msgs/Int16.h>                 // ROS built-in msg header file to generate my own message, which is going to be published

int main(int argc, char **argv)             // Node main function
{
  ros::init(argc, argv, "Node1");           // Initializes node name as "Node1"
  ros::NodeHandle nh;                       // Node handle declaration for communication with ROS system

  // Declare and create publisher 'ros_number_pub' which publishes message type of 'std_msgs::Int16'.
  // The name of topic to be published is 'number' and the size of the publisher queue is set to 10.
  ros::Publisher ros_number_pub = nh.advertise<std_msgs::Int16>("number", 10);

  ros::Rate loop_rate(10);                  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals

  std_msgs::Int16 msg;                      // Declares message 'msg' in 'std_msgs::Int16' message file format
  int count = 0;                            // Variable to be used in message

  while (ros::ok())                         // ros::ok() will return false if: a SIGINT is received (Ctrl-C), ros::shutdown() has been called, all ros::NodeHandles have been destroyed and so on.
  {
    msg.data  = count;                      // Save the the int type variable, 'count', in the data of 'msg'.
    ros_number_pub.publish(msg);            // Publish the message 'msg' by the Publisher 'ros_number_pub'
    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above (10Hz).
    ++count;                                // Increase the variable 'count' by one for every loop
  }

  return 0;
}
