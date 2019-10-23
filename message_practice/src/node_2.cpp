#include "ros/ros.h"                            // ROS Default Header File
#include <std_msgs/Int16.h>                     // ROS built-in msg header file to generate my own message, which is going to be published
#include "message_practice/number_oddeven.h"           // Include the header file in order to declare the user-defined message "number_oddeven". According to CMakeLists.txt, this header file is automatically created in the directory of ~/catkin_ws/devel/include/PKG_NAME after catkin_make.
#include <boost/bind.hpp>                       // Include the header file inside the boost c++ library to use the function 'bind'. The function 'bind' is able to bind any argument to a specific value or route input arguments into arbitrary positions.


// Message callback function. This function is called when a message named "number" is received, as descibed in main function.
// As an input variable, the pointer of 'std_msgs::Int16' type message is received. The pointer of the ros publisher also comes in so that we can declare the message "number_oddeven" by using this.
void msgCallback(const std_msgs::Int16::ConstPtr& msg_ptr, const ros::Publisher* pub_ptr)
{

  message_practice::number_oddeven send_data;          // Declares a message 'send' following the 'message_practice::number_oddeven' message file format. It consists of 'int16 count' and 'string oddeven'.

  send_data.count = msg_ptr->data;              // Save the the data from the recieved message in the member variable called 'count'. In fact, the data delievered by 'msg_ptr' is the number of count from Node1, in the int type.

  // Indicate whether the recieved count number is even or odd. Then, save the result in the member variable called 'oddeven', as a type of string.
  // Also, print out whether the number is even or odd. Note that ROS_INFO almost works as same as printf().
  if(msg_ptr->data % 2 == 0){
      ROS_INFO("even");
      send_data.oddeven =  "even";
  }
  else if(msg_ptr->data % 2 == 1){
      ROS_INFO("odd");
      send_data.oddeven =  "odd";
  }

  pub_ptr->publish(send_data);                  // By using the pointer of the ros publisher, the message 'send_data' is published.
}



int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "Node2");                       // Initializes Node Name as "Node2"
  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system

  // Declare and create a publisher 'ros_number_oddeven_pub' which publishes message type of 'message_practice::number_oddeven'.
  // The name of topic to be published is 'number_oddeven' and the size of the publisher queue is set to 10.
  ros::Publisher ros_number_oddeven_pub = nh.advertise<message_practice::number_oddeven>("number_oddeven", 10);

  // Declare and create subscriber 'ros_number_sub' which subscribes message type of 'std_msgs::Int16'.
  // The subsciber hears the topic named "number", and the size of the message queue as 10.
  // Once 'ros_number_sub' recieves a message named "number", it passes the message's pointer in boost::bind(msgCallback, _1, &ros_number_oddeven_pub).
  // Consequently, boost::bind function binds them as msgCallback(msg_ptr, pub_ptr), and the callback function runs.

  // NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed,
  // it will automatically unsubscribe from the topic called "number".
  ros::Subscriber ros_number_sub = nh.subscribe<std_msgs::Int16>("number", 10, boost::bind(msgCallback, _1, &ros_number_oddeven_pub));

  ros::spin();                                          // ros::spin() will enter a loop, pumping callbacks. It will exit when Ctrl-C is pressed, or the node is shutdown by the master.


  return 0;

}
