#include "ros/ros.h"                    // ROS Default Header File
#include "string.h"                     // Defines several functions to manipulate C strings and arrays.
#include <geometry_msgs/Twist.h>        // header file to use geometry_msgs::Twist message type
#include <nav_msgs/Odometry.h>          // header file to use nav_msgs::Odometry message type
#include <tf/tf.h>                      // header file to use tf in ROS.
#include <cmath>                        // Declare a set of functions to compute common mathematical operations and transformations (e.g. sin, cos, M_PI...)
#include <iostream>                     // Perform both input and output operations.

geometry_msgs::Twist msg_move;          // Declare a global, geometry_msgs::Twist type ROS message variable that is going to be published as a movement command.
ros::Publisher cmd_pub;                 // Declare a global ROS publisher.

using namespace std;                    // Use namespace of std. It lessens our effort to write down all std::'s, such as std::string, std::vector and so on.


// Since comparing two double variables are not recommended in C++,
// the comparison with an epsilon value is what most people do.
// To this end, I defined a function which compares two double-type variables with a given offset value.
bool AreSame(double a, double b, double EPSILON = 0.01)
{
    /*
     * input arguments: a, b (double type variables to be compared), EPSILON (threshold. default = 0.01)
     * return value: true (if |a-b| < EPSILON), false (else)
    */
    return fabs(a - b) < EPSILON;
}

// Callback function. It recieves a nav_msgs::Odometry type message as a parameter.
void pose_cb(nav_msgs::Odometry msg){

    static int reach = 0; // count the order of waypoints that a robot is heading toward.

    // msg.pose.pose.position : has x,y,z as instances (3d coordinates with respect to the point where the robot starts from)
    // msg.pose.pose.orientation : has x,y,z,w as instances (quaternion)
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;

    // Declare a tf::Quaternion type variable q. The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 and Transform.
    tf::Quaternion q(msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y,
                                     msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w);


    tf::Matrix3x3 m(q);                 // Declare a tf::Matrix3x3 type variable m, which is derived from the quarternion q.
    double roll, pitch, yaw;            // Variables where euler angles converted from the quaternion q will be saved.
    m.getRPY(roll,pitch,yaw);           // convert from quaternion to RPY

    double waypoint[4][2] = {{1, 1}, {4, 0}, {2, -1}, {0,0}};       // array of waypoints

    double x_goal = waypoint[reach][0];                 // Goal position where the robot is going to.
    double y_goal = waypoint[reach][1];

    if(AreSame(x, x_goal) && AreSame(y, y_goal)){       // If the robot is approximately at the goal position,
        reach = (reach+1)%4;                            // increase the order of waypoints that a robot is heading toward by 1. (Note that reach repeatedly changes from 0 to 3.)
        cout << "A goal waypoint has been changed from (" << x_goal << ", " << y_goal << ") to (" << waypoint[reach][0] << ", " << waypoint[reach][1] << ")" << endl;   // Announcement that the aiming waypoint has changed.
        return;                                         // Terminate this callback function.
    }

    double dx = x_goal -x, dy = y_goal - y;             // Difference between the current point and the goal point.

    double comm_theta = (fmod((atan2(dy, dx) - yaw)+M_PI,2*M_PI)-M_PI)*2;   // Rotational velocity based on feedback control. It is proportional to the difference between the angle which the robot has to achieve and that the robot is heading now.
    double comm_x = max(0.2, min(sqrt(pow(dx, 2.0)+pow(dy, 2.0)), 0.5));    // Linear velocity based on feedback control. It is proportional to the distance between the goal and the current position. Note that minimum value is given as 0.2, and the maximum is 0.5

    cout << "comm_theta: " << comm_theta << ", comm_x: " << comm_x << endl;     // Print out two command velocity values.

    msg_move.linear.x = comm_x; // Assign comm_x on the linear.x value of msg_move. (linear movement)
    msg_move.angular.z = comm_theta; // Assign comm_theta on the angular.z value of msg_move. (rotational movement)

    cmd_pub.publish(msg_move);

}


// The node's main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kobuki_controller");     // Initializes Node Name as "kobuki_controller"
    ros::NodeHandle nh;                             // Node handle declaration for communication with ROS system

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);      // Declare a publisher. It publishes geometry_msgs::Twist type message named "/mobile_base/commands/velocity" with a single queue size.
    ros::Subscriber robot_pos_sub = nh.subscribe("/odom", 1, pose_cb);                      // Declare a subscriber which subscribes a topic named "/odom" and pass it into the callback function named 'pose_cb'. The size of the message queue is 1.


    while(ros::ok()){                   // ros::ok() will return false if: a SIGINT is received (Ctrl-C), ros::shutdown() has been called, all ros::NodeHandles have been destroyed and so on.
            ros::Rate loop_rate(10);    // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals.
            ros::spinOnce();            // ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
            loop_rate.sleep();          // Pause this loop. Consequently, the loop works at an interval of 10Hz.
    }

    return 0;
}

