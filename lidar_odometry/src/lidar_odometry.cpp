#include <ros/ros.h>                        // ROS Default Header File
#include "string.h"                         // Defines several functions to manipulate C strings and arrays.
#include <cmath>                            // declares a set of functions to compute common mathematical operations and transformations (e.g. sin, cos, M_PI...).
#include <sensor_msgs/LaserScan.h>          // header file to use sensor_msgs::LaserScan message type
#include <vector>                           // header to use std::vector
#include <algorithm>                        // A collection of functions especially designed to be used on ranges of elements

#include <fstream>                          // Input/output stream class to operate on files.
#include <iostream>                         // Perform both input and output operations.

// Include pcl library - related headers. Pcl is a large scale, open project for 2D/3D image and point cloud processing.
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
//#include <tf/transform_broadcaster.h>     // Include it to use TransformBroadcaster to help make the task of publishing transforms easier in ROS.

using namespace std;                        // Use namespace of std. It lessens our effort to write down all std::'s, such as std::string, std::vector and so on.

/// Define global variables.

// Allocate a new pcl::PointCloud<pcl::PointXYZ> type variable and declare its pointer as prev_cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
// Eigen type matrices to save a matrix. Matrix4f denotes 4x4 matrix with float type elements.
Eigen::Matrix4f transMtx_now;
Eigen::Matrix4f transMtx_prev;
Eigen::Matrix4f transMtx_delta;

// Declare double-type array pos with a length of three.
double pos[3];

/// Callback function. It recieves a sensor_msgs::LaserScan type message (i.e. scanned data from 2d lidar) as a parameter.
void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian.
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;

    std::vector<float> range = msg.ranges;      // Range information is saved in msg as 1d array. So, declare a vector<float> type 'range' and same the range data into it.

    int len = range.size();                     // size of range vector (i.e. the number of range data)

    float angle_now;                            // A variable denotes the current angle that the program is looking probing.

    /// 1. LaserScan msg to PCL::PointXYZ

    // initialize pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new::pcl::PointCloud<pcl::PointXYZ>);    // Allocate a new pcl::PointCloud<pcl::PointXYZ> type variable and declare its pointer as new_cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;                                             // ExtractIndices extracts a set of indices from a point cloud.
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());                             // Declare a PointIndices::Ptr type variable 'inf_points'. Indices of points with infinite distance will be saved here.

    new_cloud->is_dense = false;        // new_cloud is not dense but sparse.
    new_cloud->width = len;             // Set new_cloud's width as len.
    new_cloud->height = 1;              // Set new_cloud's height as 1. (Remind that the input data 'msg' is from 2d lidar.)
    new_cloud->points.resize(len);      // Set the size of an instance named new_cloud->points as len. (Equal to that of vector<float> range)

    // fill the pointcloud
    for(int i = 0; i < len; i++){

        // TO DO START

        angle_now = angle_min + i*angle_increment;  // The current angle that the program is looking probing.

        // TO DO END

        if (std::isinf(range[i])==false){           // If a point being probed has finite distance, assign it as an element of pointcloud.

            // Assign an element of pointcloud. Its position has 3DOF(x,y,z). Each data is saved at i-th element of new_cloud->points.
            // Since the 2d lidar scans data from -PI (angle_min) to +PI (angle_max) from its frontward, I let that direction as y-axis of the sensor's coordinate and righthand side as x-axis.
            // TO DO START

            new_cloud->points[i].x = range[i]*cos(angle_now);      // x =  r*cos(theta) where theta is measured angle from the frontward direction.
            new_cloud->points[i].y = range[i]*sin(angle_now);       // y = + r*sin(theta) where theta is measured angle from the frontward direction.
            new_cloud->points[i].z = 0.0;                           // Simply let z as zero (Since there is no motion along z-direction)
            // TO DO END

        }
        else{                                       // Otherwise, if a point being probed has infinite distance, assign its indice as an element of inf_points.
            // indices of infinite distance points
            inf_points->indices.push_back(i);

        }
    }

    // Remove infinite distance points from new_cloud
    extract.setInputCloud(new_cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);                      // Set whether the indices should be returned, or all points _except_ the indices. The input argument set to be true if all points _except_ the input indices will be returned, false otherwise
    extract.filter(*new_cloud);                     // Filtered (i.e. elements which have infinite distance are filtered out) pointcloud is saved as new_cloud.

    /// 2. Get transformation between previous pointcloud and current pointcloud

    // transMtx_prev : transformation matrix at time (t-1)
    // transMtx_now : transformation matrix at time (t)
    // 4X4 transformation matrix (3X3: rotation matrix, 3X1: translation vector)

    if(prev_cloud->width == 0){                     // Initial condition. (If prev_cloud is not defined yet)

        // initialize transformation matrix. initial posiiton: x = 0, y = 0, theta = 0;
        transMtx_prev << cos(0), -sin(0), 0, 0,
                        sin(0), cos(0), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    }

    else{

        // ICP algorithm
        // http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
        // http://pointclouds.org/documentation/tutorials/interactive_icp.php#interactive-icp

        // Creates an instance of an IterativeClosestPoint called 'icp' and gives it some useful information.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_cloud);             // Set prev_cloud as an initial value.
        icp.setInputTarget(new_cloud);              // Set new_cloud as an final value.

        pcl::PointCloud<pcl::PointXYZ> Final;       // Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm.
        icp.align(Final);                           // Align the initial value(prev_cloud) and the final value(new_cloud) so that the trasformation from the initial value to the final value is calculated.
        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;    // If they are aligned correctly, icp.hasConverged() = 1 (true). It then outputs the fitness score of the final transformation.
        std::cout << icp.getFinalTransformation() << std::endl;     // Print transformational matrix from the initial value to the final value.

        transMtx_delta = icp.getFinalTransformation();              // Save it at a Matrix 'transMtx_delta'.

    /// 3. Get current transformation matrix using previous transformation and ICP result
    //  (Odometry calculation)

        // TO DO START

        transMtx_now =  transMtx_prev * transMtx_delta;

        // TO DO END

    // 4. Get current position from transformation matrix

        // TO DO START
        Eigen::Matrix3f R = transMtx_now.block<3,3>(0,0);
        Eigen::Vector3f t = transMtx_now.block<3,1>(0,3);
        Eigen::Vector3f relative_pos = -R.transpose()*t;

        /*
        // Print each matrix for debugging
        std::cout << "T_now: \n" << transMtx_now << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "t: \n" << t << std::endl;
        std::cout << "-R^T * t: \n" << relative_pos << std::endl;
        */

        pos[0] = relative_pos(0);
        pos[1] = relative_pos(1);
        pos[2] = 180/M_PI*atan2(transMtx_now(0,1), transMtx_now(0,0));


        transMtx_prev = transMtx_now; // Save current transformation matrix in transMtx_prev

        // TO DO END

        /*
        // Plot tf for debugging

        static tf::TransformBroadcaster br;
        tf::Transform transform;

        Eigen::Matrix4f& Tm = transMtx_now;

        tf::Vector3 origin;
        origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

        //cout << origin << endl;
        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
              static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
              static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        transform.setOrigin(origin);
        transform.setRotation(tfqt);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser_frame", "world_frame"));
        */

    }
    /// 5. Save new_cloud in prev_cloud

    prev_cloud = new_cloud;

}


/// The node's main function
int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_odometry_node");           // Initializes Node Name as "lidar_odometry_node"
    ros::NodeHandle nh;                                     // Node handle declaration for communication with ROS system

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);     // Declare a subscriber 'sub_lidar' which subscribes a topic named "/scan" and pass it into the callback function named 'lidar_cb'. The size of the message queue is 1.
    ros::Rate loop_rate(5);                                 // Set the loop period. '5' refers to 5 Hz and the main loop repeats at 0.2 second intervals.

    prev_cloud->width = 0;                                  // initialize prev_cloud


    // TO DO START
    string filePath = "/home/jlee/hw4_result.txt";          // Specify file where the status (x,y,theta) will be written.
    // TO DO END

    ofstream txtFile(filePath);                             // Define a stream class object named 'txtFile' on 'filePath' directory to write on files, and open it.

    while(ros::ok()){                                       // ros::ok() will return false if: a SIGINT is received (Ctrl-C), ros::shutdown() has been called, all ros::NodeHandles have been destroyed and so on.
        ros::spinOnce();                                    // ros::spinOnce() will call all the callbacks waiting to be called at that point in time.

        ROS_INFO("pos : x = %f | y = %f | theta = %f", pos[0], pos[1], pos[2]);             // Print information about the robot's position on a terminal.
        txtFile << "position: " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;      // Write text message on the 'filePath' through a stream called 'txtFile'.

        loop_rate.sleep();                                  // Pause this loop. Consequently, the loop works at an interval of 5Hz.
    }

    txtFile.close();                                        // Close a stream 'txtFile'.
    return 0;
}
