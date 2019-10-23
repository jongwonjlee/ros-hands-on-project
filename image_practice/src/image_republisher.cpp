#include <ros/ros.h>                                // ROS Default Header File
#include <image_transport/image_transport.h>        // header which includes everything we need to publish and subscribe to images.
#include <opencv2/highgui/highgui.hpp>              //  These headers will allow us to display images using OpenCV's simple GUI capabilities.
#include <cv_bridge/cv_bridge.h>                    //  These headers will allow us to display images using OpenCV's simple GUI capabilities.
#include <boost/bind.hpp>                           // Include the header file inside the boost c++ library to use the function 'bind'. The function 'bind' is able to bind any argument to a specific value or route input arguments into arbitrary positions.


// Message callback function. This function is called when a message named "number" is received, as descibed in main function.
// As an input variable, the pointer of 'std_msgs::Int16' type message is received. The pointer of the ros publisher also comes in so that we can declare the message "number_oddeven" by using this.
void imageCallback(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher* publisher_flipped, const image_transport::Publisher* publisher_grayscale){

    // Declare two sensor_msgs::ImagePtr type message files. These are going to be published by two different image_transport::Publishers.
    sensor_msgs::ImagePtr msg_flipped;
    sensor_msgs::ImagePtr msg_grayscale;

    // If cv_bridge causes any exceptional error, do it.
    try {
        // Declare three different cv::Mat type variables. (i.e. Each one ontains an image.)
        cv::Mat buffer;
        cv::Mat frame_flipped;
        cv::Mat frame_grayscale;

        // Convert the ROS image message (sensor_msgs::ImageConstPtr& msg) to an OpenCV image (cv::Mat buffer) with BGR color space, 8bit pixel encoding
        buffer =cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::flip(buffer, frame_flipped, 1);                     // Flip the input image (cv::Mat buffer) and save it into a variable cv::Mat frame_flipped.
        cv::cvtColor( buffer, frame_grayscale, CV_BGR2GRAY );   // Convert color space of the input image (cv::Mat buffer) from BGR to grayscale and save it into a variable cv::Mat frame_grayscale.

        // cv_bridge::CvImage is used to convert OpenCV images to ROS images.
        // In cv_bridge::CvImage(const std_msgs::Header &header, const std::string &encoding, const cv::Mat &img), header means header message, encoding means encoding type of an image, img is an image variable to be converted into ROS message.
        // A method toImageMsg() converts sensor_msgs::Image into sensor_msgs::ImagePtr.
        // std_msgs::Header() denotes a header with default (zero) value.
        msg_flipped = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_flipped).toImageMsg();
        msg_grayscale = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_grayscale).toImageMsg();

        // Each image_transport::Publisher* type publisher's pointer publishes a message.
        publisher_flipped->publish(msg_flipped);
        publisher_grayscale->publish(msg_grayscale);

    }
    // Otherwise, return an error message.
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what()); // This error infers that codes above were not excecuted normally.
    }

}


int main(int argc, char** argv){

    ros::init(argc, argv, "image_republisher");                                    // Initializes Node Name as "image_republisher"
    ros::NodeHandle nh;                                                            // Node handle declaration for communication with ROS system
    image_transport::ImageTransport it(nh);                                        // We create an ImageTransport instance, initializing it with our NodeHandle. ImageTransport is used to create image publishers and subscribers, as much as we use methods of NodeHandle to create generic ROS publishers and subscribers.

    // Declare two different image_transport::Publisher. They publishes sensor_msgs::Image type message in ROS.
    image_transport::Publisher pub_flip = it.advertise("flipped/image", 1);
    image_transport::Publisher pub_gray = it.advertise("grayscale/image", 1);

    // Declare and create subscriber 'sub' which subscribes message type of 'sensor_msgs::Image'.
    // The subsciber hears the topic named "camera/image", and the size of the message queue as 1.
    // Once 'sub' recieves a message named "camera/image", it passes the message's pointer in boost::bind(imageCallback, _1, &pub_flip, &pub_gray).
    // Consequently, boost::bind function binds them as imageCallback(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher* publisher_flipped, const image_transport::Publisher* publisher_grayscale), and the callback function runs.

    // it.subscribe() returns a image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed,
    // it will automatically unsubscribe from the topic called "camera/image".
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, boost::bind(imageCallback, _1, &pub_flip, &pub_gray));

    // ros::spin() will enter a loop, pumping callbacks. It will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    ros::spin();

}
