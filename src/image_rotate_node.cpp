#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <stdio.h>

using namespace cv;

class ImageRotate
{

    ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

    public:
    ImageRotate()
        : it_(nh_)
    {
        ROS_INFO("Allo");
      image_pub_ = it_.advertise("image_rotated", 10);
      image_sub_ = it_.subscribe("/image_raw", 10, &ImageRotate::handleImage, this);


    }

    cv::Mat transform(cv::Mat image)
    {

        int rotationX_ = -28.6;
        int rotationY_ = 0;
        int zoom_ = 100;

        //Load parameters from file
        //std::ifstream infile("test.txt");
        //infile >> rotationX_ >> rotationY_ >> zoom_;

        const double w = image.cols;
        const double h = image.rows;

        // Projection 2D -> 3D matrix
        Mat A1 = (Mat_<double>(4,3) <<
            1, 0, -w/2,
            0, 1, -h/2,
            0, 0,    0,
            0, 0,    1);

        // Rotation matrices around the X axis
        const double alpha = rotationX_ / 180. * 3.1416 / w;
        Mat RX = (Mat_<double>(4, 4) <<
            1,          0,           0, 0,
            0, cos(alpha), -sin(alpha), 0,
            0, sin(alpha),  cos(alpha), 0,
            0,          0,           0, 1);

        // Rotation matrices around the X axis

        const double beta = rotationY_ / 180. * 3.1416 / h;
        Mat RY = (Mat_<double>(4, 4) <<
            cos(beta),  0,           sin(beta), 0,
            0,			1,			 0, 0,
            -sin(beta), 0,			 cos(beta), 0,
            0,          0,           0, 1);

        // Translation matrix on the Z axis
        const double dist = 0.5;//zoom_ / 100;
        Mat T = (Mat_<double>(4, 4) <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, dist,
            0, 0, 0, 1);

        const double f = 1;
        // Camera Intrisecs matrix 3D -> 2D
        Mat A2 = (Mat_<double>(3,4) <<
            f, 0, w/2, 0,
            0, f, h/2, 0,
            0, 0,   1, 0);

        Mat transfo = A2 * (T * (RX * RY * A1));

        Mat destination;

        warpPerspective(image, destination, transfo, image.size(), INTER_CUBIC | WARP_INVERSE_MAP);

        return destination;
    }


    void handleImage(const sensor_msgs::Image::ConstPtr& msg)
    {
        //ROS_INFO("got something!");

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        // Draw an example circle on the video stream
        //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        cv_ptr->image = transform(cv_ptr->image);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }


};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_rotate_node");
    ImageRotate rotate;

    ros::spin();

     return 0;
}