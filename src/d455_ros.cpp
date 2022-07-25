#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <math.h>
#include "../function/locate_elp.hpp"
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
const double depth_scale = 0.001;
const double cx = 320.207;
const double cy = 236.529;
const double fx = 388.677;
const double fy = 388.677;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 10,
                                   &ImageConverter::imageCb, this);
        depth_sub_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 10,
                                   &ImageConverter::depthCb, this);
        cv::namedWindow("Image_window");
    }

    ~ImageConverter()
    {
        cv::destroyWindow("Image_window");
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;
        cv::rectangle(img, cv::Point(344, 540), cv::Point(360, 556), (0, 255, 0), 1);
        cv::line(img, cv::Point(344, 540), cv::Point(360, 556), (0, 255, 0), 1);
        cv::imshow("Image_window", img);
        // std::cout<<"x="<<img.rows<<"\t"<<"y="<<img.cols<<std::endl;
    }
    void depthCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_img = cv_ptr->image;
        locate::axis B = locate::Cal_axis(depth_img, cv::Point(320, 240));
        std::cout << "\treal_x=" << B.real_x;
        std::cout << "\treal_y=" << B.real_y;
        std::cout << "real_z=" << B.real_z << std::endl;
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        if (cv::waitKey(1) == 27)
        {
            break;
        }
        ros::spinOnce();
        // rate.sleep();
    }
    return 0;
}