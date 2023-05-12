#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <human_tracking/AiDetection.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/Range.h>

static const std::string OPENCV_WINDOW = "Image window";





class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber ai_sub;
    ros::Subscriber dist_sub;
    ros::Subscriber error_sub;
    image_transport::Publisher image_pub_;

    std_msgs::Int16 x_mean_msg_;
    std_msgs::Int16 y_mean_msg_;
    std_msgs::String reqd_class_;
    human_tracking::AiDetection ai_msg;
    std_msgs::Float32 error;
    sensor_msgs::Range distance_msg;
    


    std::string text_r = "TURN RIGHT !";
    std::string text_l = "TURN LEFT !";
    std::string text_f = "MOVE FORWARD !";
    std::string text_b = "MOVE BACKWARD !";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    int baseline = 0;
    float reqd_depth = 2.5;

    public:
    ImageConverter()
    : it_(nh_)
    {
    // Subscribe to input video feed and publish output video feed

    // NOT it_ 
    ai_sub = nh_.subscribe("/tflite_data", 5, &ImageConverter::ai_sub_cb, this);
    dist_sub = nh_.subscribe("/mavros/distance_sensor/lidar_pub", 5, &ImageConverter::dist_sub_cb, this);
    error_sub = nh_.subscribe("/error_yaw", 5, &ImageConverter::error_sub_cb, this);


    image_sub_ = it_.subscribe("/hires", 1,
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/human_bb_view", 1);


    }

    //call back function for yolo output
    void ai_sub_cb(const human_tracking::AiDetection::ConstPtr& msg)
    {
        ai_msg = *msg;
        ROS_INFO("callback");
    }

    //call back function for 1D LiDAR
    void dist_sub_cb(const sensor_msgs::Range::ConstPtr& msg)
    {
        distance_msg = *msg;

    }  

    //call back function for yaw error
    void error_sub_cb(const std_msgs::Float32::ConstPtr& msg)
    {
        error = *msg;

    }       

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Inside IMAGE CALLBACK");
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

        reqd_class_.data = "person";
        x_mean_msg_.data = int( (ai_msg.x_min + ai_msg.x_max)/2 );
        y_mean_msg_.data = int( (ai_msg.y_min + ai_msg.y_max)/2 );

        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        {
            // Center of camera feed
            cv::circle(cv_ptr->image, cv::Point(320, 240), 5, CV_RGB(255,0,0) , -1); 

            // If detected object is person
            if(ai_msg.class_name==reqd_class_.data)
            {
                ROS_INFO("Detected human");
                // Center of human bounding box
                cv::circle(cv_ptr->image, cv::Point(x_mean_msg_.data, y_mean_msg_.data), 3, CV_RGB(0,255,0),-1 );
                // Human bounding box
                cv::rectangle(cv_ptr->image, cv::Point(ai_msg.x_min, ai_msg.y_min), cv::Point(ai_msg.x_max, ai_msg.y_max), CV_RGB(0,255,0), 2);

                // Conditions for displaying text on window
                if(abs(error.data)>0.017)
                {
                if (x_mean_msg_.data < 320)
                {
                    cv::Size textSize = cv::getTextSize(text_l, fontFace, fontScale, thickness, &baseline);
                    cv::Point textOrg(10, textSize.height + 10);
                    cv::putText(cv_ptr->image, text_l, textOrg, fontFace, fontScale, (255, 255, 255), thickness);               
                }
                else if(x_mean_msg_.data > 320)
                {
                    cv::Size textSize = cv::getTextSize(text_r, fontFace, fontScale, thickness, &baseline);
                    cv::Point textOrg(10, textSize.height + 10);
                    cv::putText(cv_ptr->image, text_r, textOrg, fontFace, fontScale, (255, 255, 255), thickness);   
                }
                }
                else
                {
                if(distance_msg.range > reqd_depth)
                {
                    cv::Size textSize = cv::getTextSize(text_f, fontFace, fontScale, thickness, &baseline);
                    cv::Point textOrg(10, textSize.height + 10);
                    cv::putText(cv_ptr->image, text_r, textOrg, fontFace, fontScale, (255, 255, 255), thickness);   
                }
                else if(distance_msg.range < reqd_depth)
                {
                    cv::Size textSize = cv::getTextSize(text_b, fontFace, fontScale, thickness, &baseline);
                    cv::Point textOrg(10, textSize.height + 10);
                    cv::putText(cv_ptr->image, text_r, textOrg, fontFace, fontScale, (255, 255, 255), thickness);   
                } 
                }


                               

            }
        }

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    ROS_INFO("VIEW node has started");

    ImageConverter ic;
    ros::spin();
    return 0;
}


