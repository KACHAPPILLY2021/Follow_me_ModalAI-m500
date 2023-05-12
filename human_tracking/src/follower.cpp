#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <human_tracking/AiDetection.h>
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"



// sensor message
sensor_msgs::Range distance_msg;

human_tracking::AiDetection ai_msg;
sensor_msgs::Range lidar_distace;

std_msgs::Float32 x_mean_msg;
std_msgs::Float32 y_mean_msg;
std_msgs::String reqd_class;

std_msgs::Float32 read_error;
std_msgs::Float32 d;


//call back function for lidar sensor
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  distance_msg = *msg;
}

//call back function for yolo output
void ai_sub_cb (const human_tracking::AiDetection::ConstPtr& msg){
    ai_msg = *msg;
}

// Finding correspondance between pixel location and yaw 
float pixel_to_yaw(float pix)
{
    float pix_1 = 5, pix_2 = 635, yaw_1 = 0.61 , yaw_2 = -0.61;
    float yaw_conv = ((yaw_2 - yaw_1)/(pix_2 - pix_1))*(pix - pix_1) + yaw_1;

    return yaw_conv;
}

// Calculating Distance required for forward and backward motion 
std::pair<float, float> distance_calculation(sensor_msgs::Range distance_msg , float Kp , float Kd , float prev_error , float prev_distance , float reqd)
{

    // get the current distance from the current message distance_msgs
    float current_dist = distance_msg.range;
    // Error = current - required
    float error = current_dist - reqd;

    //calculate pd control error
    float pd_error =  Kp*error + Kd*(error-prev_error) ;

    float distance_cmd;

    // add the error to the current distance to calucalte the command distance
    if (pd_error<0.3)
    {
        distance_cmd = prev_distance + pd_error;
    }
    else
    {
        distance_cmd = prev_distance + 0;
        error = 0;
    }
    

    // return curretn error and current command
    return std::make_pair(error, distance_cmd);
}


// Calculating yaw angle required for heading control
std::pair<float, float> track_human(human_tracking::AiDetection ai_msg , float Kp , float Kd , float prev_error , float prev_yaw , std_msgs::String class_name)
{
    // Center of bounding box
    x_mean_msg.data = (ai_msg.x_min + ai_msg.x_max)/2;

    float req_yaw = pixel_to_yaw(x_mean_msg.data);

    // Error = current - required
    float error = req_yaw - 0;
    float speed =  Kp*error + Kd*(error-prev_error) ;


    if(ai_msg.class_name==class_name.data)
    {

        ROS_INFO("HUUMMAN");
        ROS_INFO("SPEED:  [%f] ", speed);
        prev_yaw = prev_yaw + speed;
    }
    else
    {

        prev_yaw = prev_yaw + 0;
        error = 0;
        ROS_INFO("NNOOOOO");
        ROS_INFO("SPEED:  [%f] ", speed);
    }

    return std::make_pair(error, prev_yaw);
}



int main(int argv, char **argc){


    int count = 0;

    // Gains for yaw control
    float Kp = 0.15;
    float Kd = 0.1;

    float prev_error = 0.0;

    float yaw_value;
    float yaw_to_send = 0;

    // Gains for forward and backward motion
    float kp_lidar = 0.0005;
    float kd_lidar = 0.04;
    float reqd_depth = 2.5; // in metres
    float prev_error_lidar = 0.0;
    float distance_to_send = 0.0;

    std_msgs::String reqd_class;
    reqd_class.data = "person";

    float x_send = 0;
    float y_send = 0; 


    ros::init(argv, argc, "follower");
    ros::NodeHandle nh;


    ros::Subscriber ai_sub = nh.subscribe <human_tracking::AiDetection>("/tflite_data", 5, ai_sub_cb);
    ros::Publisher target_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    ros::Publisher jeff_pub = nh.advertise<std_msgs::Float32>("/jeffin", 5);
    ros::Publisher error_pub = nh.advertise<std_msgs::Float32>("/error_yaw", 5);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/lidar_pub", 5, rangeCallback);

    ros::Rate rate(10.0);

    
    // Log an informational message
    ROS_INFO("My node has started.");

    mavros_msgs::PositionTarget target_pose;

    // mavros_msgs::PositionTarget pose_vel;

    // Note that this type_mask is assuming you are affecting VELOCITY control over the vehicle.  See the original offboard example for the position type mask
    // pose_vel.coordinate_frame = pose_vel.FRAME_BODY_NED;//pose_vel.FRAME_LOCAL_NED;
    // pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW | pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;

    target_pose.type_mask = target_pose.IGNORE_VX | target_pose.IGNORE_VY | target_pose.IGNORE_VZ | target_pose.IGNORE_AFZ | target_pose.IGNORE_AFY | target_pose.IGNORE_AFX;
    // target_pose.coordinate_frame = target_pose.FRAME_LOCAL_NED;
    target_pose.coordinate_frame = target_pose.FRAME_BODY_NED; // Check whether FLU # Z +ve UP, X +ve forward, Yaw +ve CC

    while(ros::ok){


        ROS_INFO("COUNT :  [%i]  ", count);

        // Delaying up start of control so error does not overshoot
        if (count <500){
            target_pose.yaw =  0;
            target_pose.position.z = 1.0;
            target_pose_pub.publish(target_pose);
            count++;

        } 
        else
        {
        // Yaw control
        std::pair<float, float> result = track_human(ai_msg , Kp , Kd , prev_error , yaw_to_send , reqd_class);
        prev_error = result.first;
        yaw_to_send = result.second;

        read_error.data = prev_error;
        target_pose.yaw =  yaw_to_send;
        target_pose.position.z = 1.0;

        // Forward and backward control
        if (abs(prev_error) < 0.017 && (ai_msg.class_name==reqd_class.data)){

        std::pair<float, float> result_lidar = distance_calculation( distance_msg , kp_lidar , kd_lidar , prev_error_lidar , distance_to_send , reqd_depth);
        prev_error_lidar  = result_lidar.first;
        distance_to_send = result_lidar.second;

        d.data = distance_to_send;

        x_send = x_send + distance_to_send*cos(yaw_to_send);
        y_send = y_send + distance_to_send*sin(yaw_to_send);

        target_pose.position.x = x_send;
        target_pose.position.y = y_send;

 
        }

        target_pose_pub.publish(target_pose);
        error_pub.publish(read_error);
        jeff_pub.publish(d);
        }

        ros::spinOnce();
        rate.sleep();

    }

}
