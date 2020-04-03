#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <fstream>
#include <iostream>

#include <math.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

// Init variables
float y_pos, x_pos, x_vel, x_vel_time, frequency, delay, yaw_offset, magnetic_declination_radians , imu_yaw ,x_utm, y_utm;
bool zero_altitude, broadcast_utm_transform, publish_filtered_gps, use_odometry_yaw, wait_for_datum;


void getParams()
{
    ros::param::get("/x_vel", x_vel);
    ros::param::get("/x_vel_time", x_vel_time);
    ros::param::get("/navsat_transform/frequency", frequency);
    ros::param::get("/navsat_transform/delay", delay);
    ros::param::get("/vectornav/navsat_transform/magnetic_declination_radians", magnetic_declination_radians);
    ros::param::get("/navsat_transform/yaw_offset", yaw_offset);
    ros::param::get("/navsat_transform/zero_altitude", zero_altitude);
    ros::param::get("/navsat_transform/broadcast_utm_transform", broadcast_utm_transform);
    ros::param::get("/navsat_transform/publish_filtered_gps", publish_filtered_gps);
    ros::param::get("/navsat_transform/use_odometry_yaw", use_odometry_yaw);
    ros::param::get("/navsat_transform/wait_for_datum", wait_for_datum);
}

void writeParams(std::string path_to_param_file, float heading_err)
{
    // Open file
    std::ofstream paramsFile (path_to_param_file.c_str());
    
    // Write to file
        paramsFile << "navsat_transform:" << std::endl;
        paramsFile << std::fixed << std::setprecision(0) << "  frequency: " << frequency << std::endl;
        paramsFile << std::fixed << std::setprecision(1) << "  delay: " << delay << std::endl;
    
        // Adding heading error to magnetic declination parameter to correct initial poor estimate
        paramsFile << std::fixed << std::setprecision(5) << "  magnetic_declination_radians: " << (magnetic_declination_radians + heading_err)<< std::endl;
        paramsFile << std::fixed << std::setprecision(5) << "  yaw_offset: " << yaw_offset << std::endl;
        paramsFile << "  zero_altitude: " << std::boolalpha << zero_altitude << std::endl;
        paramsFile << "  broadcast_utm_transform: " << std::boolalpha << broadcast_utm_transform << std::endl;
        paramsFile << "  publish_filtered_gps: " << std::boolalpha << publish_filtered_gps << std::endl;
        paramsFile << "  use_odometry_yaw: " << std::boolalpha << use_odometry_yaw << std::endl;
        paramsFile << "  wait_for_datum: " << std::boolalpha << wait_for_datum << std::endl;

    // Close file
    paramsFile.close();
}





//SUB to get Data
    void filtered_odom_CB(const geometry_msgs::PoseWithCovarianceStamped odom_msgs)
    {
            y_pos = odom_msgs.pose.pose.position.y;
            x_pos = odom_msgs.pose.pose.position.x;
    }

    void imu_data(const sensor_msgs::Imu imu_msgs)
    {
        //imu_yaw = imu_msgs.orientation.w;
            
        
        imu_yaw = (tf::getYaw(imu_msgs.orientation)) *180/M_PI;
       // ROS_INFO("The IMU yaw orientation is %lf",imu_yaw);
           
    }





int main(int argc, char **argv)
{
    x_vel = .1;
    x_vel_time = 10; 
	
    //Initiate node and set hangle
    ros::init(argc, argv, "calibrate_heading");
    ros::NodeHandle n;
    ROS_INFO("Initiated calibration node");

    // Initialise publishers and subscribers
    //ros::Subscriber sub_odom = n.subscribe("/robot_due/utm", 1, filtered_odom_CB);
    ros::Subscriber sub_imu = n.subscribe("/imu/data", 1, imu_data);
    ros::Subscriber sub_fix = n.subscribe("/hedge_pose", 1, filtered_odom_CB);


    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel",20);
    ros::Publisher pubCalibrationNodeEnded = n.advertise<std_msgs::Bool>("/calibrate_status",20);

    // Get parameters from parameer server
    getParams();
    ROS_WARN("PLEASE ENSURE YOU HAVE MIN. %.1f m OF CLEAR SPACE IN FRONT OF YOUR ROBOT FOR CALIBRATION",(x_vel*x_vel_time));    

    // set publish rate and calculate no. of messages to count
    int pubRate = 20;
    int numVelMsgs = x_vel_time * pubRate;
    ros::Rate rate(pubRate);
    
    
   // ros::spinOnce();

    for(int i=0; i< 5; i++)
    {
        
        rate.sleep();
    }
    ros::Duration(2).sleep(); // Pause for 2 seconds to prevent quick forwards and backwards movement
    
    
        


    ros::spinOnce();


   
    //ROS_INFO("Starting point is x: %f, y: %f and yaw : %f", x_utm, y_utm,imu_yaw);


   

    float x0 = x_pos;
    float y0 = y_pos;
    float yaw = imu_yaw;




    // Create forward velocity commmands and publish
    geometry_msgs::Twist velmsg;
    velmsg.linear.x= x_vel;
    velmsg.angular.z=0;
    for(int i=0; i< numVelMsgs; i++)
    {
        pubVel.publish(velmsg);
	ros::spinOnce();
        rate.sleep();
    }
    ros::Duration(2).sleep(); // Pause for 2 seconds to prevent quick forwards and backwards movement
    
    // Read y value in filtered odometry and determine correction to heading
    ros::spinOnce();

    ROS_INFO("End point is x: %f, y: %f ", x_pos, y_pos);
    float angle_utm = -1 * (atan2(x_pos-x0,y_pos-y0) - M_PI/2);
    ROS_INFO("ATAN2 %f ",atan2(x_pos-x0,y_pos-y0));

    if (angle_utm > M_PI)
    {
        angle_utm = angle_utm - M_PI*2;
    }

    float IMUangle = imu_yaw;

    float yaw_correction = (angle_utm * (180/M_PI)) - IMUangle;

    
    ROS_INFO("The IMU yaw orientation is %lf",imu_yaw);
    ROS_INFO("Delta is del_x : %f and del_y:  %f ", x_pos-x0,y_pos-y0);

    ROS_INFO("Angle between UTM points: %f and IMU angle: %f and yaw correction is %f", angle_utm * (180/M_PI), IMUangle, yaw_correction) ;


    ROS_INFO("Current Correction %f , with applied yaw_correction %f", magnetic_declination_radians , magnetic_declination_radians + yaw_correction);
    // write params file
    std::string path =  ros::package::getPath("mmp30_control") + "/config/navsat_params.yaml";
    ROS_INFO("Writing calibration results to file...");
    writeParams(path, yaw_correction);
    ROS_INFO("Wrote to param file: ");
    std::cout << path.c_str() << std::endl;       

    // Create backward commmands and publish
    ROS_INFO("Returning to start...");
    velmsg.linear.x= -1*x_vel;
    velmsg.angular.z=0;
    
    for(int i=0; i< numVelMsgs; i++)
    {
        pubVel.publish(velmsg);
	ros::spinOnce();
        rate.sleep();
    }
    velmsg.linear.x= 0;
    pubVel.publish(velmsg);
	ros::spinOnce();
    rate.sleep();

    
    // ROS_INFO("Heading Calibration Complete");

    // Notify joy_launch_control that collection is complete
        std_msgs::Bool node_ended;
        node_ended.data = true;
        pubCalibrationNodeEnded.publish(node_ended);
    
    ROS_INFO("Ending Node...");
    ROS_WARN("PLEASE RESTART YOUR EKF NODES TO APPLY NEW CALIBRATION PARAMETERS.");
	ros::shutdown();
	return 0;
}
