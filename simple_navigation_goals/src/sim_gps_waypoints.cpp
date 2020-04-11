
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/PoseArray.h>

#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <string>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>


// initialize variables
    typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction>
    MoveBaseClient; //create a type definition for a client called MoveBaseClient

    std::vector <std::pair<double, double> > waypointVect;
    std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
    geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;


    geometry_msgs::PoseArray poseArray;  


    int count = 0, waypointCount = 0, wait_count = 0;
    double numWaypoints = 0;
    double latiGoal, longiGoal, latiNext, longiNext;
    std::string utm_zone;
    std::string path_local, path_abs, coordinates_files;

    std::ofstream waypointsCSV;
    std::string CSVpath = ros::package::getPath("simple_navigation_goals") + "/nav_master_waypoints/waypoints.csv";

    waypointsCSV.open(CSVpath, std::ios_base::app);
    waypointsCSV <<"X" << ","
                << "Y" << ","
                << "Z"<< ","
                << "THETAX"<< ","
                << "THETAY"  << ","
                << "THETAZ" << ","
                << "QUATW"  << std::endl;
                
    waypointsCSV.close();



//


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("simple_navigation_goals") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}


std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
  double lati = 0, longi = 0;

  // Creates the pathway by finding the simple_navigation_goals package and then
  // adding on the path_local path to find the coordinates text file
  path_abs = ros::package::getPath("simple_navigation_goals") + path_local;
  
  // Reading from txt file that converts the file into a string
  std::ifstream fileRead(path_abs.c_str());

  //Iterates through the coordinates file based on number of waypoints
  for(int i = 0; i <numWaypoints; i ++)
  {
    // Each Point read given to lati , longi
    fileRead >> lati;
    fileRead >> longi;

    // Append pair of Lati,Longi into the waypointVect
    waypointVect.push_back(std::make_pair(lati, longi));
    
  }

  // Close the fileRead file after done iterating through
  fileRead.close();

  // Outputting Vector to terminal
  ROS_INFO("The following GPS Waypoints have been set:");
  for(std::vector < std::pair < double,double >> ::iterator iterDisp = waypointVect.begin();
      iterDisp != waypointVect.end(); iterDisp++)
      {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
      }
      return waypointVect;

}


geometry_msgs::PointStamped latlongtoUTM(double lati_input, double longi_input)
{
  //Initialize utm_x utm_y , UTM_point_output
  double utm_x = 0, utm_y = 0;
  geometry_msgs::PointStamped UTM_point_output;


  //convert lat/long to UTM (OLD METHOD
       //RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y , utm_x, 
     // utm_zone);

  //GPS common LATLONG to UTM conversions (NEW METHOD)
  gps_common::UTM(lati_input, longi_input , &utm_x, &utm_y);

  //Construct UTM_point and map_point geometry messages
  UTM_point_output.header.frame_id = "utm";
  UTM_point_output.header.stamp = ros::Time(0);
  UTM_point_output.point.x = utm_x;
  UTM_point_output.point.y = utm_y;
  UTM_point_output.point.z = 0;

  return UTM_point_output;
}


geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("map", "utm", time_now, ros::Duration(3.0));
            tf::StampedTransform transform_map;
            listener.lookupTransform("map", "utm", ros::Time(0), transform_map);
            map_point_output.header.frame_id="map";
            map_point_output.header.stamp=UTM_input.header.stamp;
            double origin_x=transform_map.getOrigin().x();
            double origin_y=transform_map.getOrigin().y();
            
            listener.transformPoint("map", UTM_input, map_point_output);
            // map_point_output.point.x=UTM_input.point.x;
            // map_point_output.point.y=UTM_input.point.y;
            ROS_INFO("UTM MAP POINT : %f, %f",map_point_output.point.x,map_point_output.point.y);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}


move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "utm_relative";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    return goal;
}


geometry_msgs::PoseArray pubArray(move_base_msgs::MoveBaseGoal goal, ros::Publisher poseArrayPub)
{
    // This function will take in a MoveBase Goal and publish a Posearray type message
    // Rviz will then be able to read it and display it in RVIZ

    //ros::Publisher poseArrayPub;
    geometry_msgs::PoseStamped pose;

    //poseArray.poses.clear();
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "utm_relative";

    pose.pose.position.x = goal.target_pose.pose.position.x;
    pose.pose.position.y = goal.target_pose.pose.position.y;    
    pose.pose.position.z = goal.target_pose.pose.position.z;

    pose.pose.orientation.x = goal.target_pose.pose.orientation.x;
    pose.pose.orientation.y = goal.target_pose.pose.orientation.y;
    pose.pose.orientation.z = goal.target_pose.pose.orientation.z;
    pose.pose.orientation.w = goal.target_pose.pose.orientation.w;

    poseArray.poses.push_back(pose.pose);


    poseArrayPub.publish(poseArray);

    ROS_INFO("Array Arrow Published! ");

    std::cout << CSVpath;
    waypointsCSV.open(CSVpath, std::ios_base::app);
    waypointsCSV << pose.pose.position.x << ","
                << pose.pose.position.y << ","
                << pose.pose.position.z << ","
                << pose.pose.orientation.x << ","
                << pose.pose.orientation.y << ","
                << pose.pose.orientation.z << ","
                << pose.pose.orientation.w << "," << std::endl;
                
    waypointsCSV.close();


    return poseArray;
    
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated gps_waypoint node");

    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()
    // ros::spin() lets all callbacks get called for subscriber, causes code to loop 
    // continously instead of ending at the end of main()
    MoveBaseClient ac("/move_base", true);

    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/simple_navigation_goals/waypoint_following_status", 100);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        wait_count++;
        if(wait_count > 3)
        {
            ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }
  
    ros::Publisher poseArrayPub = n.advertise<geometry_msgs::PoseArray>("array",1000);

    //Count how many waypoints are in txt file
    std::string path_local = "/coordinates_files.txt" ;
    numWaypoints = countWaypointsInFile(path_local);


    //Was not able get ros::param::get() to work in such that it outputs the file path to desired file.
    //ros::param::get("/coordinates_file",coordinates_files);
    //std::string path_abs =  ros::package::getPath("simple_navigation_goals") + path_local ;	
    //ROS_INFO("Saving coordinates to: %s", path_abs.c_str());
    //

    //Iterate through waypoints in text file and output to console
    waypointVect = getWaypoints(path_local);


    //Iterate through vector of waypoints to publish waypoint arrows for RVIZ visualization
  
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
        {
            //Setting Goals based on place in waypoint file
            latiGoal = iter->first;
            longiGoal = iter->second;

            // Final point Bool initialization 
            bool final_point = false;

            //Setting next goal point if not at last waypoint
            if(iter < (waypointVect.end() - 1))
            {
                //Temporarily sets iter forward to get next waypoint
                iter++;
                latiNext = iter->first;
                longiNext = iter->second;
                iter--;
            }
            else //set to current if it is the last point
            {
                latiNext = iter->first;
                longiNext = iter->second;
                final_point = true;
            }
            // ^ Waypoint Values Set

            //Declare/Output what values were received for Lat/Long
            //ROS_INFO("Received Latitude goal:%.8f", latiGoal);
            //ROS_INFO("Received Longitude goal:%.8f", longiGoal);

            //Convert lat/long into UTM
            UTM_point = latlongtoUTM(latiGoal, longiGoal);
            UTM_next = latlongtoUTM(latiNext, longiNext);
            // ROS_INFO("Current UTM POINT GOAL : %f , %f" , UTM_point.point.x,UTM_point.point.y );

            //Transform UTM to map point in odom frame
            map_point = UTMtoMapPoint(UTM_point);
            map_next = UTMtoMapPoint(UTM_next);

            ROS_INFO( " Latitude/Longitude : %f , %f \n  UTM: %f,%f  \n Odom: %f, %f", latiGoal, longiGoal ,UTM_point.point.x,UTM_point.point.y, map_point.point.x,map_point.point.y );

            //Build goal to send to move_base //initiate a move_base_msg called goal
            move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); 

            //Publish Array of goals
            poseArray = pubArray(goal,poseArrayPub);

        }


    ROS_INFO("All Array Arrows Have Published! Will now begin autonomous navigation");

    //------------------------------------------------------------------------------


    /*
    //Iterate through vector of waypoints for setting goals
    
        for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
            {
                //Setting Goals based on place in waypoint file
                latiGoal = iter->first;
                longiGoal = iter->second;

                // Final point Bool initialization 
                bool final_point = false;

                //Setting next goal point if not at last waypoint
                if(iter < (waypointVect.end() - 1))
                {
                    //Temporarily sets iter forward to get next waypoint
                    iter++;
                    latiNext = iter->first;
                    longiNext = iter->second;
                    iter--;
                }
                else //set to current if it is the last point
                {
                    latiNext = iter->first;
                    longiNext = iter->second;
                    final_point = true;
                }
                // ^ Waypoint Values Set


                //Declare/Output what values were received for Lat/Long
                //ROS_INFO("Received Latitude goal:%.8f", latiGoal);
                //ROS_INFO("Received Longitude goal:%.8f", longiGoal);

                //Convert lat/long into UTM
                UTM_point = latlongtoUTM(latiGoal, longiGoal);
                UTM_next = latlongtoUTM(latiNext, longiNext);
                // ROS_INFO("Current UTM POINT GOAL : %f , %f" , UTM_point.point.x,UTM_point.point.y );


                //Transform UTM to map point in odom frame
                map_point = UTMtoMapPoint(UTM_point);
                map_next = UTMtoMapPoint(UTM_next);

                //ROS_INFO("TEST TEST TEST");

                ROS_INFO( " Latitude/Longitude : %f , %f \n  UTM: %f,%f  \n Odom: %f, %f", latiGoal, longiGoal ,UTM_point.point.x,UTM_point.point.y, map_point.point.x,map_point.point.y );


                //Build goal to send to move_base //initiate a move_base_msg called goal
                move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); 
            

                // Send Goal
                ROS_INFO("Sending goal");
                ac.sendGoal(goal); //push goal to move_base node

                //Wait for result
                ac.waitForResult(); //waiting to see if move_base was able to reach goal

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("mmp_30 has reached its goal!");
                    //switch to next waypoint and repeat
                }
                else
                {
                    ROS_ERROR("mmp_30 was unable to reach its goal. GPS Waypoint unreachable.");
                    ROS_INFO("Exiting node...");
                    // Notify joy_launch_control that waypoint following is complete
                    std_msgs::Bool node_ended;
                    node_ended.data = true;
                    pubWaypointNodeEnded.publish(node_ended);
                    ros::shutdown();
                }
            } // End for loop iterating through waypoint vector

    
    
        ROS_INFO("mmp30 has reached all of its goals!!!\n");
        ROS_INFO("Ending node...");

        // Notify joy_launch_control that waypoint following is complete
        std_msgs::Bool node_ended;
        node_ended.data = true;
        pubWaypointNodeEnded.publish(node_ended);
    */

    ROS_INFO("End of script");


    ros::shutdown();
    ros::spin();
    return 0;



}








