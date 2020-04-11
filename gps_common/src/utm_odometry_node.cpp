/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool relative_utm = false;
bool first_utm = true;
double northing_origin, easting_origin;
std::shared_ptr<tf::TransformBroadcaster> odom_broadcaster_ptr;

void callback(const sensor_msgs::NavSatFixConstPtr &fix)
{
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
  {
    ROS_DEBUG_THROTTLE(60, "No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0))
  {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (relative_utm && first_utm)
  {
    northing_origin = northing;
    easting_origin = easting;
    first_utm = false;
  }
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  if (frame_id.empty())
    odom_trans.header.frame_id = fix->header.frame_id;
  else
    odom_trans.header.frame_id = frame_id;

  odom_trans.child_frame_id = odom_trans.header.frame_id + "_relative";

  odom_trans.transform.translation.x = easting_origin;
  odom_trans.transform.translation.y = northing_origin;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation.x = 0;
  odom_trans.transform.rotation.y = 0;
  odom_trans.transform.rotation.z = 0;
  odom_trans.transform.rotation.w = 1;

  //Send the transform
  odom_broadcaster_ptr->sendTransform(odom_trans);

  if (odom_pub)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;
    if (relative_utm)
    {
      odom.header.frame_id = odom.header.frame_id + "_relative";
    }

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting - easting_origin;
    odom.pose.pose.position.y = northing - northing_origin;
    odom.pose.pose.position.z = fix->altitude;

    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{fix->position_covariance[0],
                                            fix->position_covariance[1],
                                            fix->position_covariance[2],
                                            0, 0, 0,
                                            fix->position_covariance[3],
                                            fix->position_covariance[4],
                                            fix->position_covariance[5],
                                            0, 0, 0,
                                            fix->position_covariance[6],
                                            fix->position_covariance[7],
                                            fix->position_covariance[8],
                                            0, 0, 0,
                                            0, 0, 0, rot_cov, 0, 0,
                                            0, 0, 0, 0, rot_cov, 0,
                                            0, 0, 0, 0, 0, rot_cov}};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  odom_broadcaster_ptr.reset(new tf::TransformBroadcaster());
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("relative_utm", relative_utm, false);

  odom_pub = priv_node.advertise<nav_msgs::Odometry>("odom", 10);
  

  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);

  ros::spin();
}
