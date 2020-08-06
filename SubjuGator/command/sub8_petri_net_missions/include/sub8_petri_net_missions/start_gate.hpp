#pragma once
#include <ros/ros.h>
#include <boost/tuple/tuple.hpp>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <signal.h>

#include <mil_poi/POI.h>
#include <mil_poi/POIArray.h>


typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::Point POI;
typedef ros::Duration Duration;

struct PoseDuration
{
  Pose pose;
  Duration duration;
};

struct PoiDuration
{
  POI poi;
  Duration duration;
};

struct POI_2
{
  POI pois[2];
};

struct PoseConf
{
  Pose pose;
  double conf;
};

struct PoseSuccess
{
  Pose pose;
  bool success;
};
struct PoseConf_2
{
  PoseConf pose_confs[2];
};

PoseConf find_post(PoiDuration _poi_timeout);
