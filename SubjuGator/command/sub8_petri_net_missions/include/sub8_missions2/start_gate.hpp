#pragma once
#include <ros/ros.h>
#include <boost/tuple/tuple.hpp>
#include <geometry_msgs/Pose.h>
#include <vector>


#include <mil_poi/POI.h>
#include <mil_poi/POIArray.h>


typedef mil_poi::POI POI;
typedef mil_poi::POIArray POIArray;
typedef geometry_msgs::Pose Pose;
typedef ros::Duration Duration;

typedef boost::tuple<Pose, Duration> PoseDuration;
typedef boost::tuple<POI, Duration> POIDuration;
typedef boost::tuple<Pose, double> PoseConf;
typedef boost::tuple<Pose, bool> PoseSuccess;
typedef boost::tuple<PoseConf, PoseConf> PoseConfTuple;



