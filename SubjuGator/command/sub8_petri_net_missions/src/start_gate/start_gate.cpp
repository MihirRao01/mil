#include <ros/ros.h>
#include <boost/bind.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mil_msgs/MoveToAction.h>

#include <nav_msgs/Odometry.h>
#include <mil_msgs/MoveToActionGoal.h>


#include <start_gate/start_gate_net.hpp>
#include <sub8_petri_net_missions/start_gate.hpp>


Duration start_gate_net::get_timeout::Body()
{
  //std::cout << "get_timeout\n";
  double timeout;
  std::string pname = "timeout";
  if (!ros::param::get(pname, timeout))
  {
    ROS_ERROR("Getting timout Failed");
    return Duration();
  }
  return ros::Duration(timeout);
}

POI_2 start_gate_net::get_next_pois::Body()
{
  //std::cout << "get_next_pois\n";
  POI pois[2];
  std::string pname = "/poi_server/initial_pois/start_gate";
  for (int i = 1; i < 3; ++i)
  {
    std::vector<double> poi;
    if (!ros::param::get(pname + std::to_string(i), poi))
    {
      ROS_ERROR("could not get the poi");
      return {POI(), POI()};
    }
    pois[i-1].x = poi.at(0);
    pois[i-1].y = poi.at(1);
    pois[i-1].z = poi.at(2);
  }
  return {pois[0], pois[1]};
}

// TODO: move to lib
template<typename MSG>
MSG get_next_msg(ros::NodeHandle& nh, const std::string& topic_name)
{
  bool got_pose = false;
  MSG ret;
  boost::function<void (const typename MSG::ConstPtr&)> cb =
  [&ret, &got_pose] (const typename MSG::ConstPtr& msg)
  {
    ret = *msg;
    got_pose = true;
  };
  auto sub = nh.subscribe<MSG>(topic_name, 1, cb);
  do
  {
    ros::spinOnce();
  } while(!got_pose);
  sub.shutdown();
  return ret;
}

void move_to_strafe(ros::NodeHandle& nh, const double& x, const double& y)
{

  auto odom = get_next_msg<nav_msgs::Odometry>(nh, "/odom");
  mil_msgs::MoveToGoal new_goal;
  new_goal.posetwist.pose.orientation = odom.pose.pose.orientation; // dont change orientation
  new_goal.posetwist.pose.position.x = x;
  new_goal.posetwist.pose.position.y = y;
  new_goal.posetwist.pose.position.z = odom.pose.pose.position.z;

  actionlib::SimpleActionClient<mil_msgs::MoveToAction> ac("/moveto", true);
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server is up.");
  ac.sendGoal(new_goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(120));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  ros::spinOnce();
}


void start_gate_net::approach_pois::Body(POI_2 _pois)
{
  //std::cout << "approach_pois\n";
  // TODO, make a cpp version of pose editor
  // rosaction move to 1 meter short of the average of the pois
  int argc = 0;
  char** argv = nullptr;
  ros::init(argc, argv, "approach_pois", true);
  ros::NodeHandle nh;
  auto odom = get_next_msg<nav_msgs::Odometry>(nh, "/odom");
  std::vector<double> pois_avg = {(_pois.pois[0].x + _pois.pois[1].x)/2.0,
                                  (_pois.pois[0].y + _pois.pois[1].y)/2.0};
  std::vector<double> pois_to_odom = {odom.pose.pose.position.x - pois_avg.at(0),
                                      odom.pose.pose.position.y - pois_avg.at(1)};

  auto mag = sqrt(pow(pois_to_odom.at(0),2) + pow(pois_to_odom.at(1),2));
  pois_avg.at(0) += 2*pois_to_odom.at(0)/mag;
  pois_avg.at(1) += 2*pois_to_odom.at(1)/mag;

  move_to_strafe(nh, pois_avg.at(0), pois_avg.at(1));

  //TODO make into an action server lib
  return;
}

PoseConf find_post(PoiDuration _poi_timeout)
{
  //std::cout << "find_post\n";

  const auto& poi = _poi_timeout.poi;
  const auto& timeout = _poi_timeout.duration;
  auto pose = Pose();
    pose.position.x = poi.x;
    pose.position.y = poi.y;
    pose.position.z = poi.z;
  double conf = 0.0;

  int argc = 0;
  char** argv = nullptr;
  ros::init(argc, argv, "find_post", true);
  ros::NodeHandle nh;
  //get dist from POI
  double dist = -1;
  boost::function<void (const nav_msgs::Odometry::ConstPtr&)> odom_cb =
  [&dist, &poi] (const nav_msgs::Odometry::ConstPtr& msg)
  {
    dist = sqrt(pow(poi.x - msg->pose.pose.position.x,2) +
                pow(poi.y - msg->pose.pose.position.y,2) +
                pow(poi.z - msg->pose.pose.position.z,2));
  };

  auto sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odom_cb);
  while(ros::Time::now().sec == 0); // wait for rostime to report correctly
  auto stop_time = ros::Time::now() + timeout;
  // simulated behavior
  do
  {
    //std::cout << dist << std::endl;
    ros::spinOnce();
    // return the POI and rand% confidence
    if (dist < 5.0 && dist > 0)
    {
      conf = (double) std::rand() / RAND_MAX;
      //std::cout << conf << std::endl;
      //std::cout << "found the post\n";
      return {pose, conf};
    }
    //std::cout << std::to_string(ros::Time::now().sec) + " " + std::to_string(stop_time.sec) + "\n";
  } while(ros::Time::now().sec < stop_time.sec);
  if (dist == -1)
    ROS_ERROR("distance from POI was never successfully captured");
  // if more than 5 meters from the POI, report the POI with no confidence
  return {pose, conf};

  // TODO
  // try to recognize if a post is insight with the prior of it being near the poi
  // give up after timeout
}


PoseConf start_gate_net::post_0::Body(PoiDuration _poi_timeout)
{
  return find_post(_poi_timeout);
}

PoseConf start_gate_net::post_1::Body(PoiDuration _poi_timeout)
{
  return find_post(_poi_timeout);
}

PoseSuccess start_gate_net::success::Body(PoseConf_2 _pose_confs)
{
  //std::cout << "success" <<std::endl;
  if (_pose_confs.pose_confs[0].conf > 0.5 && _pose_confs.pose_confs[1].conf > 0.5)
  {
    Pose pose;
    pose.position.x = (_pose_confs.pose_confs[0].pose.position.x +
                       _pose_confs.pose_confs[1].pose.position.x)/2;
    pose.position.y = (_pose_confs.pose_confs[0].pose.position.y +
                       _pose_confs.pose_confs[1].pose.position.y)/2;
    return {pose, true};
  }
  return {Pose(), false};
}

void start_gate_net::go_through_gate::Body(Pose _pose)
{
  int argc = 0;
  char** argv = nullptr;
  ros::init(argc, argv, "approach_pois", true);
  ros::NodeHandle nh;
  move_to_strafe(nh, _pose.position.x, _pose.position.y);
  return;
}




void sigint_handler(int dummy)
{
  start_gate_net::PetriNet_Close();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "StartGateMission", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigint_handler);
  start_gate_net::PetriNet_Init();
  while(true);
  return 0;
}


