#include <ros/ros.h>

#include <start_gate/start_gate_net.hpp>
#include <sub8_petri_net_missions/start_gate.hpp>


Duration start_gate_net::get_timeout::Body()
{
  std::cout << "get_timeout" <<std::endl;
  // init ros node
  //int argc = 0;
  //char** argv = nullptr;
  //ros::init(argc, argv, "get_timeout");
  // rosparam get start_gate timeout
  return Duration();
}

POI_2 start_gate_net::get_next_pois::Body()
{
  std::cout << "get_next_pois" <<std::endl;
  // init rosnode
  //int argc = 0;
  //char** argv = nullptr;
  //ros::init(argc, argv, "get_next_pois");
  // rosparam get the next pois
  return {POI(), POI()};
}

void start_gate_net::approach_pois::Body(POI_2 _pois)
{
  std::cout << "approach_pois" <<std::endl;
  // init rosnode
  // rosaction move to 1 meter short of the average of the pois
  return;
}

PoseConf find_post(PoiDuration _poi_timeout)
{
  std::cout << "find_post" <<std::endl;
  // init rosnode
  // try to recognize if a post is insight with the prior of it being near the poi
  // give up after timeout
  return {Pose(), 0.0};
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
  std::cout << "success" <<std::endl;
  return {Pose(), true};
}

void start_gate_net::go_through_gate::Body(Pose _pose)
{
  start_gate_net::PetriNet_Close();
  return;
}
/*
void sigint_handler(int dummy)
{
  start_gate_net::PetriNet_Close();
  exit(0);
}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "StartGateMission", ros::init_options::NoSigintHandler);
  //signal(SIGINT, sigint_handler);
  start_gate_net::PetriNet_Init();
  while(true);
  return 0;
}


