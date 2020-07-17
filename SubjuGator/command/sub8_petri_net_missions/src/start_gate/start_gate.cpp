#include <ros/ros.h>
#include <start_gate_net.hpp>
#include <sub8_missions2/start_gate.hpp>

Duration start_gate_net::get_timeout::Body()
{}

POIArray start_gate_net::get_next_pois::Body()
{}

void start_gate_net::approach_pois::Body(POIArray _pois)
{}

PoseConf start_gate_net::post_0::Body(POIDuration _poi_timeout)
{}

PoseConf start_gate_net::post_1::Body(POIDuration _poi_timeout)
{}

PoseSuccess start_gate_net::success::Body(PoseConfTuple _pose_confs)
{}

void start_gate_net::go_through_gate::Body(Pose)
{}


int main()
{
  return 0;
}








