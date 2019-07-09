/**
 * Author: Patrick Emami
 * Date: 9/29/15
 *
 */
#ifndef TGEN_COMMON_H_
#define TGEN_COMMON_H_

#include <string>

using std::string;

namespace sub8
{
namespace trajectory_generator
{
// Strings used for logging statements in the TGEN
class TGenMsgs
{
public:
  // PlannerStatus responses
  static constexpr const char* INVALID_START = "Invalid start state or no start state specified";
  static constexpr const char* INVALID_GOAL = "Invalid goal state";
  static constexpr const char* UNRECOGNIZED_GOAL_TYPE = "The goal is of a type that a planner does not "
                                                        "recognize";
  static constexpr const char* TIMEOUT = "The planner failed to find a solution";
  static constexpr const char* APPROXIMATE_SOLUTION = "The planner found an approximate solution";
  static constexpr const char* EXACT_SOLUTION = "The planner found an exact solution";
  static constexpr const char* CRASH = "The planner crashed!";

  // Trajectory Validation
  static constexpr const char* REPLAN_NEEDED = "Current trajectory is blocked-- starting naive replan";
  static constexpr const char* TRAJECTORY_VALIDATED =
      "Current trajectory successfully validated against new traversability "
      "map.";
};
}
}
#endif /* TGEN_COMMON */