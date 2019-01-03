#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_

#include <array>
#include <utility>
#include <vector>
#include <memory>
#include <string>

#include "feasible_region.h"

namespace JMT {

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
 public:
  EndConditionSampler(
      const std::array<double, 3>& init_s);

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(double ref_stop_point) const;

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  FeasibleRegion feasible_region_;
};

}  // namespace JMT

#endif
// MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_END_CONDITION_SAMPLER_H_
