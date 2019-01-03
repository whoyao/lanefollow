/**
 * @file
 **/

#include "end_condition_sampler.h"

#include <algorithm>
#include <utility>

#include "common/util/log.h"
#include "JMT_gflags.h"


namespace JMT {

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(
    const State& init_s)
    : init_s_(init_s),
      feasible_region_(init_s) {}


std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  CHECK_GT(FLAGS_num_velocity_sample, 1);

  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_of_time_samples = 8;
  std::array<double, num_of_time_samples> time_samples;
  for (std::size_t i = 0; i + 1 < num_of_time_samples; ++i) {
    time_samples[i] = FLAGS_trajectory_time_length - i;
  }
  time_samples[num_of_time_samples - 1] = FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {
    double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);
    double v_lower = feasible_region_.VLower(time);

    State lower_end_s = {0.0, v_lower, 0.0};
    end_s_conditions.emplace_back(lower_end_s, time);

    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);

    double v_range = v_upper - v_lower;
    // Number of sample velocities
    std::size_t num_of_mid_points = std::min(
        static_cast<std::size_t>(FLAGS_num_velocity_sample - 2),
        static_cast<std::size_t>(v_range / FLAGS_min_velocity_sample_gap));

    if (num_of_mid_points > 0) {
      double velocity_seg = v_range / (num_of_mid_points + 1);
      for (std::size_t i = 1; i <= num_of_mid_points; ++i) {
        State end_s = {0.0, v_lower + velocity_seg * i, 0.0};
        end_s_conditions.emplace_back(end_s, time);
      }
    }
  }
  return end_s_conditions;
}

std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForStopping(
    const double ref_stop_point) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 4;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = FLAGS_trajectory_time_length - i;
  }
  time_sections[num_time_section - 1] = FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_sections) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    end_s_conditions.emplace_back(end_s, time);
  }
  return end_s_conditions;
}


}  // namespace JMT
