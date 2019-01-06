#include "trajectory_decouple_generator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <planner/include/TopologyManager.h>

#include "common/math/cartesian_frenet_conversion.h"




namespace JMT {

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
//    typedef std::array<double, 3> State;
//    typedef std::pair<State, double> Condition;
    typedef std::vector<std::shared_ptr<PolynomialCurve1d>> Trajectory1DBundle;


//    std::vector<CurvePoint> TrajectoryCombiner::Combine(
//            const std::vector<CurvePoint>& reference_line, const PolynomialCurve1d& lon_trajectory,
//            const PolynomialCurve1d& lat_trajectory, const double init_relative_time) {
//      std::vector<CurvePoint> combined_trajectory;
//
//      double s0 = lon_trajectory.Evaluate(0, 0.0);
//      double s_ref_max = reference_line.back().s;
//      double accumulated_trajectory_s = 0.0;
//      CurvePoint prev_trajectory_point;
//      prev_trajectory_point.x = std::numeric_limits<double>::infinity();
//      prev_trajectory_point.y = std::numeric_limits<double>::infinity();
//
//      double last_s = -FLAGS_lattice_epsilon;
//      double t_param = 0.0;
//      while (t_param < FLAGS_trajectory_time_length) {
//        // linear extrapolation is handled internally in LatticeTrajectory1d;
//        // no worry about t_param > lon_trajectory.ParamLength() situation
//        double s = lon_trajectory.Evaluate(0, t_param);
//        if (last_s > 0.0) {
//          s = std::max(last_s, s);
//        }
//        last_s = s;
//
//        double s_dot =
//                std::max(FLAGS_lattice_epsilon, lon_trajectory.Evaluate(1, t_param));
//        double s_ddot = lon_trajectory.Evaluate(2, t_param);
//        if (s > s_ref_max) {
//          break;
//        }
//
//        double s_param = s - s0;
//        // linear extrapolation is handled internally in LatticeTrajectory1d;
//        // no worry about s_param > lat_trajectory.ParamLength() situation
//        double d = lat_trajectory.Evaluate(0, s_param);
//        double d_prime = lat_trajectory.Evaluate(1, s_param);
//        double d_pprime = lat_trajectory.Evaluate(2, s_param);
//
//        CurvePoint matched_ref_point = PathMatcher::MatchToPath(reference_line, s);
//
//        double x = 0.0;
//        double y = 0.0;
//        double theta = 0.0;
//        double kappa = 0.0;
//        double v = 0.0;
//        double a = 0.0;
//
//        const double rs = matched_ref_point.s;
//        const double rx = matched_ref_point.x;
//        const double ry = matched_ref_point.y;
//        const double rtheta = matched_ref_point.theta;
//        const double rkappa = matched_ref_point.kappa;
//        const double rdkappa = matched_ref_point.kappa_prime;
//
//        std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
//        std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
//        cyber_common::math::CartesianFrenetConverter::frenet_to_cartesian(
//                rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
//                &theta, &kappa, &v, &a);
//
//        if (prev_trajectory_point.x != std::numeric_limits<double>::infinity() &&
//            prev_trajectory_point.y != std::numeric_limits<double>::infinity()) {
//          double delta_x = x - prev_trajectory_point.x;
//          double delta_y = y - prev_trajectory_point.y;
//          double delta_s = std::hypot(delta_x, delta_y);
//          accumulated_trajectory_s += delta_s;
//        }
//
//        CurvePoint cp;
//        cp.x = x;
//        cp.y = y;
//        cp.s =accumulated_trajectory_s;
//        cp.theta = theta;
//        cp.kappa = kappa;
//        cp.v = v;
//        cp.a = a;
//        cp.t = t_param + init_relative_time;
//
//        prev_trajectory_point = cp;
//
//        combined_trajectory.push_back(std::move(cp));
//
//        t_param = t_param + FLAGS_trajectory_time_resolution;
//      }
//      return combined_trajectory;
//    }


    std::vector<CurvePoint> TrajectoryCombiner::Combine1d(
            const std::shared_ptr<TopologyManager> topology,
            const PolynomialCurve1d& lon_trajectory, const double init_relative_time){
      std::vector<CurvePoint> combined_trajectory;

      double s0 = lon_trajectory.Evaluate(0, 0.0);
      double s_ref_max = topology->GetReferenceLineEndS();
      CurvePoint prev_trajectory_point;
      prev_trajectory_point.x = std::numeric_limits<double>::infinity();
      prev_trajectory_point.y = std::numeric_limits<double>::infinity();
      double accumulated_trajectory_s = 0.0;

      double last_s = -FLAGS_lattice_epsilon;
      double t_param = 0.0;
      while (t_param < FLAGS_trajectory_time_length) {
        // linear extrapolation is handled internally in LatticeTrajectory1d;
        // no worry about t_param > lon_trajectory.ParamLength() situation
        double s = lon_trajectory.Evaluate(0, t_param);
        if (last_s > 0.0) {
          s = std::max(last_s, s);
        }
        last_s = s;

        double s_dot =
                std::max(FLAGS_lattice_epsilon, lon_trajectory.Evaluate(1, t_param));
        double s_ddot = lon_trajectory.Evaluate(2, t_param);
        if (s > s_ref_max) {
          break;
        }

        // linear extrapolation is handled internally in LatticeTrajectory1d;
        // no worry about s_param > lat_trajectory.ParamLength() situation
        double d = FLAGS_car_default_d;
        double d_prime = 0.0;
        double d_pprime = 0.0;

        CurvePoint matched_ref_point = topology->MatchToPath(s);

        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v = 0.0;
        double a = 0.0;

        const double rs = matched_ref_point.s;
        const double rx = matched_ref_point.x;
        const double ry = matched_ref_point.y;
        const double rtheta = matched_ref_point.theta;
        const double rkappa = matched_ref_point.kappa;
        const double rdkappa = matched_ref_point.kappa_prime;

        std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
        std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
        cyber_common::math::CartesianFrenetConverter::frenet_to_cartesian(
                rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
                &theta, &kappa, &v, &a);

        if (prev_trajectory_point.x != std::numeric_limits<double>::infinity() &&
            prev_trajectory_point.y != std::numeric_limits<double>::infinity()) {
          double delta_x = x - prev_trajectory_point.x;
          double delta_y = y - prev_trajectory_point.y;
          double delta_s = std::hypot(delta_x, delta_y);
          accumulated_trajectory_s += delta_s;
        }

        CurvePoint cp;
        cp.x = x;
        cp.y = y;
        cp.s =accumulated_trajectory_s;
        cp.theta = theta;
        cp.kappa = kappa;
        cp.v = v;
        cp.a = a;
        cp.t = t_param + init_relative_time;

        prev_trajectory_point = cp;

        combined_trajectory.push_back(std::move(cp));

        t_param = t_param + FLAGS_trajectory_time_resolution;
      }
      return combined_trajectory;
    }

    Trajectory1dGenerator::Trajectory1dGenerator(
            const State& lon_init_state)
            : init_lon_state_(lon_init_state),
              end_condition_sampler_(lon_init_state){
    }

    void Trajectory1dGenerator::GenerateTrajectorySets(
            double stop_s,
            double dis_to_obstacle,
            double target_speed,
            TrajectorySets* ptr_trajectory_sets) const{
      if(dis_to_obstacle < std::numeric_limits<double>::infinity()) {
        GenerateSpeedProfilesForStopping(std::min(
                std::max(init_lon_state_[0] + dis_to_obstacle - FLAGS_head_offset, init_lon_state_[0]),
                stop_s), ptr_trajectory_sets);
      } else if(stop_s < std::numeric_limits<double>::infinity()) {
        GenerateSpeedProfilesForStopping(stop_s, ptr_trajectory_sets);
      }
      GenerateSpeedProfilesForCruising(target_speed, ptr_trajectory_sets);
    }

    void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
            const double target_speed,
            Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
      ADEBUG << "cruise speed is  " << target_speed;
      auto end_conditions =
              end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);
      if (end_conditions.empty()) {
        return;
      }

      // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
      // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
      // invoke the common function to generate trajectory bundles.
      GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                    ptr_lon_trajectory_bundle);
    }

    void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
            const double stop_point,
            Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
      ADEBUG << "stop point is " << stop_point;
      auto end_conditions =
              end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);
      if (end_conditions.empty()) {
        return;
      }

      // Use the common function to generate trajectory bundles.
      GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                    ptr_lon_trajectory_bundle);
    }



}  // namespace JMT
