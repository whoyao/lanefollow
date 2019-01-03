#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "JMT_gflags.h"
#include "common/struct/traj_structs.h"
#include "common/struct/lane_struct.h"
#include "JMT_curve1d.h"
#include "quartic_polynomial_curve1d.h"
#include "quintic_polynomial_curve1d.h"
#include "end_condition_sampler.h"
#include "reference_line.h"
#include "manager/TopologyManager.h"


namespace JMT {


    using State = std::array<double, 3>;
    using Condition = std::pair<State, double>;
    using TrajectorySets = std::vector<std::shared_ptr<PolynomialCurve1d>>;

    namespace TrajectoryCombiner {
        std::vector<CurvePoint> Combine(
                const std::vector<CurvePoint>& reference_line,
                const PolynomialCurve1d& lon_trajectory, const PolynomialCurve1d& lat_trajectory,
                double init_relative_time);

        std::vector<CurvePoint> Combine1d(
                const TopologyManager* topology,
                const PolynomialCurve1d& lon_trajectory, double init_relative_time);
    }


    class Trajectory1dGenerator {
    public:
        Trajectory1dGenerator(
                const std::array<double, 3>& lon_init_state);

        virtual ~Trajectory1dGenerator() = default;

        void GenerateTrajectorySets(
                double stop_s,
                double dis_to_obstacle,
                double target_speed,
                TrajectorySets* ptr_trajectory_sets) const;

    private:
        void GenerateSpeedProfilesForCruising(
                double target_speed,
                std::vector<std::shared_ptr<PolynomialCurve1d>>* ptr_lon_trajectory_bundle) const;

        void GenerateSpeedProfilesForStopping(
                double stop_point,
                std::vector<std::shared_ptr<PolynomialCurve1d>>* ptr_lon_trajectory_bundle) const;

        template <std::size_t N>
        void GenerateTrajectory1DBundle(
                const std::array<double, 3>& init_state,
                const std::vector<std::pair<std::array<double, 3>, double>>&
                end_conditions,
                std::vector<std::shared_ptr<PolynomialCurve1d>>* ptr_trajectory_bundle) const;

    private:
        std::array<double, 3> init_lon_state_;

        EndConditionSampler end_condition_sampler_;
    };

    template <>
    inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
            const std::array<double, 3>& init_state,
            const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
            std::vector<std::shared_ptr<PolynomialCurve1d>>* ptr_trajectory_bundle) const {
        CHECK_NOTNULL(ptr_trajectory_bundle);
        CHECK(!end_conditions.empty());

        ptr_trajectory_bundle->reserve(end_conditions.size());
        for (const auto& end_condition : end_conditions) {
            auto ptr_trajectory1d = std::make_shared<JMT_curve1d>(
                    std::shared_ptr<PolynomialCurve1d>(new QuarticPolynomialCurve1d(
                            init_state, {end_condition.first[1], end_condition.first[2]},
                            end_condition.second)));

            ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
            ptr_trajectory1d->set_target_time(end_condition.second);
            ptr_trajectory_bundle->push_back(ptr_trajectory1d);
        }
    }

    template <>
    inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<5>(
            const std::array<double, 3>& init_state,
            const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
            std::vector<std::shared_ptr<PolynomialCurve1d>>* ptr_trajectory_bundle) const {
        CHECK_NOTNULL(ptr_trajectory_bundle);
        CHECK(!end_conditions.empty());

        ptr_trajectory_bundle->reserve(end_conditions.size());
        for (const auto& end_condition : end_conditions) {
            auto ptr_trajectory1d = std::make_shared<JMT_curve1d>(
                    std::shared_ptr<PolynomialCurve1d>(new QuinticPolynomialCurve1d(
                            init_state, end_condition.first, end_condition.second)));

            ptr_trajectory1d->set_target_position(end_condition.first[0]);
            ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
            ptr_trajectory1d->set_target_time(end_condition.second);
            ptr_trajectory_bundle->push_back(ptr_trajectory1d);
        }
    }

}  // namespace JMT

#endif
// MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_
