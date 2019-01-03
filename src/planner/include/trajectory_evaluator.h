//
// Created by huyao on 18-10-14.
//

#ifndef PROJECT_TRAJECTORY_EVALUATOR_H
#define PROJECT_TRAJECTORY_EVALUATOR_H


#define JMT_DEBUG


#include <array>
#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>


#include "JMT_gflags.h"
#include "polynomial_curve1d.h"
#include "common/util/log.h"
#include "common/struct/lane_struct.h"
#include "common/struct/traj_structs.h"
#include "manager/TopologyManager.h"
#include "tiggo_msgs/ObjectList.h"

namespace JMT {

    enum Result {
        VALID,
        LON_DYNAMIC_COLLISION,
        LON_STATIC_COLLISION,
        LON_VELOCITY_OUT_OF_BOUND,
        LON_ACCELERATION_OUT_OF_BOUND,
        LON_JERK_OUT_OF_BOUND,
    };

    class TrajectoryEvaluator {
        // normal use
        typedef std::pair<std::shared_ptr<PolynomialCurve1d>, double>
                PairCost;

        typedef std::pair<PairCost,Result>
                PairCostWithStatus;


        // auto tuning
        typedef std::pair<
                std::shared_ptr<PolynomialCurve1d>,
                std::pair<std::vector<double>, double>>
                PairCostWithComponents;

        typedef std::pair<PairCostWithComponents,Result>
                PairCostWithComponentsAndStatus;

    public:
        explicit TrajectoryEvaluator(
                const std::array<double, 3>& init_s,
                double stop_s,
                double dis_to_obstacle,
                double target_speed,
                const std::vector<DynamicObject> &dynamic_objects_sd,
                const std::vector<std::shared_ptr<PolynomialCurve1d>> *ptr_trajectory_sets);

        virtual ~TrajectoryEvaluator() = default;

        bool has_more_trajectory_pairs() const;

        std::size_t num_of_trajectory_pairs() const;

        std::shared_ptr<PolynomialCurve1d> next_top_trajectory_pair();

        Result top_trajectory_pair_status();

        double top_trajectory_pair_cost() const;

#ifdef JMT_DEBUG
        std::vector<double> get_top_cost_component() const;
#endif

    private:
        std::pair<double,Result>  Evaluate(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        std::pair<std::pair<std::vector<double>,double>,Result>  Evaluate2(
                const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        double LonComfortCost(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        double LonDynamicCollisionCost(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        double LonStaticCollisionCost(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        double LonCollisionCost(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;

        double LonObjectiveCost(const std::shared_ptr<PolynomialCurve1d>& lon_trajectory,
                                double target_speed ) const;

        double CentripetalAccelerationCost(
                const std::shared_ptr<PolynomialCurve1d>& lon_trajectory) const;


        struct CostComparator
                : public std::binary_function<const PairCostWithStatus&, const PairCostWithStatus&, bool> {
            bool operator()(const PairCostWithStatus& left, const PairCostWithStatus& right) const {
                return left.first.second > right.first.second;
            }
        };

        struct CostComponentComparator
                : public std::binary_function<const PairCostWithComponentsAndStatus&,
                        const PairCostWithComponentsAndStatus&, bool> {
            bool operator()(const PairCostWithComponentsAndStatus& left,
                            const PairCostWithComponentsAndStatus& right) const {
                return left.first.second.second > right.first.second.second;
            }
        };


    private:
        std::priority_queue<PairCostWithStatus, std::vector<PairCostWithStatus>, CostComparator>
                cost_queue_;

        std::priority_queue<PairCostWithComponentsAndStatus,
                std::vector<PairCostWithComponentsAndStatus>,
                CostComponentComparator>
                cost_queue_with_components_;

        std::vector<DynamicObject> dynamic_objects_;

        std::array<double, 3> init_s_;

        CurvePoint planning_init_point_;

        double stop_s_;

        double dis_to_obstacle_;

        double target_speed_;
    };


    namespace ConstraintChecker1d{

        bool IsValidLongitudinalTrajectory(const PolynomialCurve1d& lon_trajectory);

        bool IsValidLateralTrajectory(const PolynomialCurve1d& lat_trajectory,
                                      const PolynomialCurve1d& lon_trajectory);
    }


}  // namespace JMT


#endif //PROJECT_TRAJECTORY_EVALUATOR_H
