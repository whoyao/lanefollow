//
// Created by cyber on 18-10-13.
//

#ifndef PROJECT_JMT_PLANNER_H
#define PROJECT_JMT_PLANNER_H


#include "tiggo_msgs/ObjectList.h"
#include "TopologyManager.h"

#include "JMT_gflags.h"
#include "trajectory_decouple_generator.h"
#include "trajectory_evaluator.h"
#include "curveSmoother.h"
#include "reference_line.h"
#include "common/struct/traj_structs.h"


namespace JMT {

    class JMTPlanner {
    public:
        JMTPlanner(planner::TopologyManager *topology)
                : topology_manager_(topology), init_s_({0.0, 0.0, 0.0}), init_d_({0.0, 0.0, 0.0}) {};

    public:
        tiggo_msgs::LocalTrajList plan (
                const CurvePoint& planning_init_point,
                double stop_s,
                double target_speed);

        tiggo_msgs::LocalTrajList plan_debug (
                const CurvePoint& planning_init_point,
                double stop_s,
                double target_speed);

        tiggo_msgs::LocalTrajList JMTPlanner::plan_new(
                const CurvePoint& planning_init_point,
                double stop_s,
                double target_speed);

        void update(const tiggo_msgs::ObjectList& dynamic_objects,
                    const CurvePoint& current_pose,
                    const double dis_to_static_obstacle);

    private:
        tf::TransformListener listener_;
        TopologyManager *topology_manager_;
        std::vector<DynamicObject> dynamic_objects_sd_;
        std::vector<CurvePoint> reference_points_;

        CurvePoint current_pose_;
        double stop_s_;
        double dis_to_static_obstacle_;
        double target_speed_;
        std::array<double, 3> init_s_;
        std::array<double, 3> init_d_;

    };
} // namespace JMT

#endif //PROJECT_JMT_PLANNER_H
