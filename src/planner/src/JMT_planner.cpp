//
// Created by cyber on 18-10-13.
//

#include "JMT_planner.h"
#include "common/math/cartesian_frenet_conversion.h"



namespace JMT {

    void ComputeInitFrenetState(const CurvePoint& matched_point,
                                const CurvePoint& cartesian_state,
                                std::array<double, 3>* ptr_s,
                                std::array<double, 3>* ptr_d) {
        cyber_common::math::CartesianFrenetConverter::cartesian_to_frenet(
                matched_point.s, matched_point.x, matched_point.y,
                matched_point.theta, matched_point.kappa, matched_point.kappa_prime,
                cartesian_state.x, cartesian_state.y,
                cartesian_state.v, cartesian_state.a,
                cartesian_state.theta,
                cartesian_state.kappa, ptr_s, ptr_d);
    }


    double CalculateDrivedS(const std::vector<CurvePoint>& trajectory, const CurvePoint& cp){
        double min_dis = std::numeric_limits<double>::infinity();
        double current_dis;
        for(const auto& tj : trajectory){
            current_dis = cp.DistanceSquareTo(tj);
            if(current_dis > min_dis)
                return tj.s;
            min_dis = std::min(min_dis, current_dis);
        }
        return trajectory.back().s;
    }

    // TODO : dynamic car transform, lane position, grid_map_check;
    tiggo_msgs::LocalTrajList JMTPlanner::plan_debug(
            const CurvePoint& planning_init_point,
            const double stop_s,
            const double target_speed) {

        AINFO << "Dis to obstacle is: " << dis_to_static_obstacle_;
        auto start = ros::WallTime::now();

        CurvePoint matched_point = topology_manager_->MatchToPath(planning_init_point.x, planning_init_point.y);

        std::array<double, 3> init_s = {0.0, 0.0, 0.0};
        std::array<double, 3> init_d = {0.0, 0.0, 0.0};
        ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
        Trajectory1dGenerator trajectory_1d_generator(init_s);
        TrajectorySets trajectory_sets;

        trajectory_1d_generator.GenerateTrajectorySets(stop_s, dis_to_static_obstacle_, target_speed_,
                                                       &trajectory_sets);

        auto end = ros::WallTime::now();
        AERROR << "Generate Duration is " << end - start ;
        start = end;

        TrajectoryEvaluator trajectory_evaluator(
                init_s, stop_s, dis_to_static_obstacle_, target_speed,
                dynamic_objects_sd_, &trajectory_sets);

        AWARN << "Total trajectories number: " << trajectory_evaluator.num_of_trajectory_pairs();
        end = ros::WallTime::now();
        AERROR << "Evaluate Duration is " << end - start ;

        std::size_t sd_dynamic_collision_failure_count = 0;
        std::size_t sd_static_collision_failure_count = 0;


        tiggo_msgs::LocalTrajList debug_res; // TODO : remove
        while (trajectory_evaluator.has_more_trajectory_pairs()) {
            Result trajectory_result = trajectory_evaluator.top_trajectory_pair_status();
            if (trajectory_result != Result::VALID) {
                switch (trajectory_result) {
                    case Result::LON_DYNAMIC_COLLISION:
                        sd_dynamic_collision_failure_count += 1;
                        break;
                    case Result::LON_STATIC_COLLISION:
                        sd_static_collision_failure_count += 1;
                        break;
                    default:
                        // Intentional empty
                        break;
                }
                continue;
            }

#ifdef JMT_DEBUG
            auto cost_component = trajectory_evaluator.get_top_cost_component();
#endif

            double trajectory_pair_cost =
                    trajectory_evaluator.top_trajectory_pair_cost();

            auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

            // combine two 1d trajectories to one 2d trajectory
            auto combined_trajectory = TrajectoryCombiner::Combine1d(
                    topology_manager_, *trajectory_pair., 0.0);

            tiggo_msgs::LocalTrajList res;
            for(const auto& cp : combined_trajectory){
                tiggo_msgs::LocalTrajPoint temp_point;
                temp_point.position.x = cp.x;
                temp_point.position.y = cp.y;
                temp_point.velocity = cp.v;
                temp_point.s = cp.s;
                temp_point.theta = cp.theta;
                res.points.push_back(std::move(temp_point));
            }

            double detect_horizon = std::min(100.0, combined_trajectory.back().s - combined_trajectory.front().s);

            double end_ss = std::dynamic_pointer_cast<JMT_curve1d>(trajectory_pair)->target_position();
            double end_st = std::dynamic_pointer_cast<JMT_curve1d>(trajectory_pair)->ParamLength();
            double end_sv = std::dynamic_pointer_cast<JMT_curve1d>(trajectory_pair)->target_velocity();

            if(debug_res.points.empty()) {    // TODO: remove
                debug_res = res;
                local_traj_->SetTrajectory(debug_res);
                local_traj_->PubTrajectory();
                AWARN << "Position: " << end_ss << "; time: " << end_st << "; velocty: " << end_sv;
            }
//                    return res;
        }
//        return debug_res; // TODO: remove

        AWARN << "LON_DYNAMIC_COLLISION number: " << sd_dynamic_collision_failure_count;
        AWARN << "STATIC COLLISION number: " << sd_static_collision_failure_count;
        tiggo_msgs::LocalTrajList empty;
        return empty;

    }


    void JMTPlanner::update(const tiggo_msgs::ObjectList& dynamic_objects,
            const CurvePoint& current_pose, const double dis_to_static_obstacle){

        // first get current pose
        current_pose_ = current_pose;
        topology_manager_->UpdateCurrentPose(current_pose);
        dis_to_static_obstacle_ = dis_to_static_obstacle;

        // transform current_pose
        auto planning_init_point = current_pose;
        // 1. compute the matched point of the init planning point on the referenceline.
        CurvePoint matched_point = topology_manager_->MatchToPath(planning_init_point.x, planning_init_point.y);
        // 2. according to the matched point, compute the init state in Frenet frame.
        ComputeInitFrenetState(matched_point, planning_init_point, &init_s_, &init_d_);


        dynamic_objects_sd_.clear();
        for(const auto& object : dynamic_objects.objects){
            geometry_msgs::PoseStamped object_local, object_world;
            object_local.pose.position.x = object.position.x;
            object_local.pose.position.y = object.position.y;
            object_local.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(object.velocity.y, object.velocity.x));
            if (!TransformPose(&listener_, "base_link", "world", object_local, object_world))
                continue;///TF转换失败
            double v = sqrt(object.velocity.x*object.velocity.x + object.velocity.y*object.velocity.y);
            double theta = tf::getYaw(object_world.pose.orientation);

            CurvePoint matched_point_ob = topology_manager_->MatchToPath(object_world.pose.position.x,
                                                                      object_world.pose.position.y);
            CurvePoint cartesian_state;
            cartesian_state.x = object_world.pose.position.x;
            cartesian_state.y = object_world.pose.position.y;
            cartesian_state.theta = theta;
            cartesian_state.v = v;
            cartesian_state.a = 0.0;
            cartesian_state.kappa = 0.0;

            std::array<double, 3> init_s = {0.0, 0.0, 0.0};
            std::array<double, 3> init_d = {0.0, 0.0, 0.0};
            ComputeInitFrenetState(matched_point_ob, cartesian_state, &init_s, &init_d);

            DynamicObject dy_object(init_s, init_d);
            dy_object.half_length = object.box_size.x/2;
            dy_object.half_width = object.box_size.y/2;

            dynamic_objects_sd_.push_back(std::move(dy_object));
        }
    }

    tiggo_msgs::LocalTrajList JMTPlanner::plan_new(const CurvePoint& planning_init_point,
                                                   const double stop_s,
                                                   const double target_speed) {

        auto start = ros::WallTime::now();

        CurvePoint matched_point = topology_manager_->MatchToPath(planning_init_point.x, planning_init_point.y);

        std::array<double, 3> init_s = {0.0, 0.0, 0.0};
        std::array<double, 3> init_d = {0.0, 0.0, 0.0};
        ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
        Trajectory1dGenerator trajectory_1d_generator(init_s);
        TrajectorySets trajectory_sets;

        trajectory_1d_generator.GenerateTrajectorySets(stop_s, dis_to_static_obstacle_, target_speed,
                                                          &trajectory_sets);

        auto end = ros::WallTime::now();
        AERROR << "Generate Duration is " << end - start ;
        start = end;

        TrajectoryEvaluator trajectory_evaluator(
                init_s, stop_s, dis_to_static_obstacle_, target_speed,
                dynamic_objects_sd_, &trajectory_sets);

        AWARN << "Total trajectories number: " << trajectory_evaluator.num_of_trajectory_pairs();
        end = ros::WallTime::now();
        AERROR << "Evaluate Duration is " << end - start ;
        start = end;

        std::size_t sd_dynamic_collision_failure_count = 0;
        std::size_t sd_static_collision_failure_count = 0;


        while (trajectory_evaluator.has_more_trajectory_pairs()) {
            Result trajectory_result = trajectory_evaluator.top_trajectory_pair_status();
            if (trajectory_result != Result::VALID) {
                switch (trajectory_result) {
                    case Result::LON_DYNAMIC_COLLISION:
                        sd_dynamic_collision_failure_count += 1;
                        break;
                    case Result::LON_STATIC_COLLISION:
                        sd_static_collision_failure_count += 1;
                        break;
                    default:
                        // Intentional empty
                        break;
                }
                continue;
            }

#ifdef JMT_DEBUG
            auto cost_component = trajectory_evaluator.get_top_cost_component();
#endif

            double trajectory_pair_cost =
                    trajectory_evaluator.top_trajectory_pair_cost();

            auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

            // combine two 1d trajectories to one 2d trajectory
            auto combined_trajectory = TrajectoryCombiner::Combine1d(
                    topology_manager_, *trajectory_pair, 0.0);

            tiggo_msgs::LocalTrajList res;
            for(const auto& cp : combined_trajectory){
                tiggo_msgs::LocalTrajPoint temp_point;
                temp_point.position.x = cp.x;
                temp_point.position.y = cp.y;
                temp_point.velocity = cp.v;
                temp_point.s = cp.s;
                temp_point.theta = cp.theta;
                res.points.push_back(std::move(temp_point));
            }

            double detect_horizon = std::min(100.0, combined_trajectory.back().s - combined_trajectory.front().s);

            return res;
        }

        AWARN << "LON_DYNAMIC_COLLISION number: " << sd_dynamic_collision_failure_count;
        AWARN << "STATIC COLLISION number: " << sd_static_collision_failure_count;
        tiggo_msgs::LocalTrajList empty;
        return empty;

    }

} // namespace JMt