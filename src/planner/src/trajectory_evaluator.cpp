//
// Created by huyao on 18-10-14.
//

#include "trajectory_evaluator.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "common/math/cartesian_frenet_conversion.h"

#define _PATH_ "/tmp/cost_fifo.pipe"
#define _CAR_PATH_ "/tmp/car_fifo.pipe"



namespace JMT {

    using PtrTrajectory1d = std::shared_ptr<PolynomialCurve1d>;
    using TrajectorySets = std::vector<PtrTrajectory1d>;

namespace {
    inline double logistic (const double x, const double max_cut_off) {
        return 2.0 / ( 1 + exp(-x*3/max_cut_off)) - 1.0;
    }
}

    void cost_pipe(std::stringstream &ss){
        static bool fifo_exist = false;
        char buf[4096];

        if(!fifo_exist) {
            if(mkfifo(_PATH_, 0666 | S_IFIFO)){
                AERROR << "FIFO create error!";
            }
            fifo_exist = true;
        }
        else {
            int fd = open(_PATH_, O_RDONLY|O_NONBLOCK);
            if(fd < 0){
                AERROR << "FIFO open error!";
            } else {
                read(fd, buf, sizeof(buf));
                close(fd);
            }


            int fd2 = open(_PATH_, O_WRONLY|O_NONBLOCK);
            if(fd2 < 0){
                AERROR << "FIFO open error!";
            } else {

                if(write(fd2, ss.str().c_str(), strlen(ss.str().c_str())+1) < 0){
                    AERROR << "FIFO write error!";
                } else {
                    close(fd2);
                }
            }
        }
    }

    void car_pipe(std::stringstream &ss){
        static bool fifo_exist = false;
        char buf[4096];

        if(!fifo_exist) {
            if(mkfifo(_CAR_PATH_, 0666 | S_IFIFO)){
                AERROR << "FIFO create error!";
            }
            fifo_exist = true;
        }
        else {
            int fd = open(_CAR_PATH_, O_RDONLY|O_NONBLOCK);
            if(fd < 0){
                AERROR << "FIFO open error!";
            } else {
                read(fd, buf, sizeof(buf));
                close(fd);
            }


            int fd2 = open(_CAR_PATH_, O_WRONLY|O_NONBLOCK);
            if(fd2 < 0){
                AERROR << "FIFO open error!";
            } else {

                if(write(fd2, ss.str().c_str(), strlen(ss.str().c_str())+1) < 0){
                    AERROR << "FIFO write error!";
                } else {
                    close(fd2);
                }
            }
        }
    }

    TrajectoryEvaluator::TrajectoryEvaluator(
            const std::array<double, 3>& init_s,
            const double delta_t,
            const double stop_s,
            const double dis_to_obstacle,
            const double target_speed,
            const std::vector<DynamicObject> &dynamic_objects_sd,
            const TrajectorySets *ptr_trajectory_sets)
            :  init_s_(init_s), dis_to_obstacle_(dis_to_obstacle),
               stop_s_(stop_s), delta_t_(delta_t), target_speed_(target_speed),
               dynamic_objects_(dynamic_objects_sd) {

        const double end_time = FLAGS_trajectory_time_length;

        std::stringstream ss;
        // if we have a stop point along the reference line,
        // filter out the lon. trajectories that pass the stop point.
        for (const auto& trajectory : *ptr_trajectory_sets) {
            double lon_end_s = trajectory->Evaluate(0, end_time);
            if (init_s[0] < stop_s &&
                lon_end_s + FLAGS_lattice_stop_buffer > stop_s) {
                continue;
            }

            if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*(trajectory))) {
                continue;
            }
#ifdef JMT_DEBUG
/////output cost value
#ifdef JMT_VISUAL

            std::pair<std::pair<std::vector<double>,double>,Result> cost_with_component
                    = EvaluateDebug(trajectory, ss);  // TODO: update here
#else
            std::pair<std::pair<std::vector<double>,double>,Result> cost_with_component
                    = Evaluate2(trajectory);  // TODO: update here
#endif
            cost_queue_with_components_.emplace(std::make_pair(
                    std::make_pair(trajectory,cost_with_component.first),
                    cost_with_component.second));
#else
            std::pair<double,Result> cost = Evaluate(stop_s, target_speed, dis_to_obstacle, trajectory);
            cost_queue_.emplace(std::make_pair(std::make_pair(trajectory, cost.first),cost.second));
#endif
        }

#ifdef JMT_VISUAL
        cost_pipe(ss);
        std::stringstream ss_car;
        ss_car << init_s[0] << ";" << init_s[1] << ";" << init_s[2] << ";" << -6.0 << ";"
               << 0.0 << ";" << 0.0 << ";" << 1.5 << ";" << 2.0
               << std::endl;
        for(const auto &obstacle : dynamic_objects_sd){
            ss_car << obstacle.S[0] << ";" << obstacle.S[1] << ";" << obstacle.S[2] << ";" << obstacle.D[0] << ";"
             << obstacle.D[1] << ";" << obstacle.D[2] << ";" << obstacle.half_width << ";" << obstacle.half_length
                                                                                           << std::endl;
        }
//        AWARN << ss_car.str();
        car_pipe(ss_car);
#endif

        ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();

    }

    bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
#ifdef JMT_DEBUG
    return !cost_queue_with_components_.empty();
#else
    return !cost_queue_.empty();
#endif
    }

    std::size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
#ifdef JMT_DEBUG
        return cost_queue_with_components_.size();
#else
        return cost_queue_.size();
#endif
    }

    PtrTrajectory1d
    TrajectoryEvaluator::next_top_trajectory_pair() {
#ifdef JMT_DEBUG
        auto top = cost_queue_with_components_.top();
        cost_queue_with_components_.pop();
        return top.first.first;
#else
        CHECK(has_more_trajectory_pairs());
        auto top = cost_queue_.top();
        cost_queue_.pop();
        return top.first.first;
#endif
    }

    Result TrajectoryEvaluator::top_trajectory_pair_status() {
#ifdef JMT_DEBUG
        Result res = cost_queue_with_components_.top().second;
        if(res != Result::VALID){
            cost_queue_with_components_.pop();
        }
        return res;
#else
        Result res = cost_queue_.top().second;
        if(res != Result::VALID){
            cost_queue_.pop();
        }
        return res;
#endif
    }

    double TrajectoryEvaluator::top_trajectory_pair_cost() const {
#ifdef JMT_DEBUG
        return cost_queue_with_components_.top().first.second.second;
#else
        return cost_queue_.top().first.second;
#endif
    }


#ifdef JMT_DEBUG
    std::vector<double> TrajectoryEvaluator::get_top_cost_component() const {
        auto top = cost_queue_with_components_.top();
        return top.first.second.first;
}
#endif

    TrajectoryEvaluator::TrajectoryEvaluator(
            const std::array<double, 3>& init_s,
            double stop_s,
            double dis_to_obstacle,
            const std::vector<DynamicObject> &dynamic_objects_sd,
            const std::vector<std::shared_ptr<PolynomialCurve1d>> *ptr_trajectory_sets)
                : init_s_(init_s), dis_to_obstacle_(dis_to_obstacle),
                stop_s_(stop_s), dynamic_objects_(dynamic_objects_sd){
        const double end_time = FLAGS_trajectory_time_length;

        std::stringstream ss;
        // if we have a stop point along the reference line,
        // filter out the lon. trajectories that pass the stop point.
        for (const auto& trajectory : *ptr_trajectory_sets) {
            double lon_end_s = trajectory->Evaluate(0, end_time);
            if (init_s[0] < stop_s &&
                lon_end_s + FLAGS_lattice_stop_buffer > stop_s) {
                continue;
            }

            if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*(trajectory))) {
                continue;
            }
#ifdef JMT_DEBUG
/////output cost value
#ifdef JMT_VISUAL

            std::pair<std::pair<std::vector<double>,double>,Result> cost_with_component
                    = EvaluateEmergencyDebug(trajectory, ss);  // TODO: update here
#else
            std::pair<std::pair<std::vector<double>,double>,Result> cost_with_component
                    = EvaluateEmergency2(trajectory);  // TODO: update here
#endif
            cost_queue_with_components_.emplace(std::make_pair(
                    std::make_pair(trajectory,cost_with_component.first),
                    cost_with_component.second));
#else
            std::pair<double,Result> cost = EvaluateEmergency(stop_s, target_speed, dis_to_obstacle, trajectory);
            cost_queue_.emplace(std::make_pair(std::make_pair(trajectory, cost.first),cost.second));
#endif
        }

#ifdef JMT_VISUAL
        cost_pipe(ss);
        std::stringstream ss_car;
        ss_car << init_s[0] << ";" << init_s[1] << ";" << init_s[2] << ";" << -6.0 << ";"
               << 0.0 << ";" << 0.0 << ";" << 1.5 << ";" << 2.0
               << std::endl;
        for(const auto &obstacle : dynamic_objects_sd){
            ss_car << obstacle.S[0] << ";" << obstacle.S[1] << ";" << obstacle.S[2] << ";" << obstacle.D[0] << ";"
                   << obstacle.D[1] << ";" << obstacle.D[2] << ";" << obstacle.half_width << ";" << obstacle.half_length
                   << std::endl;
        }
//        AWARN << ss_car.str();
        car_pipe(ss_car);
#endif

        ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();
    }

    double LonObjectiveCostDebug(
            const PtrTrajectory1d& lon_trajectory,
            const double target_speed, std::stringstream &ss ) {
        double t_max = lon_trajectory->ParamLength();
        double dist_s =
                lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

        double end_v = lon_trajectory->Evaluate(1, t_max);
        double delta_speed = std::fabs(target_speed-end_v);
        double delta_dist = std::max(0.0, 100 - dist_s);


        double speed_cost = logistic(delta_speed, 10);
        double dist_travelled_cost = logistic(delta_dist, 100);

        ss << dist_s << ";" << end_v << ";" << speed_cost << ";" << dist_travelled_cost << ";" <<
             (speed_cost * FLAGS_weight_target_speed + dist_travelled_cost * FLAGS_weight_dist_travelled) /
             (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled) << std::endl;

        return (speed_cost * FLAGS_weight_target_speed +
                dist_travelled_cost * FLAGS_weight_dist_travelled) /
               (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
    }

    std::pair<std::pair<std::vector<double>,double>,Result> TrajectoryEvaluator::EvaluateEmergencyDebug(
            const PtrTrajectory1d& lon_trajectory, std::stringstream &ss) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        Result result = Result::VALID;

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            result = Result::LON_STATIC_COLLISION;
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
            result = Result::LON_DYNAMIC_COLLISION;
        }

        double lon_objective_cost = 0.0;

        double lon_jerk_cost = LonComfortCost(lon_trajectory);
//        double lon_jerk_cost = 0.0;

//        double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
        double centripetal_acc_cost = 0.0;


        std::vector<double> res_vec = {lon_objective_cost, lon_jerk_cost, collision_cost, centripetal_acc_cost};

        ss << lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()) - lon_trajectory->Evaluate(0, 0.0)
           << ";" << lon_trajectory->Evaluate(1, lon_trajectory->ParamLength()) << ";" << lon_jerk_cost
           << ";" << collision_cost << std::endl;


        return std::make_pair(std::make_pair(res_vec, lon_objective_cost * FLAGS_weight_lon_objective +
                                                      lon_jerk_cost * FLAGS_weight_lon_jerk +
                                                      std::fabs(collision_cost) * FLAGS_weight_lon_collision +
                                                      centripetal_acc_cost * FLAGS_weight_centripetal_acceleration),
                              result);

    }

    std::pair<std::pair<std::vector<double>,double>,Result> TrajectoryEvaluator::EvaluateEmergency2(
            const PtrTrajectory1d& lon_trajectory) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        Result result = Result::VALID;

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            result = Result::LON_STATIC_COLLISION;
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
            result = Result::LON_DYNAMIC_COLLISION;
        }

        double lon_objective_cost = 0.0;

        double lon_jerk_cost = LonComfortCost(lon_trajectory);
//        double lon_jerk_cost = 0.0;

//        double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
        double centripetal_acc_cost = 0.0;


        std::vector<double> res_vec = {lon_objective_cost, lon_jerk_cost, collision_cost, centripetal_acc_cost};

        return std::make_pair(std::make_pair(res_vec, lon_objective_cost * FLAGS_weight_lon_objective +
                                                      lon_jerk_cost * FLAGS_weight_lon_jerk +
                                                      std::fabs(collision_cost) * FLAGS_weight_lon_collision +
                                                      centripetal_acc_cost * FLAGS_weight_centripetal_acceleration),
                              result);
    }


    std::pair<double,Result> TrajectoryEvaluator::EvaluateEmergency (const PtrTrajectory1d &lon_trajectory) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            return std::make_pair(FLAGS_weight_lon_collision * collision_cost, Result::LON_STATIC_COLLISION);
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
            return std::make_pair(FLAGS_weight_lon_collision * collision_cost, Result::LON_DYNAMIC_COLLISION);
        }

        // Longitudinal costs
        double lon_objective_cost = 0.0;

        double lon_jerk_cost = LonComfortCost(lon_trajectory);

        double centripetal_acc_cost = 0.0;

        return std::make_pair(lon_objective_cost * FLAGS_weight_lon_objective +
                              lon_jerk_cost * FLAGS_weight_lon_jerk +
                              centripetal_acc_cost * FLAGS_weight_centripetal_acceleration,
                              Result::VALID);
    }


    std::pair<std::pair<std::vector<double>,double>,Result> TrajectoryEvaluator::EvaluateDebug(
            const PtrTrajectory1d& lon_trajectory, std::stringstream &ss) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        Result result = Result::VALID;

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            result = Result::LON_STATIC_COLLISION;
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
            result = Result::LON_DYNAMIC_COLLISION;
        }

//        double lon_objective_cost =
//                LonObjectiveCostDebug(lon_trajectory, target_speed_, ss);
        double lon_objective_cost =
                LonObjectiveCost(lon_trajectory, target_speed_);

//        double lon_jerk_cost = LonComfortCost(lon_trajectory);
        double lon_jerk_cost = 0.0;

//        double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
        double centripetal_acc_cost = 0.0;


        std::vector<double> res_vec = {lon_objective_cost, lon_jerk_cost, collision_cost, centripetal_acc_cost};

        ss << lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()) - lon_trajectory->Evaluate(0, 0.0)
           << ";" << lon_trajectory->Evaluate(1, lon_trajectory->ParamLength()) << ";" << lon_objective_cost
           << ";" << collision_cost << std::endl;


        return std::make_pair(std::make_pair(res_vec, lon_objective_cost * FLAGS_weight_lon_objective +
                                                      lon_jerk_cost * FLAGS_weight_lon_jerk +
                                                      std::fabs(collision_cost) * FLAGS_weight_lon_collision +
                                                      centripetal_acc_cost * FLAGS_weight_centripetal_acceleration),
                                result);

    }

    std::pair<std::pair<std::vector<double>,double>,Result> TrajectoryEvaluator::Evaluate2(
            const PtrTrajectory1d& lon_trajectory) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        Result result = Result::VALID;

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            result = Result::LON_STATIC_COLLISION;
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
             result = Result::LON_DYNAMIC_COLLISION;
        }

        double lon_objective_cost =
                LonObjectiveCost(lon_trajectory, target_speed_);

//        double lon_jerk_cost = LonComfortCost(lon_trajectory);
        double lon_jerk_cost = 0.0;

//        double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
        double centripetal_acc_cost = 0.0;


        std::vector<double> res_vec = {lon_objective_cost, lon_jerk_cost, collision_cost, centripetal_acc_cost};


        return std::make_pair(std::make_pair(res_vec, lon_objective_cost * FLAGS_weight_lon_objective +
                                                      lon_jerk_cost * FLAGS_weight_lon_jerk +
                                                      std::fabs(collision_cost) * FLAGS_weight_lon_collision +
                                                      centripetal_acc_cost * FLAGS_weight_centripetal_acceleration),
                              result);

    }

    std::pair<double,Result> TrajectoryEvaluator::Evaluate(
            const PtrTrajectory1d& lon_trajectory) const {
        // Costs:
        // 1. Cost of collision on static obstacles.
        // 2. Cost of missing the objective, e.g., cruise, stop, etc.
        // 3. Cost of logitudinal jerk
        // 4. Cost of logitudinal collision

        // Cost of collision on  obstacle
        double collision_cost_signed = LonCollisionCost(lon_trajectory);
        double collision_cost = std::fabs(collision_cost_signed);
        if(collision_cost_signed < -std::numeric_limits<double>::epsilon()){
            return std::make_pair(FLAGS_weight_lon_collision * collision_cost, Result::LON_STATIC_COLLISION);
        } else if(collision_cost_signed > std::numeric_limits<double>::epsilon()){
            return std::make_pair(FLAGS_weight_lon_collision * collision_cost, Result::LON_DYNAMIC_COLLISION);
        }

        // Longitudinal costs
        double lon_objective_cost = LonObjectiveCost(lon_trajectory, target_speed_);

//        double lon_jerk_cost = LonComfortCost(lon_trajectory);
        double lon_jerk_cost = 0.0;

//        double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
        double centripetal_acc_cost = 0.0;

        return std::make_pair(lon_objective_cost * FLAGS_weight_lon_objective +
                              lon_jerk_cost * FLAGS_weight_lon_jerk +
                              centripetal_acc_cost * FLAGS_weight_centripetal_acceleration,
                              Result::VALID);
    }


    double TrajectoryEvaluator::LonComfortCost(
            const PtrTrajectory1d& lon_trajectory) const {
        double cost_sqr_sum = 0.0;
        double cost_abs_sum = 0.0;
        for (double t = 0.0; t < FLAGS_trajectory_time_length;
             t += FLAGS_trajectory_time_resolution) {
            double jerk = lon_trajectory->Evaluate(3, t);
            double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
            cost_sqr_sum += cost * cost;
            cost_abs_sum += std::fabs(cost);
        }
        return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
    }



    double TrajectoryEvaluator::LonObjectiveCost(
            const PtrTrajectory1d& lon_trajectory,
            const double target_speed ) const {
        double t_max = lon_trajectory->ParamLength();
        double dist_s =
                lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

        double end_v = lon_trajectory->Evaluate(1, t_max);
        double delta_speed = std::fabs(target_speed-end_v);
        double delta_dist = std::max(0.0, 100 - dist_s);


        double speed_cost = logistic(delta_speed, 10);
        double dist_travelled_cost = logistic(delta_dist, 100);
        return (speed_cost * FLAGS_weight_target_speed +
                dist_travelled_cost * FLAGS_weight_dist_travelled) /
               (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
    }

    double TrajectoryEvaluator::LonStaticCollisionCost(
            const PtrTrajectory1d& lon_trajectory) const {

    }

// TODO(all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
    double TrajectoryEvaluator::LonDynamicCollisionCost(
            const PtrTrajectory1d& lon_trajectory) const {

        double s0 = lon_trajectory->Evaluate(0, 0.0);
        double last_s = -FLAGS_lattice_epsilon;
        double t_param = 0.0;
        for (double t = 0.0; t < FLAGS_trajectory_time_length;
             t += FLAGS_trajectory_time_resolution) {

            double s = lon_trajectory->Evaluate(0, t);
            if (last_s > 0.0) {
                s = std::max(last_s, s);
            }
            last_s = s;

            double s_param = s - s0;
            // linear extrapolation is handled internally in LatticeTrajectory1d;
            // no worry about s_param > lat_trajectory.ParamLength() situation
            double d = 0.0;

            double object_s, object_d, delta_s, delta_d;
            for(const auto &object : dynamic_objects_){
                object_s = (object.S)[0] + (object.S)[1]*t;
                object_d = (object.D)[0] + (object.D)[1]*t;
                delta_s = object_s - s;
                delta_d = object_d - d;
                if((delta_s)*(delta_s)+(delta_d)*(delta_d) <
                            object.half_length*object.half_length + object.half_width*object.half_width
                            + FLAGS_radius_safe_dis*FLAGS_radius_safe_dis){
                    return 1.0;
                }
            }
        }
        return 0.0;
    }


    double TrajectoryEvaluator::LonCollisionCost(
            const PtrTrajectory1d& lon_trajectory) const {

        double static_obstatic_s = dis_to_obstacle_ + init_s_[0] - FLAGS_head_offset;
        double s0 = lon_trajectory->Evaluate(0, 0.0);
        double last_s = -FLAGS_lattice_epsilon;
        double t_param = 0.0;

        double min_delta_s = std::numeric_limits<double>::infinity();
        double min_delta_d = std::numeric_limits<double>::infinity();
        double min_delta_sqrt = std::numeric_limits<double>::infinity();
        // origin method, influenced by resolution
        for (double t = 0.0; t < FLAGS_trajectory_time_length;
             t += FLAGS_trajectory_time_resolution) {

            double s = lon_trajectory->Evaluate(0, t);
            if (last_s > 0.0) {
                s = std::max(last_s, s);
            }
            last_s = s;

            if(s >= static_obstatic_s){
                return -1.0;
            }

            // linear extrapolation is handled internally in LatticeTrajectory1d;
            // no worry about s_param > lat_trajectory.ParamLength() situation
            double d = FLAGS_car_default_d;

            double object_s, object_d, delta_s, delta_d;
            for(const auto &object : dynamic_objects_){
                object_s = (object.S)[0] + (object.S)[1]*(t+delta_t_);
                object_d = (object.D)[0] + (object.D)[1]*(t+delta_t_);
                delta_s = object_s - s;
                delta_d = object_d - d;
                double sqrt = delta_d*delta_d+delta_s*delta_s;
                if(sqrt < min_delta_sqrt){
                    min_delta_sqrt = sqrt;
                    min_delta_d = delta_d;
                    min_delta_s = delta_s;
                }
                if(std::fabs(delta_s) < (FLAGS_half_car_length + object.half_length) &&
                        std::fabs(delta_d) < (FLAGS_half_car_width + object.half_width)) {
                    if((object.S[0] + object.S[1]*delta_t_) < init_s_[0]){
                        continue;
                    }
//                    AERROR << "Collision: " << "dis_s: " << lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()) - lon_trajectory->Evaluate(0, 0.0)
//                           << ", vel: " << lon_trajectory->Evaluate(1, lon_trajectory->ParamLength()) << ", current_t: " << t
//                           << ", delta_s: " << delta_s << ", delta_d: " << delta_d << ", our s: " << init_s_[0]
//                           << ", object_s: " << object.S[0] << ", object_d: " << object.D[0];
                    return 1.0;
                }

//
//                if((delta_s)*(delta_s)+(delta_d)*(delta_d) <
//                   object.half_length*object.half_length + object.half_width*object.half_width
//                   + FLAGS_radius_safe_dis*FLAGS_radius_safe_dis){
//                    return 1.0;
//                }
            }
        }
//        AERROR << "No collision: " << "dis_s: " << lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()) - lon_trajectory->Evaluate(0, 0.0)
//               << ", vel: " << lon_trajectory->Evaluate(1, lon_trajectory->ParamLength()) << ", min_s: " << min_delta_s
//               << ", min_d: " << min_delta_d ;

//        // my new method
//        int cnt = 0;
//        for(const auto &object : dynamic_objects_){
//            double dis_s = object.S[0] + object.S[1]*delta_t_ - (init_s_[0] + FLAGS_origin_to_center);
//            double fix_dis_s = dis_s - std::copysign(FLAGS_half_car_length + object.half_length, dis_s);
//            double collsion_t;
//            if(std::signbit(fix_dis_s) != std::signbit(dis_s)){
//                collsion_t = 0.0;
//            } else {
//                collsion_t = fix_dis_s/(init_s_[1] - object.S[1]);
//            }
//
//            double temp_offset = std::fabs(( object.D )[0] + ( object.D )[1] * ( collsion_t + delta_t_ ) - FLAGS_car_default_d);
//            if(collsion_t > -std::numeric_limits<double>::epsilon() && collsion_t <= 20.0 &&
//                    temp_offset < (FLAGS_half_car_width + object.half_width)) {
//                AERROR << cnt << ">> collision_t: " << collsion_t << ", offset: "
//                       << temp_offset << ", fix_dis: " << fix_dis_s << ", Ve: " << init_s_[1] << ", Vo: " << object.S[1];
//            }
//            cnt++;
//            if(collsion_t > -std::numeric_limits<double>::epsilon() && collsion_t <= FLAGS_trajectory_time_length){
//                double object_d = (object.D)[0] + (object.D)[1]*(collsion_t+delta_t_);
//                if(std::fabs(object_d - FLAGS_car_default_d) < (FLAGS_half_car_width + object.half_width)){
//                    return 1.5;
//                }
//            }
//        }

        return 0.0;
    }




//    double TrajectoryEvaluator::CentripetalAccelerationCost(
//            const PtrTrajectory1d& lon_trajectory) const {
//
//        // Assumes the vehicle is not obviously deviate from the reference line.
//        double centripetal_acc_sum = 0.0;
//        double centripetal_acc_sqr_sum = 0.0;
//        for (double t = 0.0; t < FLAGS_trajectory_time_length;
//             t += FLAGS_trajectory_time_resolution) {
//            double s = lon_trajectory->Evaluate(0, t);
//            double v = lon_trajectory->Evaluate(1, t);
//            CurvePoint ref_point = PathMatcher::MatchToPath(*reference_line_, s);
////            CHECK(ref_point.has_kappa());
//            double centripetal_acc = v * v * ref_point.kappa;
//            centripetal_acc_sum += std::fabs(centripetal_acc);
//            centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
//        }
//
//        return centripetal_acc_sqr_sum /
//               (centripetal_acc_sum + FLAGS_lattice_epsilon);
//    }



namespace {
    inline bool fuzzy_within(const double v, const double lower, const double upper,
                             const double e = 1.0e-4) {
        return v > lower - e && v < upper + e;
    }
}

    bool ConstraintChecker1d::IsValidLongitudinalTrajectory(
            const PolynomialCurve1d& lon_trajectory) {
        double t = 0.0;
        while (t < lon_trajectory.ParamLength()) {
            double v = lon_trajectory.Evaluate(1, t);  // evalute_v
            if (!fuzzy_within(v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
                return false;
            }

            double a = lon_trajectory.Evaluate(2, t);  // evaluat_a
            if (!fuzzy_within(a, FLAGS_longitudinal_acceleration_lower_bound,
                              FLAGS_longitudinal_acceleration_upper_bound)) {
                return false;
            }

//            double j = lon_trajectory.Evaluate(3, t);
//            if (!fuzzy_within(j, FLAGS_longitudinal_jerk_lower_bound,
//                              FLAGS_longitudinal_jerk_upper_bound)) {
//                return false;
//            }
            t += FLAGS_trajectory_time_resolution;
        }
        return true;
    }



}  // namespace JMT
