#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <sys/timeb.h>
#include "JMT_planner.h"

#define LOOKHEAD (30.0)


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi () { return M_PI; }

double deg2rad (double x) { return x * pi() / 180; }

double rad2deg (double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData (string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

///////ATTENTION: Since I calculate car acc from the interval time of each loop, run program before start simulation

int main () {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    TopologyManager topology_manager(map_file_);
    JMT_
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;


    // TODO: remove out, debug_flag, almostFinish, currentCircle, last_x_val, last_y_val
    h.onMessage(
            [ &map_waypoints_x, &map_waypoints_y, &map_waypoints_s] (
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {

                static std::vector<std::vector<double>> last_Frenet;
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                //auto sdata = string(data).substr(0, length);
                //cout << sdata << endl;
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                    auto s = hasData(data);

                    if (s != "") {
                        auto j = json::parse(s);

                        string event = j[0].get<string>();

                        if (event == "telemetry") {

                            // Main car's localization Data
                            double car_x = j[1]["x"];
                            double car_y = j[1]["y"];
                            double car_s_fake = j[1]["s"];
                            double car_d = j[1]["d"];
                            double car_yaw = j[1]["yaw"];
                            double car_speed = j[1]["speed"];

                            // Previous path data given to the Planner
                            auto previous_path_x = j[1]["previous_path_x"];
                            auto previous_path_y = j[1]["previous_path_y"];
                            // Previous path's end s and d values
                            double end_path_s = j[1]["end_path_s"];
                            double end_path_d = j[1]["end_path_d"];

                            // Sensor Fusion Data, a list of all other cars on the same side of the road.
                            auto sensor_fusion = j[1]["sensor_fusion"];

                            /************************************* HERE IS MY CODE ******************** ****************/
                            int num_waypoints = map_waypoints_x.size();
                            int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

                            tk::spline s1;
                            tk::spline s2;
                            vector<double> ptss;
                            vector<double> ptsx;
                            vector<double> ptsy;
                            vector<double> smooth_map_waypoints_x;
                            vector<double> smooth_map_waypoints_y;
                            vector<double> smooth_map_waypoints_s;

                            // windows for smooth current path
                            for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++) {
                                // for smooting, take so many previous and so many subsequent waypoints
                                int idx = (next_waypoint_index+i) % num_waypoints;
                                if (idx < 0) {
                                    // correct for wrap
                                    idx += num_waypoints;
                                }
                                // correct for wrap in s for spline interpolation (must be continuous)
                                double current_s = map_waypoints_s[idx];
                                double base_s = map_waypoints_s[next_waypoint_index];
                                // start of a new circle
                                if (i < 0 && current_s > base_s) {
                                    current_s -= TRACK_LENGTH;
                                    if(almostFinish){
                                        currentCircle += 1;
                                        update_flag = true;
                                    }
                                }
                                // end of current circle
                                if (i > 0 && current_s < base_s) {
                                    current_s += TRACK_LENGTH;
                                    almostFinish = true;
                                } else {
                                    almostFinish = false;
                                }

                                // keep points for spline smooth
                                ptss.push_back(current_s);
                                ptsx.push_back(map_waypoints_x[idx]);
                                ptsy.push_back(map_waypoints_y[idx]);
                            }

                            // smooth current path
                            s1.set_points(ptss, ptsx);
                            s2.set_points(ptss, ptsy);
                            auto iterx = map_waypoints_x.begin();
                            auto itery = map_waypoints_y.begin();
                            // do not smooth the path before current path, just push back the map points for getFrennet.
                            for (auto iters = map_waypoints_s.begin(); *iters < *ptss.begin(); iters++) {
                                smooth_map_waypoints_s.push_back(*iters);
                                smooth_map_waypoints_x.push_back(*(iterx++));
                                smooth_map_waypoints_y.push_back(*(itery++));
                            }
                            // smooth current path
                            for (double s = *ptss.begin(); s < ceil(*( ptss.end() - 1 )); s += SMOOTH_POINTS_DIST) {
                                smooth_map_waypoints_s.push_back(s);
                                smooth_map_waypoints_x.push_back(s1(s));
                                smooth_map_waypoints_y.push_back(s2(s));
                            }

                            vector<double> car_s_d = getFrenet(car_x, car_y, car_yaw, smooth_map_waypoints_x, smooth_map_waypoints_y);
                            double car_s = car_s_d[0];

                            INFO(car_s_fake);
                            // points was left
                            int prev_size = previous_path_x.size();
                            char tmp[128];
                            sprintf(tmp,"points need keep is %d, but total pass is %d.",PREVIOUS_POINTS_TO_KEEP, DISPLAY_PATH_POINTS - prev_size );
                            ERROR(tmp);
                            if(DISPLAY_PATH_POINTS - prev_size > PREVIOUS_POINTS_TO_KEEP) {
                                ERROR("point is not enough.");
                            }

                            // cuurent car pose in s direction
                            double car_pose_s[] = {car_s, car_speed / 2.23694, car_acc};
                            // current car pose in d direction
                            double car_pose_d[] = {car_d, car_speed_d, car_acc_d};
                            // make sure path is smooth in low speed, ignore d_dot and d_dotdot in low speed
                            if(ego_state.current_state == START){
                                car_pose_d[1] = 0.0;
                                car_pose_d[2] = 0.0;
                            }

                            static bool first = true;
                            if(last_x_val.size() > 0) {
                                for (int i = 0; i < DISPLAY_PATH_POINTS - prev_size; i++) {
                                    out << std::fixed << std::setprecision(10) << last_x_val[i] << "," << last_y_val[i] << endl;
                                    if(!first) {
                                        out2 << std::fixed << std::setprecision(10) << last_Frenet[i][0] << "," << last_Frenet[i][1] << endl;
                                    }
                                }
                                if(first){
                                    for (int i = 0; i < DISPLAY_PATH_POINTS - prev_size + PREVIOUS_POINTS_TO_KEEP; i++) {
                                        out2 << std::fixed << std::setprecision(10) << last_Frenet[i][0] << "," << last_Frenet[i][1] << endl;
                                    }
                                }
                                first = false;
                            }


                            // THE MOST IMPORTANT FUNCTION OF MY CODE, choose the best state and the best traj
                            const TG::path_with_cost_group &all_trajectory_Frenet = ego_state.get_all_trajectory_Frenet(
                                    car_s_fake, sensor_fusion, prev_size, car_pose_s, car_pose_d, update_flag);
/*
                            for(const auto & a_traj : all_trajectory_Frenet) {
                                const auto temp = std::get<1>(a_traj);
                                auto iter1 = temp[0].begin();
                                auto iter2 = temp[1].begin();
                                for(int i=0; i<temp[0].size();i++){
                                    std::cout << (iter1+i)->front() << "," << (iter2+i)->front() << std::endl;
                                }
                            }  */


                            TG::path_xy_with_cost_group after_tran;

                            for(const auto &one_trajectory_Frenet : all_trajectory_Frenet){
                                TG::points_xy_2d one_traj_xy;
                                const auto &one_traj_frenet = get<1>(one_trajectory_Frenet);

                                auto iterFs = one_traj_frenet[0].begin();
                                auto iterFd = one_traj_frenet[1].begin();
                                for(int i=0; i<one_traj_frenet[0].size(); i++){
                                     one_traj_xy.emplace_back( getXY( *((iterFs+i)->begin()), *((iterFd+i)->begin()),
                                                                               smooth_map_waypoints_s, smooth_map_waypoints_x,
                                                                               smooth_map_waypoints_y) );
                                }

                                after_tran.emplace_back(make_tuple(get<0>(one_trajectory_Frenet), one_traj_xy, get<2>(one_trajectory_Frenet), get<3>(one_trajectory_Frenet)));

                            }


                            TG::points_xy_2d next_XY = ego_state.choose_next_state(after_tran);

                            last_Frenet = next_XY;
                            // Define the actual (X,Y) points we will use for the planner
                            std::vector<double> next_x_vals;
                            std::vector<double> next_y_vals;

                            // keep 10 points in front of our car, make sure the path is smooth
                            for (int i = 0; i < min(prev_size, PREVIOUS_POINTS_TO_KEEP); i++) {
                                next_x_vals.emplace_back(previous_path_x[i]);
                                next_y_vals.emplace_back(previous_path_y[i]);
                            }

                            timepast = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now()-last_time);
                            cout << "time point 1 : " << timepast.count() << endl;
                            last_time = std::chrono::steady_clock::now();


                            //Fill up the rest of our path planner after filling it with previous points, here we will always output 150 points
                            for(const auto& point_xy_2d : next_XY){
                                next_x_vals.emplace_back(point_xy_2d[0]);
                                next_y_vals.emplace_back(point_xy_2d[1]);
                            }

//                            out2 << temp_goal_XY[0] + 1 << ',' << temp_goal_XY[1] << endl;
                            ERROR(next_XY.front().front());
                            ERROR(next_XY.front().back());

                            last_x_val = next_x_vals;
                            last_y_val = next_y_vals;

//                            auto iter1 = next_x_vals.begin();
//                            auto iter2 = next_y_vals.begin();
//                            for(int i=0;i<next_x_vals.size();i++){
//                                cout << *(iter1+i) << ";" << *(iter2+i) << endl;
//                            }

                            timepast = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now()-last_time);
                            cout << "time point 2 : " << timepast.count() << endl;
                            last_time = std::chrono::steady_clock::now();

                            /*********************************** END OF MY CODE ****************************************/

                            json msgJson;
                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            //this_thread::sleep_for(chrono::milliseconds(1000));
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        }
                    } else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
            });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([] (uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                        size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h] (uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h] (uWS::WebSocket<uWS::SERVER> ws, int code,
                            char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
