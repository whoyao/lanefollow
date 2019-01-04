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
#include "common.h"
#include "common/util/log.h"


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

std::vector<DynamicObjectXY> get_dynamic_obstacle(std::vector<std::vector<double>> sensor_fusion){
    std::vector<DynamicObjectXY> res_vec;
    for (auto one : sensor_fusion) {
        DynamicObjectXY one_obstacle (one[1], one[2], std::sqrt(one[3]*one[3]+one[4]*one[4]), std::atan2(one[4], one[3]),
                                        2.0, 2.0);
        res_vec.push_back(std::move(one_obstacle));
    }
    return res_vec;

}

int MatchPointInCurvePointList(CurvePoint test_point, std::vector<CurvePoint> point_list){
    int index = 0;
    int min_index = index;
    double min_dis = std::numeric_limits<double>::infinity();
    for(auto point : point_list) {
        double temp_dis = test_point.DistanceSquareTo(point);
        if( temp_dis < min_dis ) {
            min_dis = temp_dis;
            min_index = index;
        }
        index ++;
    }
    return index;
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
    JMT::JMTPlanner planner(&topology_manager);
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;


    // TODO: remove out, debug_flag, almostFinish, currentCircle, last_x_val, last_y_val
    h.onMessage(
            [ &planner ] (
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {

                static std::vector<CurvePoint> last_path;
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

                            CurvePoint current_pose;
                            current_pose.x = car_x;
                            current_pose.y = car_y;
                            current_pose.theta = car_yaw;
                            current_pose.v = car_speed;

                            auto dynamic_obstacle = get_dynamic_obstacle(sensor_fusion);
                            planner.update(dynamic_obstacle, current_pose, std::numeric_limits<double>::infinity());

                            // points was left
                            int prev_size = previous_path_x.size();
                            char tmp[128];
                            AINFO << "points need keep is " << PREVIOUS_POINTS_TO_KEEP << ", but total pass is " << DISPLAY_PATH_POINTS - prev_size;
                            if(DISPLAY_PATH_POINTS - prev_size > PREVIOUS_POINTS_TO_KEEP) {
                                ERROR("point is not enough.");
                            }

                            int last_index = min(prev_size, PREVIOUS_POINTS_TO_KEEP);

                            CurvePoint planning_init_point;
                            if(last_index < 5 || last_path.empty()) {
                                planning_init_point = current_pose;
                            } else {
                                CurvePoint temp_point, planning_init_point;
                                double x = previous_path_x[last_index];
                                double y = previous_path_y[last_index];
                                temp_point.x = x;
                                temp_point.y = y;
                                int indx_on_last_path = MatchPointInCurvePointList(temp_point, last_path);
                                planning_init_point = last_path[indx_on_last_path];
                            }
                            auto new_path = planner.plan_new(planning_init_point, std::numeric_limits<double>::infinity(), 20);

/*
                            for(const auto & a_traj : all_trajectory_Frenet) {
                                const auto temp = std::get<1>(a_traj);
                                auto iter1 = temp[0].begin();
                                auto iter2 = temp[1].begin();
                                for(int i=0; i<temp[0].size();i++){
                                    std::cout << (iter1+i)->front() << "," << (iter2+i)->front() << std::endl;
                                }
                            }  */


                            // Define the actual (X,Y) points we will use for the planner
                            std::vector<double> next_x_vals;
                            std::vector<double> next_y_vals;

                            // keep 10 points in front of our car, make sure the path is smooth
                            for (int i = 0; i < last_index; i++) {
                                next_x_vals.emplace_back(previous_path_x[i]);
                                next_y_vals.emplace_back(previous_path_y[i]);
                            }


                            //Fill up the rest of our path planner after filling it with previous points, here we will always output 150 points
                            for(const auto& point_xy_2d :new_path ){
                                next_x_vals.emplace_back(point_xy_2d.x);
                                next_y_vals.emplace_back(point_xy_2d.y);
                            }

//                            out2 << temp_goal_XY[0] + 1 << ',' << temp_goal_XY[1] << endl;
//                            auto iter1 = next_x_vals.begin();
//                            auto iter2 = next_y_vals.begin();
//                            for(int i=0;i<next_x_vals.size();i++){
//                                cout << *(iter1+i) << ";" << *(iter2+i) << endl;
//                            }

                            last_path = new_pah;
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
