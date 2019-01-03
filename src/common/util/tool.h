//
// Created by luyifan on 18-7-10.
//

#ifndef STATEMACHINE_TOOL_H
#define STATEMACHINE_TOOL_H

#include "tiggo_msgs/LocalTrajPoint.h"
#include "tiggo_msgs/Object.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "common/curve/bezier.h"

namespace planner{
#define MY_INF std::numeric_limits<double>::infinity()
#define MY_PI 3.141592658
    /*计算两点之间的距离*/
    inline double dist2Points(tiggo_msgs::LocalTrajPoint p1, tiggo_msgs::LocalTrajPoint p2){
        return sqrt(pow(p1.position.x - p2.position.x, 2.0) + pow(p1.position.y - p2.position.y, 2.0));
    }

    inline double dist2Points(tiggo_msgs::LocalTrajPoint p1, geometry_msgs::PoseStamped p2){
        return sqrt(pow(p1.position.x - p2.pose.position.x, 2.0) + pow(p1.position.y - p2.pose.position.y, 2.0));
    }

    inline double dist2Points(Bezier::Vec2 pt1, Bezier::Vec2 pt2){
        return sqrt(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0));
    }

    inline double dist2Points(Bezier::Vec2 pt1,  tiggo_msgs::LocalTrajPoint pt2){
        return sqrt(pow(pt1.x - pt2.position.x, 2.0) + pow(pt1.y - pt2.position.y, 2.0));
    }

    inline double dist2Points(Bezier::Vec2 pt1,  geometry_msgs::PoseStamped pt2){
        return sqrt(pow(pt1.x - pt2.pose.position.x, 2.0) + pow(pt1.y - pt2.pose.position.y, 2.0));
    }

    inline double dist2Points(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2){
        return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2.0) + pow(p1.pose.position.y - p2.pose.position.y, 2.0));
    }

    inline double dist2Points(tiggo_msgs::Object p1, geometry_msgs::PointStamped p2){
        return sqrt(pow(p1.position.x - p2.point.x, 2.0) + pow(p1.position.y - p2.point.y, 2.0));
    }

    inline double dist2Points(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2){
        return sqrt(pow(p1.point.x - p2.point.x, 2.0) + pow(p1.point.y - p2.point.y, 2.0));
    }

    inline double dist2Points(geometry_msgs::PoseStamped p1, geometry_msgs::Point p2){
        return sqrt(pow(p1.pose.position.x - p2.x, 2.0) + pow(p1.pose.position.y - p2.y, 2.0));
    }
}
#endif //STATEMACHINE_TOOL_H
