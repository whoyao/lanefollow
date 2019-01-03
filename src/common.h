//
// Created by cybernp on 18-1-28.
//

#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#include <map>
#include "log.h"

#define NUM_WAYPOINTS_BEHIND        (5)
#define NUM_WAYPOINTS_AHEAD         (5)
#define TRACK_LENGTH                (6945.554)
#define SMOOTH_POINTS_DIST          (0.2)

#define DISPLAY_PATH_POINTS         (150)
#define PREVIOUS_POINTS_TO_KEEP     (50)

#define INTERVAL                    (0.02)
#define MAX_SPEED                   (45/2.23694)
#define LIMIT_SPEED                 (46/2.23694)
#define MAX_ACCELERATION            (3.0)
#define LIMIT_JERK                  (8.0)
#define LIMIT_ACC                   (5.5)
#define SAFT_DISTANCE               (25.0)
#define CHANGE_DISTANCE             (75.0)
#define COLLISION_DISTANCE          (12.0)

#define PERTURB_NUMBER              (5)
#define SIGMA_S                     (1.0)
#define SIGMA_S_DOT                 (0.5)
#define SIGMA_S_DDOT                (0.2)
#define SIGMA_D                     (0.25)
#define SIGMA_D_DOT                 (0.1)
#define SIGMA_D_DDOT                (0.0)

#define VEHICLE_RADIUS              (2.0)
#define COLLISION_WEIGHT            (9999)
#define BUFFER_WEIGHT               (10)
#define IN_LANE_BUFFER_WEIGHT       (10)
#define EFFICIENCY_WEIGHT           (800)
#define NOT_MIDDLE_WEIGHT           (100)
#define ACC_JERK_WEIGHT             (5000)
#define LIMIT_SPEED_WEIGHT          (7500)

enum STATE{
    START,
    KEEP_LANE,
    LEFT_CHANGE_LANE,
    RIGHT_CHANGE_LANE,
    PREPARE_LEFT_CHANGE_LANE,
    PREPARE_RIGHT_CHANGE_LANE
};

enum LANE{
    LANE1,
    LANE2,
    LANE3,
    LANE_NUM
};

#endif //PATH_PLANNING_COMMON_H
