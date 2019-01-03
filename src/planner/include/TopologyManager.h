//
// Created by luyifan on 18-1-12.
// Rewrote by huyao on 19-1-2
//

#ifndef STATEMACHINE_TOPOLOGYMANAGER_H
#define STATEMACHINE_TOPOLOGYMANAGER_H

#include "ros/ros.h"
#include "common/util/log.h"
#include "common/math/vec2d.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/struct/traj_structs.h"
#include "common/util/tool.h"
#include "curveSmoother.h"
#include "JMT_gflags.h"

//文件读取
#define READ_FILE_OK	0	//正常读取文件
#define READ_FILE_NULL	1	//无文件或文件无法打开
#define READ_FILE_ERROR	2	//文件内容错误
//属性定义
//属性1
#define ATTRIBUTE1_START			0	//起点
#define ATTRIBUTE1_ENTRANCE			1	//交叉口入点
#define ATTRIBUTE1_EXIT				2	//交叉口出口
#define ATTRIBUTE1_COMMON			3	//普通路点
#define ATTRIBUTE1_PARK_ENTRANCE	4	//进入停车区
#define ATTRIBUTE1_PARK_EXIT		5	//驶出停车区
#define ATTRIBUTE1_PARK				6	//停车位位置
#define ATTRIBUTE1_END				7	//终点
#define ATTRIBUTE1_INTERPOLATION	8	//插值点

typedef enum file_status {
    READ_FILE_OK,
    READ_FILE_NULL,
    READ_FILE_ERROR
} FILE_STATUS;

namespace PathMatcher {

    CurvePoint MatchToPath(const std::vector<CurvePoint>& reference_line,
                           double x, double y);

    CurvePoint FindProjectionPoint(const CurvePoint& p0,
                                   const CurvePoint& p1,
                                   double x, double y);

    CurvePoint MatchToPath(const std::vector<CurvePoint>& reference_line,
                           double s);

} // namespace PathMatcher

class ReferenceLine{
public:
    ReferenceLine(const std::vector<CurvePoint> &sparse_points);
    ~ReferenceLine();

public:
    bool UpdateReferenceLine(const std::vector<CurvePoint> &sparse_points);
    CurvePoint MatchToPath(double x, double y) const;
    CurvePoint MatchToPath(double s) const;
    double GetEndS() const;

private:
    JMT::CurveSmoother smoother_;

    std::vector<CurvePoint> sparse_points_;
    std::vector<CurvePoint> reference_points_;
};


class TopologyManager{
public:
    TopologyManager(std::string file_name);
    ~TopologyManager();

    FILE_STATUS get_status() {return status;}

    CurvePoint MatchToPath(double x, double y) const;
    CurvePoint MatchToPath(double s) const;
    double GetReferenceLineEndS() const;
    void UpdateCurrentPose(const CurvePoint& curve_point);

    double DistToNextIntersection();
    double DistToLastIntersection();
    double DistToEnd();

private:
    std::shared_ptr<ReferenceLine> reference_line_;
    CurvePoint current_pose_;
    FILE_STATUS status_;
};
#endif //STATEMACHINE_TOPOLOGYMANAGER_H
