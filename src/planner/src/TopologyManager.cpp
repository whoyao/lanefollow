//
// Created by luyifan on 18-1-12.
//

#include "manager/TopologyManager.h"


CurvePoint PathMatcher::MatchToPath(const std::vector<CurvePoint>& reference_line,
                                    const double x, const double y) {
    CHECK_GT(reference_line.size(), 0);

    auto func_distance_square = [](const CurvePoint& point, const double x,
                                   const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return dx * dx + dy * dy;
    };

    double distance_min = func_distance_square(reference_line.front(), x, y);
    std::size_t index_min = 0;

    for (std::size_t i = 1; i < reference_line.size(); ++i) {
        double distance_temp = func_distance_square(reference_line[i], x, y);
        if (distance_temp < distance_min) {
            distance_min = distance_temp;
            index_min = i;
        }
    }

    std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
    std::size_t index_end =
            (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

    if (index_start == index_end) {
        return reference_line[index_start];
    }

    return FindProjectionPoint(reference_line[index_start],
                               reference_line[index_end], x, y);
}


CurvePoint PathMatcher::MatchToPath(const std::vector<CurvePoint>& reference_line,
                                    const double s) {
    auto comp = [](const CurvePoint& point, const double s) {
        return point.s < s;
    };

    auto it_lower =
            std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
    if (it_lower == reference_line.begin()) {
        return reference_line.front();
    } else if (it_lower == reference_line.end()) {
        return reference_line.back();
    }

    // interpolate between it_lower - 1 and it_lower
    // return interpolate(*(it_lower - 1), *it_lower, s);
    return InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
}


CurvePoint PathMatcher::FindProjectionPoint(const CurvePoint& p0,
                                            const CurvePoint& p1, const double x,
                                            const double y) {
    double v0x = x - p0.x;
    double v0y = y - p0.y;

    double v1x = p1.x - p0.x;
    double v1y = p1.y - p0.y;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    return InterpolateUsingLinearApproximation(p0, p1, p0.s + delta_s);
}


// 读取原始路径文件
FILESTATUS ReadFile(const std::string filename, std::vector<CurvePoint> &reference_points) {

    // 测试文件是否存在
    std::ifstream fin;
    fin.open(filename);
    if(!fin){
        return READ_FILE_NULL;
    }

    // 读取文件，存为vector
    reference_points.clear();
    std::string line;
    while (getline(fin, line))
    {
        if (line.length() > 1)
        {
            CurvePoint temp;
            std::vector<std::string> strs;
            boost::split(strs, line, [](char c){return c == ' ';});
            if(strs.size() < 2){
                return READ_FILE_ERROR;
            }
            temp.x = std::stod(strs[0]);
            temp.y = std::stod(strs[1]);
            TaskPoints.push_back(temp);
        }
    }
    fin.close();
    return READ_FILE_OK;
}


ReferenceLine::ReferenceLine (const std::vector<CurvePoint> &sparse_points) {
    if(!UpdateReferenceLine(sparse_points)) {
        AERROR << "Smootth reference line failed! ";
    }
}


bool ReferenceLine::UpdateReferenceLine(const std::vector<CurvePoint> &sparse_points) {
    sparse_points_ = sparse_points;
    return smoother_.clothoideSpline(sparse_points, sparse_points[0].theta,
                                   sparse_points[0].kappa, sparse_points[0].s,
                                   FLAGS_clothoide_n_lookahead, reference_points_);
}

CurvePoint ReferenceLine::MatchToPath (double x, double y) const{
    return PathMatcher::MatchToPath(reference_points_, x, y);
}

CurvePoint ReferenceLine::MatchToPath (double s) const{
    return PathMatcher::MatchToPath(reference_points_, s);
}

double ReferenceLine::GetEndS() const{
    return reference_points_.back().s;
}

TopologyManager::TopologyManager(std::string file_name){
    std::vector<CurvePoint> sparse_points;
    status_ = ReadFile(file_name, &sparse_points);
    reference_line_ = std::make_shared<ReferenceLine>(sparse_points);
}


CurvePoint TopologyManager::MatchToPath (double x, double y) const {
    return reference_line_.MatchToPath(x, y);
}

CurvePoint TopologyManager::MatchToPath (double s) const {
    return reference_line_.MatchToPath(s);
}

void TopologyManager::UpdateCurrentPose(const CurvePoint& curve_point) {
    current_pose_ = curve_point;
}

double TopologyManager::GetReferenceLineEndS() const{
    return reference_line_.GetEndS();
}


