//
// Created by huyao on 2018/6/8.
// Email: hooyao@sjtu.edu.cn
//

#ifndef PROJECT_TRAJ_STRUCTS_H
#define PROJECT_TRAJ_STRUCTS_H

#include <climits>
#include <cmath>
#include <vector>
#include <memory>

class CurvePoint {

public:
    CurvePoint() : s(0.0), x(0.0), y(0.0), theta(0.0), kappa(0.0), kappa_prime(0.0), v(0.0), a(0.0), t(0.0) {};
    virtual  ~CurvePoint() = default;

public:
    //! Returns the distance to the given vector
    double DistanceTo(const CurvePoint &other) const;

    //! Returns the squared distance to the given vector
    double DistanceSquareTo(const CurvePoint &other) const;

    //! Returns the difference of theta to the given vector
    double ThetaTo(const CurvePoint &other) const;

public:
    double s;
    double x;
    double y;
    double theta;
    double kappa;
    double kappa_prime;
    double v;
    double a;
    double t;
//    int index;
};

class DynamicObject{
public:
    DynamicObject(std::array<double, 3> s,
                   std::array<double, 3> d)
            : S(s), D(d){};

public:
    std::array<double, 3> S;
    std::array<double, 3> D;
    double half_length;
    double half_width;
};


class DynamicObjectXY{
public:
    DynamicObjectXY(double x, double y, double v, double theta, double half_length, double half_width)
            : x(x), y(y), v(v), theta(theta), half_length(half_length), half_width(half_width){};

public:
    double x;
    double y;
    double v;
    double theta;
    double half_length;
    double half_width;
};

//! Function used for STL sort algorithm
bool CompareCurvePointByS(const CurvePoint& lit, const CurvePoint& rit);

CurvePoint InterpolateUsingLinearApproximation( const CurvePoint &p0,
                                                    const CurvePoint &p1,
                                                    double s);

#endif //PROJECT_TRAJ_STRUCTS_H
