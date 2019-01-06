//
// Created by huyao on 18-6-13.
// Email: hooyao@sjtu.edu.cn
//
#include <common/struct/traj_structs.h>

#include <cmath>

#include "common/math/math_utils.h"

double CurvePoint::DistanceTo(const CurvePoint &other) const{
    return std::hypot(x - other.x, y - other.y);
}

double CurvePoint::DistanceSquareTo(const CurvePoint &other) const{
    const double dx = x - other.x;
    const double dy = y - other.y;
    return dx * dx + dy * dy;
}

double CurvePoint::ThetaTo(const CurvePoint &other) const {
    return cyber_common::math::NormalizeAngle(theta - other.theta);
}


double InterpolateTheta(const double theta0, const double theta1, const double weight) {
    const double a0_n = cyber_common::math::NormalizeAngle(theta0);
    const double a1_n = cyber_common::math::NormalizeAngle(theta1);
    double d = a1_n - a0_n;
    if (d > M_PI) {
        d = d - 2 * M_PI;
    } else if (d < -M_PI) {
        d = d + 2 * M_PI;
    }

    const double a = a0_n + d * weight;
    return cyber_common::math::NormalizeAngle(a);
}


CurvePoint InterpolateUsingLinearApproximation( const CurvePoint &p0,
                                                    const CurvePoint &p1,
                                                    const double s){
    double s0 = p0.s;
    double s1 = p1.s;

    if( s0 > s1){
        return InterpolateUsingLinearApproximation(p1, p0, s);
    }

    CurvePoint cp;
    double weight;
    if( s1 - s0 < std::numeric_limits<double>::epsilon()){
        weight = 1.0;
    }
    else {
        weight = ( s - s0 ) / ( s1 - s0 );
    }
    cp.x = (1 - weight) * p0.x + weight * p1.x;
    cp.y = (1 - weight) * p0.y + weight * p1.y;
    cp.theta = InterpolateTheta(p0.theta, p1.theta, weight);
    cp.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    cp.kappa_prime = (1 - weight) * p0.kappa_prime + weight * p1.kappa_prime;
    cp.s = s;
    return cp;
}

bool CompareCurvePointByS(const CurvePoint& lit, const CurvePoint& rit)
{
    if(lit.s<rit.s){
        return true;
    }else{
        return false;
    }
}
