/**
 * @file polynomial_curve1d.h
 **/

#ifndef MODULES_PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
#define MODULES_PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_

#include <iostream>

namespace JMT {

class PolynomialCurve1d {
public:
    PolynomialCurve1d() = default;
    virtual ~PolynomialCurve1d() = default;

    virtual double Evaluate(std::uint32_t order,
                            double param) const = 0;

    virtual double ParamLength() const = 0;

protected:
    double param_ = 0.0;
};

}  // namespace JMT

#endif  // MODULES_PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
