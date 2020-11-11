#include "qp_spline_speed_optimizer/polynomial_xd.h"

namespace TiEV {

PolynomialXd::PolynomialXd(const std::uint32_t order) : params_(order + 1, 0.0) {}

PolynomialXd::PolynomialXd(const std::vector<double>& params) : params_(params) {}

std::uint32_t PolynomialXd::order() const {
    return static_cast<std::uint32_t>(params_.size()) - 1;
}

void PolynomialXd::SetParams(const std::vector<double>& params) {
    params_ = params;
}

const std::vector<double>& PolynomialXd::params() const {
    return params_;
}

PolynomialXd PolynomialXd::DerivedFrom(const PolynomialXd& base) {
    std::vector<double> params;
    if(base.order() <= 0) {
        params.clear();
    }
    else {
        params.resize(base.params().size() - 1);
        for(std::uint32_t i = 1; i < base.order() + 1; ++i) {
            params[i - 1] = base[i] * i;
        }
    }
    return PolynomialXd(params);
}

PolynomialXd PolynomialXd::IntegratedFrom(const PolynomialXd& base, const double intercept) {
    std::vector<double> params;
    params.resize(base.params().size() + 1);
    params[0] = intercept;
    for(std::uint32_t i = 0; i < base.params().size(); ++i) {
        params[i + 1] = base[i] / (i + 1);
    }
    return PolynomialXd(params);
}

double PolynomialXd::operator()(const double value) const {
    double result = 0.0;
    for(auto rit = params_.rbegin(); rit != params_.rend(); ++rit) {
        result *= value;
        result += (*rit);
    }
    return result;
}

double PolynomialXd::operator[](const std::uint32_t index) const {
    if(index >= params_.size()) {
        return 0.0;
    }
    else {
        return params_[index];
    }
}
}
