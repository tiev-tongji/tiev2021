#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <Eigen/Dense>

#include "common.h"
#include "cluster.h"
#include "lane_model.h"

class Polynomial
{
public:
    Polynomial() : m_confidence(0.0f) {}
    Polynomial(Eigen::Vector3f param, float y_bias, float min_y) :
        m_param(param), m_y_bias(y_bias), m_min_y(min_y), m_confidence(0.0) {}
    template <class T> Polynomial(const cluster::Cluster_<T> &points, float y_bias){
        if(points.size() < 4){ m_confidence = 0.0f; return; }
        Eigen::MatrixXf A(points.size()+2, 3); Eigen::VectorXf b(points.size()+2);
        {// construct and solve svd problem
            int idx = 0;
            m_min_y = 0.f;
            m_y_bias = y_bias;
            for(auto point = points.begin(); point != points.end(); ++point, ++idx) {
                float y = point->y - m_y_bias;
                b(idx) = point->x;
                A(idx, 0) = y;
                A(idx, 1) = y * y;
                A(idx, 2) = 1.f;
                if (y < m_min_y) {m_min_y = y;}
            }
            {
                const float M1 = 1.e4f, M2 = 1.f; // M1 为一次项竖直约束，M2为线性度约束
                int idx = static_cast<int>(points.size());
                b(idx) = 0.f; A(idx, 0) = M1; A(idx, 1) = 0.f; A(idx, 2) = 0.f; ++idx;
                b(idx) = 0.f; A(idx, 0) = 0.f; A(idx, 1) = M2; A(idx, 2) = 0.f; ++idx;
            }
            {
                m_param = A.fullPivLu().solve(b);
                const float MAX_RELATIVE_ERROR = 10.0;
                float relative_error = (A * m_param - b).norm() / b.norm();
                m_confidence = (MAX_RELATIVE_ERROR - relative_error) / MAX_RELATIVE_ERROR;
            }
        }
    }
    cluster::Cluster_<float> resample(float min_y=0.f, float max_y=0.f, float step_y=1.f);
    float min_y()  const { return m_min_y; }
    void set_min_y(float _min_y) { m_min_y = _min_y; }
    float y_bias() const { return m_y_bias; }
    float get_x_bias() const { return m_param(2);}
    // interfacing Survived<Target>
    float distance_to(const Polynomial& other) { return (m_param - other.m_param).norm(); }
    int dim() const { return 3; }
    float get_confidence() const { return m_confidence; }
    cv::Mat full_state() const { 
        return (cv::Mat_<float>(6, 1) << m_param(0), m_param(1), m_param(2), 0.f, 0.f, 0.f);
    }
    cv::Mat state() const { 
        return (cv::Mat_<float>(3, 1) << m_param(0), m_param(1), m_param(2));
    }
    friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly) {
        os << poly.m_param.transpose();
        return os;
    }

private:
    Eigen::Vector3f m_param; // polynomial param: w0 * y + w1 * y^2 + w3 = x, where y = y - y_bias, and w0 = 0.
    float m_y_bias;
    float m_min_y;
    float m_confidence;
};

std::map<LaneModel::Line<float>*, Polynomial>
makeSharedPolynomial(const LaneModel::Road<float>& road,  float y_bias, float* confidence = nullptr, float M = 1.e1f);

#endif // POLYNOMIAL_H
