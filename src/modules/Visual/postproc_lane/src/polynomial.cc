#include "polynomial.h"
using namespace Eigen;


cluster::Cluster_<float> Polynomial::resample(float min_y, float max_y, float step_y)
{
    cluster::Cluster_<float> cluster;

    min_y = std::min(m_min_y, min_y);
    int size = static_cast<int>(std::round((max_y - min_y)/step_y));
    MatrixXf A(size, 3);
    {
        float y = max_y;
        for(int idx=0; idx < size; y-=step_y, ++idx) {
            A(idx, 0) = y;
            A(idx, 1) = y * y;
            A(idx, 2) = 1.f;
        }
    }
    VectorXf x = A * m_param;
    {
        float y = max_y;
        for(int idx=0; idx < size; y-=step_y, ++idx) {
            cluster.push_back(cv::Point2f(x(idx), y + m_y_bias));
        }
    }
    return cluster;
}

std::map<LaneModel::Line<float>*, Polynomial>
makeSharedPolynomial(const LaneModel::Road<float>& road,  float y_bias, float* confidence, float M)
{
    std::map<LaneModel::Line<float>*, Polynomial> dict;
    auto lines = road.lines();

    float min_y = 0.f;
    size_t total_points_size = 0;
    const int total_line_count = static_cast<int>(lines.size());
    {// prepare
        bool any_valid_cluster = false;
        for(auto& line : lines){
            auto points = line->points();
            size_t points_size = points.size();
            if (points_size < 2) continue;
            if (points_size > 2) any_valid_cluster = true;
            total_points_size += points_size;
        }
        if (!any_valid_cluster){
            *confidence = 0.0f;
            return dict;
        }
    }

    MatrixXf A(total_points_size + 2 + static_cast<size_t>(total_line_count-1), total_line_count+2);
    VectorXf b(total_points_size + 2 + static_cast<size_t>(total_line_count-1));
    {// build problem
        int row = 0;
        long line_id = 0;
        for(auto line = lines.begin(); line != lines.end(); ++line, ++line_id){
            auto points = (*line)->points();
            if (!points.size()) continue;
            float last_x = points.begin()->x;
            const float DIFF_X_REWEIGHTED = 5.f;
            for(auto point = points.begin(); point != points.end();
                ++point, ++row) {
                float weight;
                {// update weight
                    weight = (DIFF_X_REWEIGHTED - fabs(last_x - point->x));
                    last_x = point->x;
                    //if(weight < 0.f) continue;
                }
                float y = point->y - y_bias;
                b(row) = point->x * weight; //!NOTE
                A(row, 0) = y * weight; A(row, 1) = y * y * weight;//!NOTE
                for(int j = 0; j < total_line_count; ++j){
                    A(row, j+2) = (line_id == j ? 1.f : 0.f) * weight;//!NOTE
                }
                if (y < min_y) {min_y = y;}
            }
        }
        {// regularization : small coefficient loss
            const float M1 = 1.e2f, M2 = 0.e3f;
            b(row) = 0.f; A.row(row).fill(0.f);
            A(row, 0) = M1 * total_points_size;
            ++row;
            b(row) = 0.f; A.row(row).fill(0.f);
            A(row, 1) = M2 * total_points_size;
            ++row;
        }
        {// regularization : lanes have fixed width 
            //const float M = 1.e1f;
            const float FIXED_WIDTH = 70.f * M;
            for (int line_id = 0; line_id < total_line_count-1; ++line_id, ++row) {
                b(row) = FIXED_WIDTH; A.row(row).fill(0.f);
                A(row, line_id + 2) = +M;
                A(row, line_id + 3) = -M;
            }
        }
    }
    VectorXf params(total_line_count+2);
    {// fitting
        params = A.fullPivLu().solve(b);
        //PRINT(params.transpose());
        float relative_error = (A * params - b).norm() / b.norm() / total_points_size * 1.e5f;
        PRINT(relative_error);
        if(confidence){
            const float MAX_ALLOW_ERR = 15.0f;
            if(relative_error > MAX_ALLOW_ERR) *confidence = 0;
            else *confidence = fabs(MAX_ALLOW_ERR - relative_error) / MAX_ALLOW_ERR;
        }
    }
    {// output
        long line_id = 0;
        for(auto line = lines.begin(); line != lines.end(); ++line, ++line_id){
            Vector3f param; param << params(0), params(1), params(line_id + 2); //!NOTE
            dict[*line] = Polynomial(param, y_bias, min_y);
        }
    }
    return dict;
}

