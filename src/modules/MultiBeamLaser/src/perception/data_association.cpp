#include "data_association.hpp"
#include "successive_shortest_path.h"

DataAssociation::DataAssociation()
    : score_threshold_(1e-40)
{
}

bool DataAssociation::assign(const Eigen::MatrixXd &src,
                             std::unordered_map<int, int> &direct_assignment,
                             std::unordered_map<int, int> &reverse_assignment)
{
    std::vector<std::vector<double>> score(src.rows());
    for (int row = 0; row < src.rows(); ++row)
    {
        score.at(row).resize(src.cols());
        for (int col = 0; col < src.cols(); ++col)
        {
            score.at(row).at(col) = src(row, col);
        }
    }
    // Solve
    assignment_problem::MaximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

    for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();)
    {
        if (src(itr->first, itr->second) < score_threshold_)
        {
            itr = direct_assignment.erase(itr);
            continue;
        }
        else
        {
            ++itr;
        }
    }
    for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();)
    {
        if (src(itr->second, itr->first) < score_threshold_)
        {
            itr = reverse_assignment.erase(itr);
            continue;
        }
        else
        {
            ++itr;
        }
    }
}