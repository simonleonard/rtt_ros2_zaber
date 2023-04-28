#pragma once

#include <vector>

#include "Eigen/Dense"

class SavitzkyGolayFilter {
   public:
    SavitzkyGolayFilter(int l, int r, int o);
    int window_size() const { return window_size_; }
    std::vector<double> filter(const std::vector<double>& in) const;
    double filter_last_one(const std::vector<double>& in) const;

   private:
    int left_;
    int right_;
    int order_;
    int window_size_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd weights_;
};