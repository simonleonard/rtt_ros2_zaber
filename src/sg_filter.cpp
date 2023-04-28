#include "rtt_ros2_zaber/sg_filter.hpp"

#include "Eigen/Core"

int int_power(int x, unsigned int p) {
    if (p == 0) return 1;
    if (p == 1) return x;

    int tmp = int_power(x, p / 2);
    if (p % 2 == 0)
        return tmp * tmp;
    else
        return x * tmp * tmp;
}

SavitzkyGolayFilter::SavitzkyGolayFilter(int l, int r, int o)
    : left_(l), right_(r), order_(o), window_size_(l + r + 1) {
    Eigen::MatrixXd H(window_size_, order_ + 1);
    for (int i = 0; i < window_size_; ++i) {
        for (int j = 0; j < order_ + 1; ++j) {
            H(i, j) = int_power(i - l - 1, j - 1);
        }
    }
    B_ = H * (H.transpose() * H).inverse() * H.transpose();
    weights_ = B_.row(l);
}

std::vector<double> SavitzkyGolayFilter::filter(
    const std::vector<double>& in) const {
    const size_t n = in.size();
    Eigen::VectorXd in_v =
        Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(in.data(), n);
    Eigen::VectorXd out_v(n);

    out_v.head(left_) = B_.topRows(left_) * in_v.head(window_size_);
    for (int i = left_; i < n - right_; ++i) {
        out_v(i) = weights_.dot(in_v.segment(i - left_, window_size_));
    }
    out_v.tail(right_) = B_.bottomRows(right_) * in_v.tail(window_size_);
    return {out_v.data(), out_v.data() + n};
}
double SavitzkyGolayFilter::filter_last_one(
    const std::vector<double>& in) const {
    Eigen::VectorXd in_v = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        in.data() + in.size() - window_size_, window_size_);
    return weights_.dot(in_v);
}