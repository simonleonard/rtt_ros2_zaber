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
    : left(l), right(r), order(o), window_size(l + r + 1) {
    Eigen::MatrixXd H(window_size, order + 1);
    for (int i = 0; i < window_size; ++i) {
        for (int j = 0; j < order + 1; ++j) {
            H(i, j) = int_power(i - l - 1, j - 1);
        }
    }
    B = H * (H.transpose() * H).inverse() * H.transpose();
    weights = B.row(l);
}

std::vector<double> SavitzkyGolayFilter::filter(const std::vector<double>& in) {
    const size_t n = in.size();
    Eigen::VectorXd in_v =
        Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(in.data(), n);
    Eigen::VectorXd out_v(n);

    out_v.head(left) = B.topRows(left) * in_v.head(window_size);
    for (int i = left; i < n - right; ++i) {
        out_v(i) = weights.dot(in_v.segment(i - left, window_size));
    }
    out_v.tail(right) = B.bottomRows(right) * in_v.tail(window_size);
    return {out_v.data(), out_v.data() + n};
}
double SavitzkyGolayFilter::filter_last_one(const std::vector<double>& in) {
    Eigen::VectorXd in_v = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        in.data() + in.size() - window_size, window_size);
    return weights.dot(in_v);
}