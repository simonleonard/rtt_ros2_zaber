#include "rtt_ros2_zaber/sg_filter.hpp"

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
    weights = B.row(l + 1);
}
Eigen::VectorXd SavitzkyGolayFilter::filter(
    const Eigen::Ref<Eigen::VectorXd>& in) {
    const int n = in.size();
    Eigen::VectorXd out(n);
    out.head(left) = B.topRows(left) * in.head(window_size);
    for (int i = left; i < n - right; ++i) {
        out(i) = weights.dot(in.segment(i, window_size));
    }
    out.tail(right) = B.bottomRows(right) * in.tail(window_size);

    return out;
}
double SavitzkyGolayFilter::filter_one(const Eigen::Ref<Eigen::VectorXd>& in) {
    return weights.dot(in);
}