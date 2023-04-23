#include <vector>

#include "Eigen/Dense"

class SavitzkyGolayFilter {
   public:
    SavitzkyGolayFilter(int l, int r, int o);
    std::vector<double> filter(const std::vector<double>& in);
    double filter_last_one(const std::vector<double>& in);

   private:
    int left;
    int right;
    int order;
    int window_size;
    Eigen::MatrixXd B;
    Eigen::VectorXd weights;
};