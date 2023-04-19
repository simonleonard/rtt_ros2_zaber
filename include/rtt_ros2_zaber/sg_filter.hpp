#include "Eigen/Dense"

class SavitzkyGolayFilter {
   public:
    SavitzkyGolayFilter(int l, int r, int o);
    Eigen::VectorXd filter(const Eigen::Ref<Eigen::VectorXd>& in);
    double filter_one(const Eigen::Ref<Eigen::VectorXd>& in);

   private:
    int left;
    int right;
    int order;
    int window_size;
    Eigen::MatrixXd B;
    Eigen::VectorXd weights;
};