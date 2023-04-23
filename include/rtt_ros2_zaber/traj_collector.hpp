#include <memory>
#include <vector>

#include "Eigen/Dense"

class TrajCollectorIterator;

class TrajCollector {
   public:
    void addPoint(const Eigen::Ref<Eigen::VectorXd>& inputs,
                  const Eigen::Ref<Eigen::VectorXd>& outputs);

    size_t size() const { return tx_.size(); }

    const std::vector<double>& tx() const { return tx_; }
    const std::vector<double>& ls() const { return ls_; }
    const std::vector<double>& tz() const { return tz_; }

    const std::vector<double>& tip_x() const { return x_; }
    const std::vector<double>& tip_y() const { return y_; }
    const std::vector<double>& tip_z() const { return z_; }

    std::vector<double>& tip_x() { return x_; }
    std::vector<double>& tip_y() { return y_; }
    std::vector<double>& tip_z() { return z_; }

    void clear();
    std::unique_ptr<TrajCollectorIterator> createrIterator() const;

   private:
    // Control inputs: joint states.
    std::vector<double> tx_;
    std::vector<double> ls_;
    std::vector<double> tz_;

    // Control outputs: tip position.
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;
};

class TrajCollectorIterator {
   public:
    explicit TrajCollectorIterator(const TrajCollector& traj_collector)
        : traj_collector_(traj_collector), index_(0) {}
    void first() { index_ = 0; }
    void next() { index_++; }
    bool isDone() const { return index_ == traj_collector_.size(); }

    Eigen::Vector3d current_inputs() const;
    Eigen::Vector3d current_outputs() const;

    Eigen::Vector3d last_inputs() const;
    Eigen::Vector3d last_outputs() const;
   private:
    const TrajCollector& traj_collector_;
    int index_;
};