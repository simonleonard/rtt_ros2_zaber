#pragma once

#include <memory>
#include <vector>

#include "Eigen/Dense"
#include "rtt_ros2_zaber/sg_filter.hpp"

class TrajCollectorIterator;

class TrajCollector {
   public:
    void addPoint(const Eigen::Ref<Eigen::VectorXd>& js,
                  const Eigen::Ref<Eigen::VectorXd>& tp,
                  long time /* ns */);

    size_t size() const { return tx_.size(); }

    const std::vector<double>& tx() const { return tx_; }
    const std::vector<double>& ls() const { return ls_; }
    const std::vector<double>& tz() const { return tz_; }

    const std::vector<double>& tip_x() const { return x_; }
    const std::vector<double>& tip_y() const { return y_; }
    const std::vector<double>& tip_z() const { return z_; }

    const std::vector<double>& tip_x_filtered() const { return x_filtered_; }
    const std::vector<double>& tip_y_filtered() const { return y_filtered_; }
    const std::vector<double>& tip_z_filtered() const { return z_filtered_; }

    void filter_tip_position_all(const SavitzkyGolayFilter& filter);
    bool filter_tip_position_last(const SavitzkyGolayFilter& filter);

    void clear();
    std::unique_ptr<TrajCollectorIterator> createrIterator(bool filtered) const;

    void write_to_file(const std::string& file_path,
                       bool added_filter_data) const;

   private:
    // Control inputs: joint states.
    std::vector<double> tx_;
    std::vector<double> ls_;
    std::vector<double> tz_;

    // Control outputs: tip position.
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> z_;

    std::vector<double> x_filtered_;
    std::vector<double> y_filtered_;
    std::vector<double> z_filtered_;

    std::vector<double> timestamps_;
};

class TrajCollectorIterator {
   public:
    explicit TrajCollectorIterator(const TrajCollector& traj_collector,
                                   bool filtered)
        : traj_collector_(traj_collector),
          x_(filtered ? traj_collector.tip_x_filtered()
                      : traj_collector.tip_x()),
          y_(filtered ? traj_collector.tip_y_filtered()
                      : traj_collector.tip_y()),
          z_(filtered ? traj_collector.tip_z_filtered()
                      : traj_collector.tip_z()),
          index_(0) {}
    void first() { index_ = 0; }
    void next() { index_++; }
    bool isDone() const { return index_ == traj_collector_.size(); }

    // Current joint states
    Eigen::Vector3d current_js() const;
    // Current tip position
    Eigen::Vector3d current_tp() const;

    Eigen::Vector3d last_js() const;
    Eigen::Vector3d last_tp() const;

   private:
    const TrajCollector& traj_collector_;
    const std::vector<double>& x_;
    const std::vector<double>& y_;
    const std::vector<double>& z_;

    int index_;
};