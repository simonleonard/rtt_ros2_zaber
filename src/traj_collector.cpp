#include "rtt_ros2_zaber/traj_collector.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
void TrajCollector::addPoint(const Eigen::Ref<Eigen::VectorXd>& inputs,
                             const Eigen::Ref<Eigen::VectorXd>& outputs,
                             long time) {
    tx_.push_back(inputs.x());
    ls_.push_back(inputs.y());
    tz_.push_back(inputs.z());

    x_.push_back(outputs.x());
    y_.push_back(outputs.y());
    z_.push_back(outputs.z());

    timestamps_.push_back(time * 1.0e-9);
}

void TrajCollector::clear() {
    tx_.clear();
    ls_.clear();
    tz_.clear();
    x_.clear();
    y_.clear();
    z_.clear();
    timestamps_.clear();
}

void TrajCollector::filter_tip_position_all(const SavitzkyGolayFilter& filter) {
    x_filtered_ = filter.filter(tip_x());
    y_filtered_ = filter.filter(tip_y());
    z_filtered_ = filter.filter(tip_z());
}

std::unique_ptr<TrajCollectorIterator> TrajCollector::createrIterator(
    bool filtered) const {
    return std::make_unique<TrajCollectorIterator>(*this, filtered);
}

bool TrajCollector::filter_tip_position_xz_last(
    const SavitzkyGolayFilter& filter) {
    y_filtered_.push_back(y_.back());

    if (x_.size() < filter.window_size()) {
        x_filtered_.push_back(x_.back());
        z_filtered_.push_back(z_.back());
        return false;
    }
    x_filtered_.push_back(filter.filter_last_one(x_));
    z_filtered_.push_back(filter.filter_last_one(z_));
    return true;
}

void TrajCollector::write_to_file(const std::string& file_path,
                                  bool added_filter_data) const {
    std::cout << "Write to " << file_path << std::endl;
    std::ofstream file(file_path);
    file << std::fixed << std::setprecision(3);

    if (added_filter_data) {
        for (int i = 0; i < size(); ++i) {
            file << tx_[i] << " " << ls_[i] << " " << tz_[i] << " " << x_[i]
                 << " " << y_[i] << " " << z_[i] << " " << x_filtered_[i] << " "
                 << y_filtered_[i] << " " << z_filtered_[i] << " "
                 << timestamps_[i] << "\n";
        }

    } else {
        for (int i = 0; i < size(); ++i) {
            file << tx_[i] << " " << ls_[i] << " " << tz_[i] << " " << x_[i]
                 << " " << y_[i] << " " << z_[i] << " " << timestamps_[i]
                 << "\n";
        }
    }

    file.close();
}

Eigen::Vector3d TrajCollectorIterator::current_inputs() const {
    return {traj_collector_.tx()[index_], traj_collector_.ls()[index_],
            traj_collector_.tz()[index_]};
}
Eigen::Vector3d TrajCollectorIterator::current_outputs() const {
    return {x_[index_], y_[index_], z_[index_]};
}

Eigen::Vector3d TrajCollectorIterator::last_inputs() const {
    return {traj_collector_.tx().back(), traj_collector_.ls().back(),
            traj_collector_.tz().back()};
}
Eigen::Vector3d TrajCollectorIterator::last_outputs() const {
    return {x_.back(), y_.back(), z_.back()};
}