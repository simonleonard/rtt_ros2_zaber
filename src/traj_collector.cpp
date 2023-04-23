#include "rtt_ros2_zaber/traj_collector.hpp"

void TrajCollector::addPoint(const Eigen::Ref<Eigen::VectorXd>& inputs,
                             const Eigen::Ref<Eigen::VectorXd>& outputs) {
    tx_.push_back(inputs.x());
    ls_.push_back(inputs.y());
    tz_.push_back(inputs.z());

    x_.push_back(outputs.x());
    y_.push_back(outputs.y());
    z_.push_back(outputs.z());
}

void TrajCollector::clear() {
    tx_.clear();
    ls_.clear();
    tz_.clear();
    x_.clear();
    y_.clear();
    z_.clear();
}

std::unique_ptr<TrajCollectorIterator> TrajCollector::createrIterator() const {
    return std::make_unique<TrajCollectorIterator>(*this);
}

Eigen::Vector3d TrajCollectorIterator::current_inputs() const {
    return {traj_collector_.tx()[index_], traj_collector_.ls()[index_],
            traj_collector_.tz()[index_]};
}
Eigen::Vector3d TrajCollectorIterator::current_outputs() const {
    return {traj_collector_.tip_x()[index_], traj_collector_.tip_y()[index_],
            traj_collector_.tip_z()[index_]};
}

Eigen::Vector3d TrajCollectorIterator::last_inputs() const {
    return {traj_collector_.tx().back(), traj_collector_.ls().back(),
            traj_collector_.tz().back()};
}
Eigen::Vector3d TrajCollectorIterator::last_outputs() const {
    return {traj_collector_.tip_x().back(), traj_collector_.tip_y().back(),
            traj_collector_.tip_z().back()};
}