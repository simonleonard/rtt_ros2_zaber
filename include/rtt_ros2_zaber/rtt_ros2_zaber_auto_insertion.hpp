#pragma once

#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"

class RttRos2ZaberAutoInsertion : public RttRos2ZaberBase {
   public:
    RttRos2ZaberAutoInsertion(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    void autoInsertion(const std::string& file);

   private:
    long insertion_start_time_;
    std::queue<Command> insert_cmds_;
};