#pragma once

#include "gz/sim/System.hh"
#include "gz/sim/Link.hh"

#include "gz/msgs/twist.pb.h"
#include "gz/transport/Node.hh"

namespace ropod {

class HolonomicControl:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
    public:

        void Configure(
                const gz::sim::Entity& entity,
                const std::shared_ptr<const sdf::Element>& sdf,
                gz::sim::EntityComponentManager& ecm,
                gz::sim::EventManager& event_manager) override;

        void PreUpdate(
                const gz::sim::UpdateInfo& info,
                gz::sim::EntityComponentManager& ecm) override;

    protected:

        bool initialized_{false};

        /* gazebo related variables */
        gz::sim::Model model_;
        gz::sim::Link link_;
        std::chrono::steady_clock::duration last_sim_update_time_{0};
        gz::transport::Node node_;

        /* configurable parameters */
        std::string cmd_vel_topic_{"/cmd_vel"};
        double cmd_vel_timeout_{0.2}; // seconds

        /* commanded velocity related */
        std::mutex mutex_;
        double cmd_vel_x_{0.0};
        double cmd_vel_y_{0.0};
        double cmd_vel_theta_{0.0};
        std::chrono::steady_clock::duration last_cmd_vel_msg_time_{0};
        std::chrono::duration<double> cmd_vel_watchdog_duration_;

        void cmdVelCb(const gz::msgs::Twist& msg);

};

} // namespace ropod
