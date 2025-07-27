#include <string>

#include "gz/common/Console.hh"
#include "gz/plugin/Register.hh"

#include "ropod_sim/HolonomicControl.h"

GZ_ADD_PLUGIN(
    ropod::HolonomicControl,
    gz::sim::System,
    ropod::HolonomicControl::ISystemConfigure,
    ropod::HolonomicControl::ISystemPreUpdate)

namespace ropod {

void HolonomicControl::Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& event_manager)
{
    model_ = gz::sim::Model(entity);

    if ( !model_.Valid(ecm) )
    {
        gzerr << "HolonomicControl plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
        return;
    }

    if ( sdf->HasElement("cmd_vel_topic") )
    {
        cmd_vel_topic_ = sdf->Get<std::string>("cmd_vel_topic");
    }
    gzmsg << "cmd_vel_topic: " << cmd_vel_topic_ << std::endl;

    if ( sdf->HasElement("cmd_vel_timeout") )
    {
        cmd_vel_timeout_ = sdf->Get<double>("cmd_vel_timeout");
    }
    gzmsg << "cmd_vel_timeout: " << cmd_vel_timeout_ << std::endl;
    cmd_vel_watchdog_duration_ = std::chrono::duration<double>(cmd_vel_timeout_);

    node_.Subscribe(cmd_vel_topic_, &HolonomicControl::cmdVelCb, this);

    link_ = gz::sim::Link(entity);
    const std::optional<std::string> link_name = link_.Name(ecm);
    link_.EnableVelocityChecks(ecm);

    initialized_ = true;
}

void HolonomicControl::PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm)
{
    if ( info.paused )
    {
        return;
    }

    const bool print = ( info.iterations % 1000 == 0 );

    if ( !initialized_ )
    {
        if ( print )
        {
            gzerr << "HolonomicControl plugin has not initialized properly." << std::endl;
        }
        return;
    }

    last_sim_update_time_ = info.simTime;

    std::lock_guard<std::mutex> guard(mutex_);

    std::optional<gz::math::Vector3d> linear_vel = link_.WorldLinearVelocity(ecm);
    std::optional<gz::math::Vector3d> angular_vel = link_.WorldAngularVelocity(ecm);
    if ( !linear_vel || !angular_vel )
    {
        gzwarn << "Could not get linear and/or angular vel of robot's link" << std::endl;
        return;
    }

    const std::chrono::duration<double> duration_since_last_cmd_vel =
        (info.simTime - last_cmd_vel_msg_time_);
    if ( duration_since_last_cmd_vel > cmd_vel_watchdog_duration_ )
    {
        cmd_vel_x_ = 0.0;
        cmd_vel_y_ = 0.0;
        cmd_vel_theta_ = 0.0;
    }

    gz::math::Vector3d vel(cmd_vel_x_, cmd_vel_y_, linear_vel.value().Z());
    gz::math::Vector3d ang_vel(angular_vel.value().X(), angular_vel.value().Y(), cmd_vel_theta_);
    link_.SetLinearVelocity(ecm, vel);
    link_.SetAngularVelocity(ecm, ang_vel);
}

void HolonomicControl::cmdVelCb(const gz::msgs::Twist& msg)
{
    std::lock_guard<std::mutex> guard(mutex_);
    cmd_vel_x_ = msg.linear().x();
    cmd_vel_y_ = msg.linear().y();
    cmd_vel_theta_ = msg.angular().z();
    last_cmd_vel_msg_time_ = last_sim_update_time_;
}

}  // namespace ropod
