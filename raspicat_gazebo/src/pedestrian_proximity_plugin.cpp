#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
/// @brief Actor plugin that stops walking when a target robot comes close and
///        resumes once the robot is far enough again.
class PedestrianProximityPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
    if (!this->actor_)
    {
      gzerr << "[PedestrianProximityPlugin] This plugin must be attached to an actor.\n";
      return;
    }

    this->world_ = this->actor_->GetWorld();
    if (!this->world_)
    {
      gzerr << "[PedestrianProximityPlugin] Failed to get world pointer.\n";
      return;
    }

    this->robot_name_ = _sdf->Get<std::string>("robot_model_name", "raspicat").first;
    this->walk_speed_ = _sdf->Get<double>("walk_speed", 1.0).first;
    this->stop_distance_ = _sdf->Get<double>("stop_distance", 1.0).first;
    this->resume_distance_ = _sdf->Get<double>("resume_distance", 1.5).first;

    if (this->resume_distance_ < this->stop_distance_)
    {
      gzerr << "[PedestrianProximityPlugin] resume_distance (" << this->resume_distance_
            << ") is less than stop_distance (" << this->stop_distance_
            << "). Clamping resume_distance to stop_distance.\n";
      this->resume_distance_ = this->stop_distance_;
    }

    this->script_time_ = this->actor_->ScriptTime();
    this->script_length_ = this->ComputeScriptLength(_model->GetSDF());
    this->last_update_time_ = this->world_->SimTime();
    this->last_pose_ = this->actor_->WorldPose();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianProximityPlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  /// @brief Walk through the actor SDF to find the largest waypoint time so we
  ///        can wrap the script time cleanly.
  double ComputeScriptLength(const sdf::ElementPtr &_actor_sdf) const
  {
    double length = 0.0;

    if (!_actor_sdf || !_actor_sdf->HasElement("script"))
      return length;

    sdf::ElementPtr script_elem = _actor_sdf->GetElement("script");
    for (auto traj_elem = script_elem->GetElement("trajectory"); traj_elem;
         traj_elem = traj_elem->GetNextElement("trajectory"))
    {
      if (!traj_elem->HasElement("waypoint"))
        continue;

      for (auto wp = traj_elem->GetElement("waypoint"); wp;
           wp = wp->GetNextElement("waypoint"))
      {
        if (wp->HasElement("time"))
          length = std::max(length, wp->Get<double>("time"));
      }
    }
    return length;
  }

  /// @brief Every simulation step: check distance and advance (or freeze)
  ///        the actor's script time.
  void OnUpdate(const common::UpdateInfo &_info)
  {
    if (!this->actor_ || !this->world_)
      return;

    const double dt = (_info.simTime - this->last_update_time_).Double();
    if (dt <= 0.0)
      return;

    this->last_update_time_ = _info.simTime;

    physics::ModelPtr robot = this->world_->ModelByName(this->robot_name_);
    if (robot)
    {
      const double distance =
        this->actor_->WorldPose().Pos().Distance(robot->WorldPose().Pos());

      // Hysteresis: stop when close, resume only after moving far enough.
      if (!this->stopped_ && distance <= this->stop_distance_)
      {
        this->stopped_ = true;
      }
      else if (this->stopped_ && distance >= this->resume_distance_)
      {
        this->stopped_ = false;
      }
    }

    // Transition logic: snapshot progress at stop time and re-apply it on resume.
    if (this->stopped_ && !this->was_stopped_)
    {
      this->script_time_ = this->actor_->ScriptTime();
      this->last_pose_ = this->actor_->WorldPose();
    }
    else if (!this->stopped_ && this->was_stopped_)
    {
      // Some gazebo versions reset the actor's internal timers when Stop/Play is used,
      // which pushes the script back to the first waypoint. Avoid using Stop/Play and
      // instead re-apply the saved script progress so playback continues from where
      // it was paused.
      this->actor_->SetScriptTime(this->script_time_);
      this->actor_->SetWorldPose(this->last_pose_);
    }

    if (this->stopped_)
    {
      // Hold pose/time steady so Gazebo does not advance the waypoint or animation.
      this->actor_->SetScriptTime(this->script_time_);
      this->actor_->SetWorldPose(this->last_pose_);
    }
    else
    {
      this->script_time_ += this->walk_speed_ * dt;
      if (this->script_length_ > 0.0)
        this->script_time_ = std::fmod(this->script_time_, this->script_length_);

      this->actor_->SetScriptTime(this->script_time_);
      this->last_pose_ = this->actor_->WorldPose();
    }
    this->was_stopped_ = this->stopped_;

    // Keep velocities zero; actors are kinematic and driven by script time or explicit pose.
    this->actor_->SetLinearVel(ignition::math::Vector3d::Zero);
    this->actor_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

private:
  physics::ActorPtr actor_;
  physics::WorldPtr world_;
  std::string robot_name_{"raspicat"};
  double walk_speed_{1.0};
  double stop_distance_{1.0};
  double resume_distance_{1.5};
  double script_time_{0.0};
  double script_length_{0.0};
  ignition::math::Pose3d last_pose_;
  bool stopped_{false};
  bool was_stopped_{false};
  common::Time last_update_time_;
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(PedestrianProximityPlugin)
}  // namespace gazebo
