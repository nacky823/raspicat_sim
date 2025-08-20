#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  class FollowActorPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      this->model_ = model;

      if (sdf->HasElement("actor_name"))
        actor_name_ = sdf->Get<std::string>("actor_name");
      if (sdf->HasElement("link_name"))
        link_name_ = sdf->Get<std::string>("link_name");
      if (sdf->HasElement("z_offset"))
        z_offset_ = sdf->Get<double>("z_offset");

      world_ = model_->GetWorld();
      link_  = model_->GetLink(link_name_);
      if (!link_)
      {
        gzerr << "[FollowActorPlugin] Link '" << link_name_ << "' not found\n";
        return;
      }

      // 毎ステップ呼ばれるアップデートイベント
      update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FollowActorPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (!world_ || !link_) return;

      // Actorは World 内では「Model」として扱われる（名前で取得）
      physics::ModelPtr actor = world_->ModelByName(actor_name_);
      if (!actor) return;

      ignition::math::Pose3d pose = actor->WorldPose();

      // yaw だけ採用：roll/pitchは常に 0（直立）
      const double yaw = pose.Rot().Yaw();
      ignition::math::Quaterniond q(0, 0, yaw);
      ignition::math::Pose3d lpose(pose.Pos(), q);

      // 必要なら高さを微調整（worldのcollision/visualで z=0.9 を入れているなら 0.0 のままがいい）
      lpose.Pos().Z() += z_offset_;

      // このモデル（円柱）のリンクをActor位置へ“ワープ”
      link_->SetWorldPose(lpose, true, true);
    }

  private:
    physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr  link_;
    std::string actor_name_ = "ped1";
    std::string link_name_  = "body";
    double z_offset_ = 0.0;
    event::ConnectionPtr update_conn_;
  };

  GZ_REGISTER_MODEL_PLUGIN(FollowActorPlugin)
}
