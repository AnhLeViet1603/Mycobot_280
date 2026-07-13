#include "conveyor_belt/ConveyorBelt.hh"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>

using namespace conveyor_belt;

class ConveyorBelt::Impl
{
public:
  /// \brief The conveyor model this plugin is attached to.
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Link entity that defines the belt surface/frame.
  gz::sim::Entity beltLink{gz::sim::kNullEntity};

  /// \brief Nominal belt speed (m/s) along the belt link's local +X.
  double velocity{0.3};

  /// \brief Belt region half-extents (m) in the belt link frame.
  double halfLength{1.0};
  double halfWidth{0.25};
  double height{0.3};

  /// \brief Belt power fraction in [0, 1], settable at runtime.
  std::atomic<double> power{1.0};

  /// \brief gz-transport node + subscription for runtime power control.
  gz::transport::Node node;

  /// \brief Callback: power percentage in [0, 100].
  void OnPower(const gz::msgs::Double & _msg)
  {
    this->power = std::clamp(_msg.data() / 100.0, 0.0, 1.0);
  }
};

ConveyorBelt::ConveyorBelt()
: dataPtr(std::make_unique<Impl>())
{
}

ConveyorBelt::~ConveyorBelt() = default;

void ConveyorBelt::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "ConveyorBelt must be attached to a model. Plugin disabled.\n";
    return;
  }
  const std::string modelName = this->dataPtr->model.Name(_ecm);

  // Read parameters.
  const std::string beltLinkName =
    _sdf->Get<std::string>("belt_link", "belt").first;
  this->dataPtr->velocity = _sdf->Get<double>("velocity", 0.3).first;
  const double length = _sdf->Get<double>("length", 2.0).first;
  const double width = _sdf->Get<double>("width", 0.5).first;
  this->dataPtr->height = _sdf->Get<double>("height", 0.3).first;
  this->dataPtr->halfLength = 0.5 * length;
  this->dataPtr->halfWidth = 0.5 * width;

  this->dataPtr->beltLink = this->dataPtr->model.LinkByName(_ecm, beltLinkName);
  if (this->dataPtr->beltLink == gz::sim::kNullEntity) {
    gzerr << "ConveyorBelt: belt_link [" << beltLinkName
          << "] not found in model [" << modelName << "]. Plugin disabled.\n";
    return;
  }

  const std::string defaultTopic =
    "/model/" + modelName + "/conveyor/power";
  const std::string powerTopic =
    _sdf->Get<std::string>("power_topic", defaultTopic).first;
  this->dataPtr->node.Subscribe(powerTopic, &Impl::OnPower, this->dataPtr.get());

  gzmsg << "ConveyorBelt attached to [" << modelName << "], belt_link ["
        << beltLinkName << "], velocity " << this->dataPtr->velocity
        << " m/s, power topic [" << powerTopic << "].\n";
}

void ConveyorBelt::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  if (_info.paused || this->dataPtr->beltLink == gz::sim::kNullEntity) {
    return;
  }

  // Belt frame in world coordinates and its forward (+X) direction.
  const gz::math::Pose3d beltPose =
    gz::sim::worldPose(this->dataPtr->beltLink, _ecm);
  const gz::math::Vector3d forward =
    beltPose.Rot().RotateVector(gz::math::Vector3d::UnitX);
  const gz::math::Vector3d beltVel =
    forward * (this->dataPtr->velocity * this->dataPtr->power.load());

  const gz::sim::Entity conveyorEntity = this->dataPtr->model.Entity();

  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name,
            gz::sim::components::Pose>(
    [&](const gz::sim::Entity & _modelEntity,
        const gz::sim::components::Model *,
        const gz::sim::components::Name *,
        const gz::sim::components::Pose * _pose) -> bool
    {
      // Skip the conveyor itself and static models (ground plane, etc.).
      if (_modelEntity == conveyorEntity) {
        return true;
      }
      if (_ecm.EntityHasComponentType(
          _modelEntity, gz::sim::components::Static::typeId))
      {
        const auto * isStatic =
          _ecm.Component<gz::sim::components::Static>(_modelEntity);
        if (isStatic != nullptr && isStatic->Data()) {
          return true;
        }
      }
      // The model Pose component is relative to its parent; for top-level
      // models (cubes spawned into the world) that parent is the world, so
      // this is the world pose we need to test against the belt frame.
      // Transform the model origin into the belt frame.
      const gz::math::Vector3d local =
        beltPose.Inverse().CoordPositionAdd(_pose->Data().Pos());

      if (std::abs(local.X()) > this->dataPtr->halfLength ||
          std::abs(local.Y()) > this->dataPtr->halfWidth ||
          local.Z() < -0.05 || local.Z() > this->dataPtr->height)
      {
        return true;  // Not on the belt.
      }

      // Command the belt velocity on the model's canonical link.
      const gz::sim::Entity canonical =
        gz::sim::Model(_modelEntity).CanonicalLink(_ecm);
      if (canonical == gz::sim::kNullEntity) {
        return true;
      }
      auto * cmd =
        _ecm.Component<gz::sim::components::LinearVelocityCmd>(canonical);
      if (cmd == nullptr) {
        _ecm.CreateComponent(
          canonical, gz::sim::components::LinearVelocityCmd(beltVel));
      } else {
        cmd->Data() = beltVel;
      }
      return true;
    });
}

GZ_ADD_PLUGIN(
  conveyor_belt::ConveyorBelt,
  gz::sim::System,
  conveyor_belt::ConveyorBelt::ISystemConfigure,
  conveyor_belt::ConveyorBelt::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(conveyor_belt::ConveyorBelt, "conveyor_belt::ConveyorBelt")
