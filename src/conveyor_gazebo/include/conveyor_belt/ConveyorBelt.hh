#ifndef CONVEYOR_BELT__CONVEYORBELT_HH_
#define CONVEYOR_BELT__CONVEYORBELT_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace conveyor_belt
{
/// \brief Conveyor belt system plugin for Gazebo Sim (Harmonic).
///
/// Applies a constant belt velocity to every dynamic model whose origin
/// rests within the belt volume, so cubes placed on the belt slide toward
/// the end. Ported in concept from rokokoo/gazebo-conveyor (Gazebo Classic
/// + ROS 1) to the modern gz-sim system-plugin API.
///
/// SDF parameters (all optional except <belt_link>):
///   <belt_link>   Name of the link that defines the belt surface/frame.
///   <velocity>    Belt speed in m/s along the belt link's local +X. [0.3]
///   <length>      Belt region length along local X (m).            [2.0]
///   <width>       Belt region width along local Y (m).             [0.5]
///   <height>      Region height above the belt top surface (m).    [0.3]
///   <power_topic> gz-transport topic (gz.msgs.Double, 0-100 %) to
///                 scale the belt speed at runtime.  [/model/<name>/conveyor/power]
class ConveyorBelt
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ConveyorBelt();
  ~ConveyorBelt() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  class Impl;
  std::unique_ptr<Impl> dataPtr;
};
}  // namespace conveyor_belt

#endif  // CONVEYOR_BELT__CONVEYORBELT_HH_
