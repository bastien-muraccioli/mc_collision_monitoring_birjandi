#include "CollisionMonitoringBirjandi.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionMonitoringBirjandi::~CollisionMonitoringBirjandi() = default;

void CollisionMonitoringBirjandi::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionMonitoringBirjandi::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::reset called");
}

void CollisionMonitoringBirjandi::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::before");
}

void CollisionMonitoringBirjandi::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionMonitoringBirjandi::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionMonitoringBirjandi::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionMonitoringBirjandi", mc_plugin::CollisionMonitoringBirjandi)
