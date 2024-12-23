#ifndef DAVE_GZ_WORLD_PLUGINS__OCEAN_CURRENT_WORLD_PLUGIN_HH_
#define DAVE_GZ_WORLD_PLUGINS__OCEAN_CURRENT_WORLD_PLUGIN_HH_

#include <dave_gz_world_plugins/gauss_markov_process.hh>
#include <dave_gz_world_plugins/tidal_oscillation.hh>
#include <gz/sim/System.hh>
#include "dave_gz_world_plugins_msgs/msgs/StratifiedCurrentVelocity.pb.h"

// #include <cmath>
#include <gz/math/Vector4.hh>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

namespace dave_gz_world_plugins
{
class OceanCurrentWorldPlugin : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemUpdate,
                                public gz::sim::ISystemPostUpdate
{
public:
  OceanCurrentWorldPlugin();
  ~OceanCurrentWorldPlugin() override;

  // ----------------------------------------------

  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr);

  void LoadTidalOscillationDatabase();

  void LoadStratifiedCurrentDatabase();

  void LoadGlobalCurrentConfig();

  // ----------------------------------------------

  void Update(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm);

  // ----------------------------------------------

  void PostUpdate(const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm);

  void PublishCurrentVelocity();

  void PublishStratifiedCurrentVelocity();

  // ----------------------------------------------

  struct SharedData
  {
    // Gauss-Markov process instances for current models
    dave_gz_world_plugins::GaussMarkovProcess currentHorzAngleModel;  // Horizontal angle
    dave_gz_world_plugins::GaussMarkovProcess currentVelModel;        // Velocity
    dave_gz_world_plugins::GaussMarkovProcess currentVertAngleModel;  // Vertical angle

    // Stratified current database and models
    std::vector<gz::math::Vector3d> stratifiedDatabase;  // Database values for stratified currents
    std::vector<std::vector<dave_gz_world_plugins::GaussMarkovProcess>>
      stratifiedCurrentModels;  // Stratified current models

    // Tidal harmonic flag
    bool tidalHarmonicFlag;

    // Current velocity information
    gz::math::Vector3d currentVelocity;  // Current linear velocity vector
    std::vector<gz::math::Vector4d>
      currentStratifiedVelocity;  // Depth-specific linear velocity vectors for stratified current

    // Tidal harmonic data (M2, S2, N2 constituents)
    double M2_amp, M2_phase, M2_speed;  // M2 tidal constituent
    double S2_amp, S2_phase, S2_speed;  // S2 tidal constituent
    double N2_amp, N2_phase, N2_speed;  // N2 tidal constituent

    // Tidal oscillation mean directions
    double ebbDirection;    // Mean ebb direction
    double floodDirection;  // Mean flood direction

    // Tidal oscillation world start time (GMT)
    int world_start_time_day;
    int world_start_time_month;
    int world_start_time_year;
    int world_start_time_hour;
    int world_start_time_minute;

    // Topics
    std::string currentVelocityTopic;
    std::string stratifiedCurrentVelocityTopic;
    // std::string vehicleDepthTopic;

    // Date and speed information
    std::vector<std::array<int, 5>> dateGMT;  // Date in GMT (year, month, day, hour, minute)
    std::vector<double> speedcmsec;           // Speed in cm/sec

    // ------------------------------------------

    // // Gauss-Markov process instances for current models
    // std::shared_ptr<dave_gz_world_plugins::GaussMarkovProcess>
    //   currentHorzAngleModel;                                                     // Horizontal
    //   angle
    // std::shared_ptr<dave_gz_world_plugins::GaussMarkovProcess> currentVelModel;  // Velocity
    // std::shared_ptr<dave_gz_world_plugins::GaussMarkovProcess>
    //   currentVertAngleModel;  // Vertical angle

    // // Stratified current database and models
    // std::shared_ptr<std::vector<gz::math::Vector3d>>
    //   stratifiedDatabase;  // Database values for stratified currents
    // std::shared_ptr<std::vector<dave_gz_world_plugins::GaussMarkovProcess>>
    //   stratifiedCurrentModels;  // Stratified current models

    // // Tidal harmonic flag
    // std::shared_ptr<bool> tidalHarmonicFlag;

    // // Current velocity information
    // std::shared_ptr<gz::math::Vector3d> currentVelocity;  // Current linear velocity vector
    // std::shared_ptr<std::vector<gz::math::Vector4d>>
    //   currentStratifiedVelocity;  // Depth-specific linear velocity vectors for stratified
    //   current

    // // Tidal harmonic data (M2, S2, N2 constituents)
    // std::shared_ptr<double> M2_amp, M2_phase, M2_speed;  // M2 tidal constituent
    // std::shared_ptr<double> S2_amp, S2_phase, S2_speed;  // S2 tidal constituent
    // std::shared_ptr<double> N2_amp, N2_phase, N2_speed;  // N2 tidal constituent

    // // Tidal oscillation mean directions
    // std::shared_ptr<double> ebbDirection;    // Mean ebb direction
    // std::shared_ptr<double> floodDirection;  // Mean flood direction

    // // Tidal oscillation world start time (GMT)
    // std::shared_ptr<int> world_start_time_day;
    // std::shared_ptr<int> world_start_time_month;
    // std::shared_ptr<int> world_start_time_year;
    // std::shared_ptr<int> world_start_time_hour;
    // std::shared_ptr<int> world_start_time_minute;

    // // Topics
    // std::shared_ptr<std::string> currentVelocityTopic;
    // std::shared_ptr<std::string> stratifiedCurrentVelocityTopic;
    // // std::string vehicleDepthTopic;

    // // Date and speed information
    // std::shared_ptr<std::vector<std::array<int, 5>>>
    //   dateGMT;  // Date in GMT (year, month, day, hour, minute)
    // std::shared_ptr<std::vector<double>> speedcmsec;  // Speed in cm/sec
  };

  std::shared_ptr<SharedData> sharedDataPtr = std::make_shared<SharedData>();

private:
  // std::shared_ptr<rclcpp::Node> ros_node_;

  struct PrivateData;
  std::unique_ptr<PrivateData> dataPtr;
};

}  // namespace dave_gz_world_plugins

#endif  // DAVE_GZ_WORLD_PLUGINS__OCEAN_CURRENT_WORLD_PLUGIN_HH_
