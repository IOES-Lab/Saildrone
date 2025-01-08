#ifndef GZ_SENSORS_MULTIBEAMSONAR_HH_
#define GZ_SENSORS_MULTIBEAMSONAR_HH_

#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/EnvironmentalData.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <opencv2/core/core.hpp>  // For OpenCV Mat
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gz
{
namespace sensors
{
/// \brief Kinematic state for an entity in the world.
///
/// All quantities are defined w.r.t. the world frame.
struct EntityKinematicState
{
  gz::math::Pose3d pose;
  gz::math::Vector3d linearVelocity;
  gz::math::Vector3d angularVelocity;
};

/// \brief Kinematic state for all entities in the world.
using WorldKinematicState = std::unordered_map<uint64_t, EntityKinematicState>;

/// \brief Kinematic state for all entities in the world.
struct WorldState
{
  WorldKinematicState kinematics;
  gz::math::SphericalCoordinates origin;
};

class MultibeamSonarSensor : public gz::sensors::RenderingSensor
{
public:
  MultibeamSonarSensor();

public:
  ~MultibeamSonarSensor();

  /// Inherits documentation from parent class
public:
  virtual bool Load(const sdf::Sensor & _sdf) override;

  /// Inherits documentation from parent class
public:
  virtual bool Load(sdf::ElementPtr _sdf) override;

  /// Inherits documentation from parent class
public:
  virtual bool Update(const std::chrono::steady_clock::duration & _now) override;

  /// Perform any sensor updates after the rendering pass
public:
  virtual void PostUpdate(const std::chrono::steady_clock::duration & _now);

  /// Inherits documentation from parent class
public:
  void SetScene(gz::rendering::ScenePtr _scene) override;

  /// \brief Set this sensor's entity ID (for world state lookup).
public:
  void SetEntity(uint64_t entity);

  /// \brief Set world `_state` to support DVL water and bottom-tracking.
public:
  void SetWorldState(const WorldState & _state);

  /// \brief Set environmental `_data` to support DVL water-tracking.
public:
  void SetEnvironmentalData(const EnvironmentalData & _data);

  /// \brief Inherits documentation from parent class
public:
  virtual bool HasConnections() const override;

  /// \brief Yield rendering sensors that underpin the implementation.
  ///
  /// \internal
public:
  std::vector<gz::rendering::SensorPtr> RenderingSensors() const;

private:
  class Implementation;

private:
  std::mutex lock_;

private:
  std::unique_ptr<Implementation> dataPtr;

  // ROS node handle member
  std::shared_ptr<rclcpp::Node> ros_node_;
  // Subscription for PointCloud2 messages
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  cv::Mat point_cloud_image_;         // Point cloud image
  cv::Mat point_cloud_normal_image_;  // Point cloud normal image
};

}  // namespace sensors
}  // namespace gz

#endif  // GZ_SENSORS_MULTIBEAMSONAR_HH_
