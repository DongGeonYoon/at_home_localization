// VoxelLayer that forgets everything not seen within observation_persistence.
//
// The stock VoxelLayer only frees a cell when a ray passes through it. The
// Mid-360 cannot see the low band near the robot (-7 deg lower FOV), and the
// layer is kept in the map frame, so people who walked off and marks shifted by
// map->odom corrections stay forever. Here the voxel grid is wiped before every
// update and rebuilt from the observation buffer, so a mark lives exactly as
// long as the marking source's observation_persistence.
//
// Wiping also drops low obstacles the moment they fall below the lower FOV
// edge as the robot closes in (a 0.2 m box vanishes ~1.6 m ahead). With
// blind_memory_time > 0, marked points are remembered and re-marked while the
// sensor cannot see them (outside its elevation FOV or range); a remembered
// point is dropped as soon as it is back in view, where the live scan decides,
// or after blind_memory_time. blind_memory_time = 0 is the plain wipe above.

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/voxel_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace costmap_host
{

class FreshVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
public:
  void onInitialize() override
  {
    VoxelLayer::onInitialize();
    declareParameter("blind_memory_time", rclcpp::ParameterValue(0.0));
    declareParameter("sensor_min_elevation_deg", rclcpp::ParameterValue(-7.0));
    declareParameter("sensor_max_elevation_deg", rclcpp::ParameterValue(52.0));
    // Points this close to the FOV edge count as unseen (edge returns are sparse).
    declareParameter("fov_margin_deg", rclcpp::ParameterValue(2.0));
    declareParameter("sensor_min_range", rclcpp::ParameterValue(0.10));
    declareParameter("sensor_max_range", rclcpp::ParameterValue(4.0));
    auto node = node_.lock();
    const std::string p = getName() + ".";
    node->get_parameter(p + "blind_memory_time", blind_memory_time_);
    node->get_parameter(p + "sensor_min_elevation_deg", min_elevation_deg_);
    node->get_parameter(p + "sensor_max_elevation_deg", max_elevation_deg_);
    node->get_parameter(p + "fov_margin_deg", fov_margin_deg_);
    node->get_parameter(p + "sensor_min_range", sensor_min_range_);
    node->get_parameter(p + "sensor_max_range", sensor_max_range_);
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (enabled_) {
      resetMaps();
      if (blind_memory_time_ > 0.0) {
        updateBlindMemory();
      }
    }
    VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    // The wipe may have freed cells anywhere in the (rolling) window.
    *min_x = std::min(*min_x, getOriginX());
    *min_y = std::min(*min_y, getOriginY());
    *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
    *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
  }

private:
  struct Remembered
  {
    double x, y, z;
    double stamp;
  };

  static constexpr double kColumn = 0.1;

  static long long columnKey(double x, double y)
  {
    return (static_cast<long long>(std::lround(x / kColumn)) << 32) ^
           (std::lround(y / kColumn) & 0xFFFFFFFFLL);
  }

  bool visibleFrom(const geometry_msgs::msg::Point & sensor, const Remembered & p) const
  {
    const double dx = p.x - sensor.x, dy = p.y - sensor.y, dz = p.z - sensor.z;
    const double planar = std::hypot(dx, dy);
    const double range = std::hypot(planar, dz);
    if (range < sensor_min_range_ || range > sensor_max_range_) {
      return false;
    }
    const double elevation = std::atan2(dz, planar) * 180.0 / M_PI;
    return elevation > min_elevation_deg_ + fov_margin_deg_ &&
           elevation < max_elevation_deg_ - fov_margin_deg_;
  }

  // Refresh the memory from the live marking buffer and hand the points the
  // sensor cannot see right now to the layer as a static marking observation.
  void updateBlindMemory()
  {
    clearStaticObservations(true, false);
    std::vector<nav2_costmap_2d::Observation> observations;
    getMarkingObservations(observations);
    if (observations.empty()) {
      return;
    }
    const double now = clock_->now().seconds();

    // Newest observation gives the sensor position used for visibility.
    const nav2_costmap_2d::Observation * newest = &observations.front();
    for (const auto & obs : observations) {
      if (rclcpp::Time(obs.cloud_->header.stamp) > rclcpp::Time(newest->cloud_->header.stamp)) {
        newest = &obs;
      }
    }
    const geometry_msgs::msg::Point sensor = newest->origin_;

    // In view: the live scan is authoritative. Out of view: keep until timeout.
    // Columns that lose a visible remembered point are checked below: if the
    // live scan has nothing left there, the object moved (a person walking
    // off), so its parts hidden in the blind band are dropped too. A low box
    // whose top has already left the FOV has no visible part and is kept.
    std::unordered_map<long long, bool> vacated;
    for (auto it = memory_.begin(); it != memory_.end(); ) {
      if (now - it->second.stamp > blind_memory_time_) {
        it = memory_.erase(it);
      } else if (visibleFrom(sensor, it->second)) {
        vacated[columnKey(it->second.x, it->second.y)] = true;
        it = memory_.erase(it);
      } else {
        ++it;
      }
    }
    // Whole marking buffer (observation_persistence), not just the newest
    // scan: one Mid-360 frame can miss a standing person entirely.
    for (const auto & obs : observations) {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*obs.cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*obs.cloud_, "y");
      for (; ix != ix.end(); ++ix, ++iy) {
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            vacated.erase(columnKey(*ix + dx * kColumn, *iy + dy * kColumn));
          }
        }
      }
    }
    if (!vacated.empty()) {
      for (auto it = memory_.begin(); it != memory_.end(); ) {
        if (vacated.count(columnKey(it->second.x, it->second.y))) {
          it = memory_.erase(it);
        } else {
          ++it;
        }
      }
    }

    // Remember what the live scan marks (same range limits as marking).
    const double res = getResolution();
    for (const auto & obs : observations) {
      const double stamp = rclcpp::Time(obs.cloud_->header.stamp).seconds();
      const double max_sq = obs.obstacle_max_range_ * obs.obstacle_max_range_;
      const double min_sq = obs.obstacle_min_range_ * obs.obstacle_min_range_;
      sensor_msgs::PointCloud2ConstIterator<float> ix(*obs.cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*obs.cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*obs.cloud_, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        const double dx = *ix - obs.origin_.x, dy = *iy - obs.origin_.y, dz = *iz - obs.origin_.z;
        const double sq = dx * dx + dy * dy + dz * dz;
        if (sq >= max_sq || sq < min_sq) {
          continue;
        }
        const long kx = std::lround(*ix / res), ky = std::lround(*iy / res);
        const long kz = std::lround(*iz / 0.1);
        const long long key = (static_cast<long long>(kx) << 42) ^
          (static_cast<long long>(ky & 0x1FFFFF) << 21) ^ (kz & 0x1FFFFF);
        auto & m = memory_[key];
        if (stamp >= m.stamp) {
          m = Remembered{*ix, *iy, *iz, stamp};
        }
      }
    }

    // Re-mark only the remembered points that are out of view now.
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = global_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    std::vector<const Remembered *> unseen;
    for (const auto & kv : memory_) {
      if (!visibleFrom(sensor, kv.second)) {
        unseen.push_back(&kv.second);
      }
    }
    if (unseen.empty()) {
      return;
    }
    modifier.resize(unseen.size());
    sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x"), oy(cloud, "y"), oz(cloud, "z");
    for (const auto * p : unseen) {
      *ox = p->x; *oy = p->y; *oz = p->z;
      ++ox; ++oy; ++oz;
    }
    geometry_msgs::msg::Point origin = sensor;
    nav2_costmap_2d::Observation remembered(origin, cloud, 1e3, 0.0, 0.0, 0.0);
    addStaticObservation(remembered, true, false);
  }

  double blind_memory_time_{0.0};
  double min_elevation_deg_{-7.0}, max_elevation_deg_{52.0}, fov_margin_deg_{2.0};
  double sensor_min_range_{0.10}, sensor_max_range_{4.0};
  std::unordered_map<long long, Remembered> memory_;
};

}  // namespace costmap_host

PLUGINLIB_EXPORT_CLASS(costmap_host::FreshVoxelLayer, nav2_costmap_2d::Layer)
