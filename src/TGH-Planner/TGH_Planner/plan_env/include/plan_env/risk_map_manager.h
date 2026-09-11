#ifndef PLAN_ENV_RISK_MAP_MANAGER_H_
#define PLAN_ENV_RISK_MAP_MANAGER_H_

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <limits>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fast_planner {

/**
 * @brief Builds a two-dimensional risk map from occupancy and ESDF data.
 *
 * Occupied and unknown cells bound a free-space corridor. For every known-free
 * cell, the horizontal and vertical free-space widths are computed and the
 * smaller one is used as the local corridor width. The raw risk values remain
 * available as doubles; the published OccupancyGrid is only a visualization.
 */
class RiskMapManager {
 public:
  using Ptr = std::shared_ptr<RiskMapManager>;

  struct Parameters {
    bool risk_enable = false;
    double lambda_distance = 1.0;
    double lambda_corridor = 1.0;
    double lambda_unknown = 1.0;
    double robot_width = 0.5;
  };

  RiskMapManager() = default;
  ~RiskMapManager() = default;

  void init(ros::NodeHandle& nh);

  /**
   * @brief Update from a standard ROS occupancy grid and a row-major ESDF map.
   *
   * Occupancy values below zero are unknown, values of 50 or greater are
   * occupied, and all other values are known free.
   */
  bool updateRiskMap(const nav_msgs::OccupancyGrid& occupancy_grid,
                     const std::vector<double>& esdf_map);

  /**
   * @brief Update directly from SDFMap's row-major 2-D buffers.
   *
   * Internal occupancy values are -1 (unknown), 0 (free), and 1 (occupied).
   */
  bool updateRiskMap(const std::vector<char>& occupancy_map,
                     const std::vector<double>& esdf_map,
                     int size_x,
                     int size_y,
                     double resolution,
                     const Eigen::Vector2d& origin,
                     const std::string& frame_id,
                     const ros::Time& stamp = ros::Time::now());

  double getRisk(const Eigen::Vector2d& position) const;
  double getRisk(int x, int y) const;
  double getCorridorWidth(int x, int y) const;
  std::vector<double> getRiskMap() const;
  std::vector<double> getCorridorWidthMap() const;
  nav_msgs::OccupancyGrid getRiskGrid() const;

  bool isEnabled() const { return params_.risk_enable; }
  bool isReady() const;
  const Parameters& getParameters() const { return params_; }

 private:
  enum CellState : int8_t { UNKNOWN = -1, FREE = 0, OCCUPIED = 1 };

  static constexpr double kEpsilon = 1e-3;

  bool computeRiskMap(const std::vector<int8_t>& occupancy,
                      const std::vector<double>& esdf_map,
                      int size_x,
                      int size_y,
                      double resolution,
                      const Eigen::Vector2d& origin,
                      const std::string& frame_id,
                      const ros::Time& stamp);
  void computeCorridorWidths(const std::vector<int8_t>& occupancy,
                             int size_x,
                             int size_y,
                             double resolution,
                             std::vector<double>& widths) const;
  static int toAddress(int x, int y, int size_x) { return x + y * size_x; }
  static int8_t decodeRosOccupancy(int8_t value);
  static int8_t decodeInternalOccupancy(char value);
  static int8_t riskToVisualizationValue(double risk);

  Parameters params_;
  ros::Publisher risk_pub_;

  mutable std::mutex mutex_;
  bool ready_ = false;
  int size_x_ = 0;
  int size_y_ = 0;
  double resolution_ = 0.0;
  Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
  std::vector<double> risk_map_;
  std::vector<double> corridor_width_map_;
  nav_msgs::OccupancyGrid risk_grid_;
};

}  // namespace fast_planner

#endif  // PLAN_ENV_RISK_MAP_MANAGER_H_
