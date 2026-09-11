#include <plan_env/risk_map_manager.h>

#include <algorithm>
#include <cmath>

namespace fast_planner {

void RiskMapManager::init(ros::NodeHandle& nh) {
  nh.param("risk_map/risk_enable", params_.risk_enable, false);
  nh.param("risk_map/lambda_distance", params_.lambda_distance, 1.0);
  nh.param("risk_map/lambda_corridor", params_.lambda_corridor, 1.0);
  nh.param("risk_map/lambda_unknown", params_.lambda_unknown, 1.0);
  nh.param("risk_map/robot_width", params_.robot_width, 0.5);

  params_.lambda_distance = std::max(0.0, params_.lambda_distance);
  params_.lambda_corridor = std::max(0.0, params_.lambda_corridor);
  params_.lambda_unknown = std::max(0.0, params_.lambda_unknown);
  params_.robot_width = std::max(0.0, params_.robot_width);

  risk_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/risk_map/risk_2D", 1, true);
  ROS_INFO_STREAM("RiskMapManager initialized: enabled=" << std::boolalpha
                  << params_.risk_enable << ", lambda_distance="
                  << params_.lambda_distance << ", lambda_corridor="
                  << params_.lambda_corridor << ", lambda_unknown="
                  << params_.lambda_unknown << ", robot_width="
                  << params_.robot_width);
}

bool RiskMapManager::updateRiskMap(const nav_msgs::OccupancyGrid& occupancy_grid,
                                   const std::vector<double>& esdf_map) {
  const int size_x = static_cast<int>(occupancy_grid.info.width);
  const int size_y = static_cast<int>(occupancy_grid.info.height);
  const size_t cell_count = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  if (occupancy_grid.data.size() != cell_count) {
    ROS_ERROR_THROTTLE(1.0, "RiskMapManager: occupancy grid size is inconsistent with metadata.");
    return false;
  }

  std::vector<int8_t> occupancy(cell_count, UNKNOWN);
  std::transform(occupancy_grid.data.begin(), occupancy_grid.data.end(), occupancy.begin(),
                 decodeRosOccupancy);

  const Eigen::Vector2d origin(occupancy_grid.info.origin.position.x,
                               occupancy_grid.info.origin.position.y);
  return computeRiskMap(occupancy, esdf_map, size_x, size_y,
                        occupancy_grid.info.resolution, origin,
                        occupancy_grid.header.frame_id, occupancy_grid.header.stamp);
}

bool RiskMapManager::updateRiskMap(const std::vector<char>& occupancy_map,
                                   const std::vector<double>& esdf_map,
                                   int size_x,
                                   int size_y,
                                   double resolution,
                                   const Eigen::Vector2d& origin,
                                   const std::string& frame_id,
                                   const ros::Time& stamp) {
  std::vector<int8_t> occupancy(occupancy_map.size(), UNKNOWN);
  std::transform(occupancy_map.begin(), occupancy_map.end(), occupancy.begin(),
                 decodeInternalOccupancy);
  return computeRiskMap(occupancy, esdf_map, size_x, size_y, resolution, origin,
                        frame_id, stamp);
}

bool RiskMapManager::computeRiskMap(const std::vector<int8_t>& occupancy,
                                    const std::vector<double>& esdf_map,
                                    int size_x,
                                    int size_y,
                                    double resolution,
                                    const Eigen::Vector2d& origin,
                                    const std::string& frame_id,
                                    const ros::Time& stamp) {
  if (size_x <= 0 || size_y <= 0 || resolution <= 0.0) {
    ROS_ERROR_THROTTLE(1.0, "RiskMapManager: invalid map metadata.");
    return false;
  }

  const size_t cell_count = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  if (occupancy.size() != cell_count || esdf_map.size() != cell_count) {
    ROS_ERROR_THROTTLE(1.0, "RiskMapManager: occupancy and ESDF map sizes do not match.");
    return false;
  }

  std::vector<double> widths;
  computeCorridorWidths(occupancy, size_x, size_y, resolution, widths);

  std::vector<double> risks(cell_count, 0.0);
  nav_msgs::OccupancyGrid risk_grid;
  risk_grid.header.frame_id = frame_id;
  risk_grid.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
  risk_grid.info.resolution = resolution;
  risk_grid.info.width = size_x;
  risk_grid.info.height = size_y;
  risk_grid.info.origin.position.x = origin.x();
  risk_grid.info.origin.position.y = origin.y();
  risk_grid.info.origin.orientation.w = 1.0;
  risk_grid.data.resize(cell_count, 0);

  if (params_.risk_enable) {
    for (size_t index = 0; index < cell_count; ++index) {
      const double distance = std::max(0.0, esdf_map[index]);
      const double distance_risk = 1.0 / (distance + kEpsilon);
      const double free_margin = std::max(0.0, widths[index] - params_.robot_width);
      const double corridor_risk = 1.0 / (free_margin + kEpsilon);
      const double unknown_risk = occupancy[index] == UNKNOWN ? 1.0 : 0.0;

      risks[index] = params_.lambda_distance * distance_risk +
                     params_.lambda_corridor * corridor_risk +
                     params_.lambda_unknown * unknown_risk;
      risk_grid.data[index] = riskToVisualizationValue(risks[index]);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_ = origin;
    risk_map_.swap(risks);
    corridor_width_map_.swap(widths);
    risk_grid_ = risk_grid;
    ready_ = true;
  }

  risk_pub_.publish(risk_grid);
  return true;
}

void RiskMapManager::computeCorridorWidths(const std::vector<int8_t>& occupancy,
                                           int size_x,
                                           int size_y,
                                           double resolution,
                                           std::vector<double>& widths) const {
  const size_t cell_count = static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  std::vector<double> horizontal(cell_count, 0.0);
  std::vector<double> vertical(cell_count, 0.0);

  // A non-free cell is a corridor boundary. Distances are measured from the
  // cell center to the boundary of the nearest non-free cell or map boundary.
  for (int y = 0; y < size_y; ++y) {
    int run_start = 0;
    while (run_start < size_x) {
      while (run_start < size_x && occupancy[toAddress(run_start, y, size_x)] != FREE) {
        ++run_start;
      }
      int run_end = run_start;
      while (run_end < size_x && occupancy[toAddress(run_end, y, size_x)] == FREE) {
        ++run_end;
      }
      const double width = (run_end - run_start) * resolution;
      for (int x = run_start; x < run_end; ++x) {
        horizontal[toAddress(x, y, size_x)] = width;
      }
      run_start = run_end;
    }
  }

  for (int x = 0; x < size_x; ++x) {
    int run_start = 0;
    while (run_start < size_y) {
      while (run_start < size_y && occupancy[toAddress(x, run_start, size_x)] != FREE) {
        ++run_start;
      }
      int run_end = run_start;
      while (run_end < size_y && occupancy[toAddress(x, run_end, size_x)] == FREE) {
        ++run_end;
      }
      const double width = (run_end - run_start) * resolution;
      for (int y = run_start; y < run_end; ++y) {
        vertical[toAddress(x, y, size_x)] = width;
      }
      run_start = run_end;
    }
  }

  widths.resize(cell_count, 0.0);
  for (size_t index = 0; index < cell_count; ++index) {
    if (occupancy[index] == FREE) {
      widths[index] = std::min(horizontal[index], vertical[index]);
    }
  }
}

double RiskMapManager::getRisk(const Eigen::Vector2d& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || resolution_ <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const int x = static_cast<int>(std::floor((position.x() - origin_.x()) / resolution_));
  const int y = static_cast<int>(std::floor((position.y() - origin_.y()) / resolution_));
  if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    return std::numeric_limits<double>::infinity();
  }
  return risk_map_[toAddress(x, y, size_x_)];
}

double RiskMapManager::getRisk(int x, int y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    return std::numeric_limits<double>::infinity();
  }
  return risk_map_[toAddress(x, y, size_x_)];
}

double RiskMapManager::getCorridorWidth(const Eigen::Vector2d& position) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || resolution_ <= 0.0) {
    return 0.0;
  }

  const int x = static_cast<int>(std::floor((position.x() - origin_.x()) / resolution_));
  const int y = static_cast<int>(std::floor((position.y() - origin_.y()) / resolution_));
  if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    return 0.0;
  }
  return corridor_width_map_[toAddress(x, y, size_x_)];
}

double RiskMapManager::getCorridorWidth(int x, int y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_ || x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    return 0.0;
  }
  return corridor_width_map_[toAddress(x, y, size_x_)];
}

std::vector<double> RiskMapManager::getRiskMap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return risk_map_;
}

std::vector<double> RiskMapManager::getCorridorWidthMap() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return corridor_width_map_;
}

nav_msgs::OccupancyGrid RiskMapManager::getRiskGrid() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return risk_grid_;
}

bool RiskMapManager::isReady() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

int8_t RiskMapManager::decodeRosOccupancy(int8_t value) {
  if (value < 0) {
    return UNKNOWN;
  }
  return value >= 50 ? OCCUPIED : FREE;
}

int8_t RiskMapManager::decodeInternalOccupancy(char value) {
  if (value == static_cast<char>(-1)) {
    return UNKNOWN;
  }
  const int state = static_cast<int>(value);
  return state > 0 ? OCCUPIED : FREE;
}

int8_t RiskMapManager::riskToVisualizationValue(double risk) {
  if (!std::isfinite(risk) || risk <= 0.0) {
    return risk > 0.0 ? 100 : 0;
  }
  // Fixed monotonic compression: raw risk 1 maps to 50, and infinity to 100.
  return static_cast<int8_t>(std::round(100.0 * risk / (1.0 + risk)));
}

}  // namespace fast_planner
