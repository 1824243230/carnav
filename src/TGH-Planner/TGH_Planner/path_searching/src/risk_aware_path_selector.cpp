#include <path_searching/risk_aware_path_selector.h>

#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

double dynamicScale(double value, double threshold, double gain) {
  if (threshold <= kEpsilon || value <= threshold) {
    return 1.0;
  }
  const double excess_ratio = std::min(1.0, (value - threshold) / threshold);
  return 1.0 + std::max(0.0, gain) * excess_ratio;
}

}  // namespace

void RiskAwarePathSelector::init(ros::NodeHandle& nh,
                                 const RiskMapManager::Ptr& risk_map_manager,
                                 double map_resolution) {
  risk_map_manager_ = risk_map_manager;
  map_resolution_ = std::max(map_resolution, kEpsilon);

  nh.param("risk_aware_path_selector/w1", params_.w1, params_.w1);
  nh.param("risk_aware_path_selector/w2", params_.w2, params_.w2);
  nh.param("risk_aware_path_selector/high_risk_threshold",
           params_.high_risk_threshold, params_.high_risk_threshold);
  nh.param("risk_aware_path_selector/open_space_width_threshold",
           params_.open_space_width_threshold, params_.open_space_width_threshold);
  nh.param("risk_aware_path_selector/dynamic_weight_gain",
           params_.dynamic_weight_gain, params_.dynamic_weight_gain);
  nh.param("risk_aware_path_selector/orientation_tie_threshold",
           params_.orientation_tie_threshold, params_.orientation_tie_threshold);
  nh.param("risk_aware_path_selector/sample_resolution",
           params_.sample_resolution, params_.sample_resolution);

  params_.w1 = std::max(0.0, params_.w1);
  params_.w2 = std::max(0.0, params_.w2);
  params_.orientation_tie_threshold = std::max(0.0, params_.orientation_tie_threshold);
  params_.sample_resolution = std::max(params_.sample_resolution, map_resolution_);

  ROS_INFO_STREAM("RiskAwarePathSelector initialized: w1=" << params_.w1
                  << ", w2=" << params_.w2
                  << ", high_risk_threshold=" << params_.high_risk_threshold
                  << ", open_space_width_threshold="
                  << params_.open_space_width_threshold);
}

PathSelectionResult RiskAwarePathSelector::selectBestPath(
    const std::vector<PathSelectionCandidate>& candidates,
    double start_yaw) const {
  PathSelectionResult result;
  if (candidates.empty()) {
    return result;
  }

  double min_length = std::numeric_limits<double>::infinity();
  double max_length = -std::numeric_limits<double>::infinity();
  double min_risk = std::numeric_limits<double>::infinity();
  double max_risk = -std::numeric_limits<double>::infinity();
  double risk_sum = 0.0;
  std::size_t valid_risk_count = 0;

  for (const auto& candidate : candidates) {
    if (std::isfinite(candidate.length)) {
      min_length = std::min(min_length, candidate.length);
      max_length = std::max(max_length, candidate.length);
    }
    if (std::isfinite(candidate.risk)) {
      min_risk = std::min(min_risk, candidate.risk);
      max_risk = std::max(max_risk, candidate.risk);
      risk_sum += candidate.risk;
      ++valid_risk_count;
    }
  }

  if (!std::isfinite(min_length)) {
    min_length = max_length = 0.0;
  }
  if (!std::isfinite(min_risk)) {
    min_risk = max_risk = 0.0;
  }

  result.average_risk = valid_risk_count > 0
                            ? risk_sum / static_cast<double>(valid_risk_count)
                            : 0.0;
  result.average_corridor_width = computeAverageCorridorWidth(candidates);
  result.w1 = params_.w1 * dynamicScale(result.average_corridor_width,
                                        params_.open_space_width_threshold,
                                        params_.dynamic_weight_gain);
  result.w2 = params_.w2 * dynamicScale(result.average_risk,
                                        params_.high_risk_threshold,
                                        params_.dynamic_weight_gain);

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    const double length_score = std::isfinite(candidate.length)
                                    ? 1.0 - normalizedValue(candidate.length, min_length,
                                                            max_length, 0.0)
                                    : 0.0;
    const double risk_penalty = std::isfinite(candidate.risk)
                                    ? normalizedValue(candidate.risk, min_risk,
                                                      max_risk, 0.0)
                                    : 1.0;
    const double cost = result.w1 * length_score - result.w2 * risk_penalty;
    const double orientation_error = initialHeadingError(candidate.path, start_yaw);

    const bool higher_cost = !result.success ||
                             cost > result.cost + params_.orientation_tie_threshold;
    const bool orientation_preferred = result.success &&
        std::abs(cost - result.cost) <= params_.orientation_tie_threshold &&
        orientation_error < result.orientation_error;
    if (higher_cost || orientation_preferred) {
      result.success = true;
      result.best_index = i;
      result.best_path = candidate.path;
      result.cost = cost;
      result.normalized_length = length_score;
      result.normalized_risk = risk_penalty;
      result.orientation_error = orientation_error;
    }
  }

  return result;
}

double RiskAwarePathSelector::computeAverageCorridorWidth(
    const std::vector<PathSelectionCandidate>& candidates) const {
  if (!risk_map_manager_ || !risk_map_manager_->isReady()) {
    return 0.0;
  }

  double width_sum = 0.0;
  std::size_t sample_count = 0;
  for (const auto& candidate : candidates) {
    for (std::size_t i = 0; i + 1 < candidate.path.size(); ++i) {
      const Eigen::Vector2d start = candidate.path[i].head<2>();
      const Eigen::Vector2d end = candidate.path[i + 1].head<2>();
      const double length = (end - start).norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(
          length / params_.sample_resolution)));
      for (int step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const double width = risk_map_manager_->getCorridorWidth(
            start + ratio * (end - start));
        if (std::isfinite(width) && width > 0.0) {
          width_sum += width;
          ++sample_count;
        }
      }
    }
  }
  return sample_count > 0 ? width_sum / static_cast<double>(sample_count) : 0.0;
}

double RiskAwarePathSelector::initialHeadingError(
    const std::vector<Eigen::Vector3d>& path, double start_yaw) {
  if (path.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d direction = path[i].head<2>() - path[0].head<2>();
    if (direction.squaredNorm() > kEpsilon) {
      return std::abs(wrapAngle(std::atan2(direction.y(), direction.x()) - start_yaw));
    }
  }
  return std::numeric_limits<double>::infinity();
}

double RiskAwarePathSelector::wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double RiskAwarePathSelector::normalizedValue(double value,
                                              double minimum,
                                              double maximum,
                                              double equal_value) {
  const double range = maximum - minimum;
  if (range <= kEpsilon) {
    return equal_value;
  }
  return std::max(0.0, std::min(1.0, (value - minimum) / range));
}

}  // namespace fast_planner
