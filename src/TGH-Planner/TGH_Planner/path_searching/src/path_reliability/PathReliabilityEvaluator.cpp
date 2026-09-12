#include <path_reliability/PathReliabilityEvaluator.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace fast_planner {

void PathReliabilityEvaluator::init(
    ros::NodeHandle& nh,
    const EDTEnvironment::Ptr& edt_environment,
    const RiskMapManager::Ptr& risk_map_manager) {
  Parameters parameters;
  nh.param("path_reliability/alpha", parameters.alpha, parameters.alpha);
  nh.param("path_reliability/beta", parameters.beta, parameters.beta);
  nh.param("path_reliability/gamma", parameters.gamma, parameters.gamma);
  nh.param("path_reliability/epsilon", parameters.epsilon, parameters.epsilon);
  nh.param("path_reliability/sample_resolution", parameters.sample_resolution,
           parameters.sample_resolution);
  nh.param("path_reliability/debug_output", parameters.debug_output,
           parameters.debug_output);
  params_ = sanitizeParameters(parameters);

  if (risk_map_manager) {
    risk_query_ = [risk_map_manager](const Eigen::Vector2d& position) {
      return risk_map_manager->getRisk(position);
    };
  } else {
    risk_query_ = RiskQuery();
  }

  if (edt_environment) {
    distance_query_ = [edt_environment](const Eigen::Vector3d& position) {
      Eigen::Vector3d query_position = position;
      return edt_environment->evaluateCoarseEDT(query_position, -1.0, true);
    };
  } else {
    distance_query_ = DistanceQuery();
  }

  if (!isReady()) {
    ROS_ERROR("PathReliabilityEvaluator requires valid risk and EDT interfaces.");
    return;
  }

  ROS_INFO_STREAM("PathReliabilityEvaluator initialized: alpha=" << params_.alpha
                  << ", beta=" << params_.beta
                  << ", gamma=" << params_.gamma
                  << ", epsilon=" << params_.epsilon
                  << ", sample_resolution=" << params_.sample_resolution
                  << ", debug_output=" << std::boolalpha << params_.debug_output);
}

PathReliabilityEvaluator::PathReliabilityEvaluator(
    const Parameters& parameters,
    RiskQuery risk_query,
    DistanceQuery distance_query)
    : params_(sanitizeParameters(parameters)),
      risk_query_(std::move(risk_query)),
      distance_query_(std::move(distance_query)) {}

PathSampleData PathReliabilityEvaluator::samplePath(
    const PathCandidate& candidate) const {
  PathSampleData sample_data;
  sample_data.path_id = candidate.path_id;

  if (!isReady()) {
    ROS_WARN_THROTTLE(1.0, "PathReliabilityEvaluator is not initialized.");
    logSampleData(sample_data);
    return sample_data;
  }

  const std::vector<Eigen::Vector3d> samples =
      generateSamplePositions(candidate);
  if (samples.empty()) {
    logSampleData(sample_data);
    return sample_data;
  }

  sample_data.positions.reserve(samples.size());
  sample_data.risk_sequence.reserve(samples.size());
  sample_data.distance_sequence.reserve(samples.size());

  double risk_sum = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  bool all_queries_valid = true;
  for (const Eigen::Vector3d& sample : samples) {
    const Eigen::Vector2d position = sample.head<2>();
    const double risk = risk_query_(position);
    const double distance = distance_query_(sample);

    sample_data.positions.push_back(position);
    sample_data.risk_sequence.push_back(risk);
    sample_data.distance_sequence.push_back(distance);

    if (!std::isfinite(risk) || risk < 0.0 || !std::isfinite(distance)) {
      all_queries_valid = false;
      continue;
    }
    risk_sum += risk;
    minimum_clearance = std::min(minimum_clearance, distance);
  }

  sample_data.valid = all_queries_valid;
  if (sample_data.valid) {
    sample_data.average_risk =
        risk_sum / static_cast<double>(sample_data.risk_sequence.size());
    sample_data.minimum_clearance = minimum_clearance;
  }

  logSampleData(sample_data);
  return sample_data;
}

ReliabilityMetrics PathReliabilityEvaluator::evaluate(
    const PathCandidate& candidate) const {
  ReliabilityMetrics metrics;
  const PathSampleData sample_data = samplePath(candidate);
  if (!sample_data.valid) {
    return metrics;
  }

  double clearance_risk_sum = 0.0;
  for (const double distance : sample_data.distance_sequence) {
    // A signed ESDF value at or inside an obstacle represents zero clearance.
    const double nonnegative_distance = std::max(0.0, distance);
    clearance_risk_sum += 1.0 / (nonnegative_distance + params_.epsilon);
  }

  const double sample_count =
      static_cast<double>(sample_data.risk_sequence.size());
  metrics.average_risk = sample_data.average_risk;
  metrics.clearance_risk = clearance_risk_sum / sample_count;

  double squared_deviation_sum = 0.0;
  for (const double risk : sample_data.risk_sequence) {
    const double deviation = risk - metrics.average_risk;
    squared_deviation_sum += deviation * deviation;
  }
  metrics.risk_variance = squared_deviation_sum / sample_count;

  const double exponent = -params_.alpha * metrics.average_risk -
                          params_.beta * metrics.clearance_risk -
                          params_.gamma * metrics.risk_variance;
  metrics.prs_score = std::exp(exponent);
  return metrics;
}

bool PathReliabilityEvaluator::isReady() const {
  return static_cast<bool>(risk_query_) && static_cast<bool>(distance_query_);
}

PathReliabilityEvaluator::Parameters
PathReliabilityEvaluator::sanitizeParameters(const Parameters& parameters) {
  Parameters sanitized = parameters;
  if (!std::isfinite(sanitized.alpha)) {
    sanitized.alpha = 1.0;
  } else {
    sanitized.alpha = std::max(0.0, sanitized.alpha);
  }
  if (!std::isfinite(sanitized.beta)) {
    sanitized.beta = 1.0;
  } else {
    sanitized.beta = std::max(0.0, sanitized.beta);
  }
  if (!std::isfinite(sanitized.gamma)) {
    sanitized.gamma = 1.0;
  } else {
    sanitized.gamma = std::max(0.0, sanitized.gamma);
  }
  if (!std::isfinite(sanitized.epsilon) || sanitized.epsilon <= 0.0) {
    sanitized.epsilon = 1e-3;
  }
  if (!std::isfinite(sanitized.sample_resolution) ||
      sanitized.sample_resolution <= 0.0) {
    sanitized.sample_resolution = 0.1;
  }
  return sanitized;
}

std::vector<Eigen::Vector3d> PathReliabilityEvaluator::generateSamplePositions(
    const PathCandidate& candidate) const {
  std::vector<Eigen::Vector3d> samples;
  if (candidate.path.empty()) {
    return samples;
  }
  for (const Eigen::Vector3d& point : candidate.path) {
    if (!point.allFinite()) {
      ROS_WARN("PathReliabilityEvaluator received a non-finite path point.");
      return samples;
    }
  }
  if (candidate.path.size() == 1) {
    samples.push_back(candidate.path.front());
    return samples;
  }

  std::vector<double> cumulative_lengths(candidate.path.size(), 0.0);
  for (std::size_t index = 1; index < candidate.path.size(); ++index) {
    cumulative_lengths[index] =
        cumulative_lengths[index - 1] +
        (candidate.path[index] - candidate.path[index - 1]).head<2>().norm();
  }

  const double total_length = cumulative_lengths.back();
  if (!std::isfinite(total_length)) {
    ROS_WARN("PathReliabilityEvaluator received a non-finite path length.");
    return samples;
  }
  if (total_length <= std::numeric_limits<double>::epsilon()) {
    samples.push_back(candidate.path.front());
    return samples;
  }

  std::size_t segment_index = 0;
  for (double arc_length = 0.0; arc_length < total_length;
       arc_length += params_.sample_resolution) {
    while (segment_index + 1 < cumulative_lengths.size() &&
           cumulative_lengths[segment_index + 1] < arc_length) {
      ++segment_index;
    }

    while (segment_index + 1 < cumulative_lengths.size() &&
           cumulative_lengths[segment_index + 1] -
                   cumulative_lengths[segment_index] <=
               std::numeric_limits<double>::epsilon()) {
      ++segment_index;
    }

    if (segment_index + 1 >= candidate.path.size()) break;
    const double segment_length = cumulative_lengths[segment_index + 1] -
                                  cumulative_lengths[segment_index];
    const double ratio =
        (arc_length - cumulative_lengths[segment_index]) / segment_length;
    samples.push_back((1.0 - ratio) * candidate.path[segment_index] +
                      ratio * candidate.path[segment_index + 1]);
  }

  if (samples.empty() ||
      (samples.back() - candidate.path.back()).head<2>().norm() >
          std::numeric_limits<double>::epsilon()) {
    samples.push_back(candidate.path.back());
  }
  return samples;
}

void PathReliabilityEvaluator::logSampleData(
    const PathSampleData& sample_data) const {
  if (!params_.debug_output) return;
  ROS_INFO_STREAM("[PRS Sampling] path_id=" << sample_data.path_id
                  << ", sample_number=" << sample_data.positions.size()
                  << ", average_risk=" << sample_data.average_risk
                  << ", minimum_clearance=" << sample_data.minimum_clearance);
}

}  // namespace fast_planner
