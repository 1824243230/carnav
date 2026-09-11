#include <path_searching/risk_aware_edge.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace fast_planner {

void RiskAwareEdge::init(ros::NodeHandle& nh,
                         const RiskMapManager::Ptr& risk_map_manager,
                         double map_resolution) {
  nh.param("risk_aware_graph/alpha", params_.alpha, 1.0);
  nh.param("risk_aware_graph/beta", params_.beta, 1.0);
  nh.param("risk_aware_graph/gamma", params_.gamma, 0.0);
  nh.param("risk_aware_graph/edge_sample_resolution", params_.sample_resolution,
           map_resolution);

  params_.alpha = std::max(0.0, params_.alpha);
  params_.beta = std::max(0.0, params_.beta);
  params_.gamma = std::max(0.0, params_.gamma);
  if (params_.sample_resolution <= 0.0) {
    params_.sample_resolution = std::max(1e-3, map_resolution);
  }
  risk_map_manager_ = risk_map_manager;

  ROS_INFO_STREAM("RiskAwareEdge initialized: alpha=" << params_.alpha
                  << ", beta=" << params_.beta
                  << ", gamma=" << params_.gamma
                  << ", sample_resolution=" << params_.sample_resolution);
}

RiskEdge RiskAwareEdge::evaluateEdge(NodeID start,
                                     NodeID end,
                                     const Eigen::Vector3d& start_position,
                                     const Eigen::Vector3d& end_position,
                                     const Eigen::Vector3d* previous_position) const {
  RiskEdge edge;
  edge.start = start;
  edge.end = end;
  edge.length = (end_position - start_position).head<2>().norm();
  edge.risk_cost = sampleEdgeRisk(start_position, end_position, edge.length);
  if (previous_position != nullptr) {
    edge.curvature_cost =
        computeCurvatureCost(*previous_position, start_position, end_position);
  }
  edge.total_cost = params_.alpha * edge.length +
                    params_.beta * edge.risk_cost +
                    params_.gamma * edge.curvature_cost;
  return edge;
}

RiskPathCost RiskAwareEdge::evaluatePath(const std::vector<Eigen::Vector3d>& path) const {
  RiskPathCost path_cost;
  if (path.size() < 2) {
    path_cost.total_cost = 0.0;
    return path_cost;
  }

  path_cost.edges.reserve(path.size() - 1);
  double risk_sum = 0.0;
  double curvature_sum = 0.0;
  for (size_t index = 0; index + 1 < path.size(); ++index) {
    const Eigen::Vector3d* previous = index > 0 ? &path[index - 1] : nullptr;
    RiskEdge edge = evaluateEdge(static_cast<NodeID>(index),
                                 static_cast<NodeID>(index + 1),
                                 path[index], path[index + 1], previous);
    path_cost.length += edge.length;
    risk_sum += edge.risk_cost;
    curvature_sum += edge.curvature_cost;
    path_cost.edges.emplace_back(std::move(edge));
  }

  const double edge_count = static_cast<double>(path_cost.edges.size());
  path_cost.risk = risk_sum / edge_count;
  path_cost.curvature_cost = curvature_sum / edge_count;
  path_cost.total_cost = params_.alpha * path_cost.length +
                         params_.beta * path_cost.risk +
                         params_.gamma * path_cost.curvature_cost;
  return path_cost;
}

double RiskAwareEdge::sampleEdgeRisk(const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& end,
                                     double length) const {
  if (params_.beta <= 0.0 || !risk_map_manager_ || !risk_map_manager_->isEnabled()) {
    return 0.0;
  }
  if (!risk_map_manager_->isReady()) {
    ROS_WARN_THROTTLE(1.0, "RiskAwareEdge: risk map is not ready; using distance-only cost.");
    return 0.0;
  }

  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(length / params_.sample_resolution)));
  double risk_sum = 0.0;
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double ratio = static_cast<double>(sample) / segment_count;
    const Eigen::Vector2d position =
        ((1.0 - ratio) * start + ratio * end).head<2>();
    const double risk = risk_map_manager_->getRisk(position);
    if (!std::isfinite(risk)) {
      return std::numeric_limits<double>::infinity();
    }
    risk_sum += risk;
  }
  return risk_sum / static_cast<double>(segment_count + 1);
}

double RiskAwareEdge::computeCurvatureCost(const Eigen::Vector3d& previous,
                                           const Eigen::Vector3d& current,
                                           const Eigen::Vector3d& next) {
  const Eigen::Vector2d incoming = (current - previous).head<2>();
  const Eigen::Vector2d outgoing = (next - current).head<2>();
  if (incoming.norm() < 1e-6 || outgoing.norm() < 1e-6) {
    return 0.0;
  }

  const Eigen::Vector2d incoming_unit = incoming.normalized();
  const Eigen::Vector2d outgoing_unit = outgoing.normalized();
  const double dot = std::max(-1.0, std::min(1.0, incoming_unit.dot(outgoing_unit)));
  const double cross = incoming_unit.x() * outgoing_unit.y() -
                       incoming_unit.y() * outgoing_unit.x();
  return std::abs(std::atan2(cross, dot));
}

}  // namespace fast_planner
