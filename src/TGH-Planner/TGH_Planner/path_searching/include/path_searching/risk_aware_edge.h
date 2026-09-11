#ifndef PATH_SEARCHING_RISK_AWARE_EDGE_H_
#define PATH_SEARCHING_RISK_AWARE_EDGE_H_

#include <Eigen/Core>
#include <plan_env/risk_map_manager.h>
#include <ros/ros.h>

#include <limits>
#include <memory>
#include <vector>

namespace fast_planner {

using NodeID = int;

struct RiskEdge {
  NodeID start = -1;
  NodeID end = -1;
  double length = 0.0;
  double risk_cost = 0.0;
  double curvature_cost = 0.0;
  double total_cost = 0.0;
};

struct RiskPathCost {
  double length = 0.0;
  double risk = 0.0;
  double curvature_cost = 0.0;
  double total_cost = std::numeric_limits<double>::infinity();
  std::vector<RiskEdge> edges;
};

/**
 * @brief Evaluates Visibility-PRM edges using geometry and the 2-D risk map.
 *
 * Risk is sampled uniformly along each edge. A path's risk is the arithmetic
 * mean of its edge risks, as required by R(path) = sum(edge risk) / N.
 * Curvature is represented by the absolute heading change at each vertex.
 */
class RiskAwareEdge {
 public:
  using Ptr = std::shared_ptr<RiskAwareEdge>;

  struct Parameters {
    double alpha = 1.0;
    double beta = 1.0;
    double gamma = 0.0;
    double sample_resolution = 0.1;
  };

  RiskAwareEdge() = default;
  ~RiskAwareEdge() = default;

  void init(ros::NodeHandle& nh,
            const RiskMapManager::Ptr& risk_map_manager,
            double map_resolution);

  RiskEdge evaluateEdge(NodeID start,
                        NodeID end,
                        const Eigen::Vector3d& start_position,
                        const Eigen::Vector3d& end_position,
                        const Eigen::Vector3d* previous_position = nullptr) const;

  RiskPathCost evaluatePath(const std::vector<Eigen::Vector3d>& path) const;

  const Parameters& getParameters() const { return params_; }

 private:
  double sampleEdgeRisk(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& end,
                        double length) const;
  static double computeCurvatureCost(const Eigen::Vector3d& previous,
                                     const Eigen::Vector3d& current,
                                     const Eigen::Vector3d& next);

  Parameters params_;
  RiskMapManager::Ptr risk_map_manager_;
};

}  // namespace fast_planner

#endif  // PATH_SEARCHING_RISK_AWARE_EDGE_H_
