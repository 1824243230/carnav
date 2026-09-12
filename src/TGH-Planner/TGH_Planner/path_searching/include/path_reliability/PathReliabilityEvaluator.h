#ifndef PATH_RELIABILITY_PATH_RELIABILITY_EVALUATOR_H_
#define PATH_RELIABILITY_PATH_RELIABILITY_EVALUATOR_H_

#include <Eigen/Core>
#include <plan_env/edt_environment.h>
#include <plan_env/risk_map_manager.h>
#include <ros/ros.h>

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace fast_planner {

struct PathCandidate {
  int path_id = -1;
  std::vector<Eigen::Vector3d> path;
};

struct PathSampleData {
  int path_id = -1;
  std::vector<Eigen::Vector2d> positions;
  std::vector<double> risk_sequence;
  std::vector<double> distance_sequence;
  double average_risk = std::numeric_limits<double>::infinity();
  double minimum_clearance = std::numeric_limits<double>::infinity();
  bool valid = false;
};

struct ReliabilityMetrics {
  double average_risk = std::numeric_limits<double>::infinity();
  double clearance_risk = std::numeric_limits<double>::infinity();
  double risk_variance = std::numeric_limits<double>::infinity();
  double prs_score = 0.0;
};

/**
 * @brief Computes path reliability without changing path selection or planning.
 *
 * Candidate paths are sampled uniformly in the XY plane. Risk samples come
 * from RiskMapManager, while clearance samples come from the existing 2-D EDT
 * interface. Invalid queries conservatively produce a zero PRS score.
 */
class PathReliabilityEvaluator {
 public:
  using Ptr = std::shared_ptr<PathReliabilityEvaluator>;
  using RiskQuery = std::function<double(const Eigen::Vector2d&)>;
  using DistanceQuery = std::function<double(const Eigen::Vector3d&)>;

  struct Parameters {
    double alpha = 1.0;
    double beta = 1.0;
    double gamma = 1.0;
    double epsilon = 1e-3;
    double sample_resolution = 0.1;
    bool debug_output = false;
  };

  PathReliabilityEvaluator() = default;

  /** Initialize the evaluator from the existing planner environment. */
  void init(ros::NodeHandle& nh,
            const EDTEnvironment::Ptr& edt_environment,
            const RiskMapManager::Ptr& risk_map_manager);

  /**
   * Construct an evaluator with injected queries. This keeps the mathematical
   * component independently testable without a running ROS map server.
   */
  PathReliabilityEvaluator(const Parameters& parameters,
                           RiskQuery risk_query,
                           DistanceQuery distance_query);

  /** Generate evaluation data without modifying the candidate path. */
  PathSampleData samplePath(const PathCandidate& candidate) const;

  ReliabilityMetrics evaluate(const PathCandidate& candidate) const;

  bool isReady() const;
  const Parameters& getParameters() const { return params_; }

 private:
  static Parameters sanitizeParameters(const Parameters& parameters);
  std::vector<Eigen::Vector3d> generateSamplePositions(
      const PathCandidate& candidate) const;
  void logSampleData(const PathSampleData& sample_data) const;

  Parameters params_;
  RiskQuery risk_query_;
  DistanceQuery distance_query_;
};

}  // namespace fast_planner

#endif  // PATH_RELIABILITY_PATH_RELIABILITY_EVALUATOR_H_
