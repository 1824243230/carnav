#include <gtest/gtest.h>

#include <path_reliability/PathReliabilityEvaluator.h>

#include <cmath>
#include <limits>

namespace fast_planner {
namespace {

TEST(PathReliabilityEvaluatorTest, ComputesMetricsFromUniformPathSamples) {
  PathReliabilityEvaluator::Parameters parameters;
  parameters.alpha = 0.5;
  parameters.beta = 0.25;
  parameters.gamma = 0.75;
  parameters.epsilon = 0.5;
  parameters.sample_resolution = 1.0;

  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d& position) { return position.x() + 1.0; },
      [](const Eigen::Vector3d&) { return 1.5; });

  PathCandidate candidate;
  candidate.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(2.0, 0.0, 0.0)};

  const ReliabilityMetrics metrics = evaluator.evaluate(candidate);
  const double expected_average_risk = 2.0;
  const double expected_clearance_risk = 0.5;
  const double expected_variance = 2.0 / 3.0;
  const double expected_prs =
      std::exp(-parameters.alpha * expected_average_risk -
               parameters.beta * expected_clearance_risk -
               parameters.gamma * expected_variance);

  EXPECT_NEAR(metrics.average_risk, expected_average_risk, 1e-12);
  EXPECT_NEAR(metrics.clearance_risk, expected_clearance_risk, 1e-12);
  EXPECT_NEAR(metrics.risk_variance, expected_variance, 1e-12);
  EXPECT_NEAR(metrics.prs_score, expected_prs, 1e-12);
}

TEST(PathReliabilityEvaluatorTest, GeneratesRiskAndDistanceSequencesByArcLength) {
  PathReliabilityEvaluator::Parameters parameters;
  parameters.sample_resolution = 0.1;

  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d& position) { return 2.0 * position.x(); },
      [](const Eigen::Vector3d& position) { return 1.0 - position.x(); });

  PathCandidate candidate;
  candidate.path_id = 7;
  candidate.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(0.15, 0.0, 0.0),
                    Eigen::Vector3d(0.30, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> original_path = candidate.path;

  const PathSampleData sample_data = evaluator.samplePath(candidate);

  ASSERT_TRUE(sample_data.valid);
  EXPECT_EQ(sample_data.path_id, 7);
  ASSERT_EQ(sample_data.positions.size(), 4U);
  ASSERT_EQ(sample_data.risk_sequence.size(), 4U);
  ASSERT_EQ(sample_data.distance_sequence.size(), 4U);
  for (std::size_t index = 0; index < sample_data.positions.size(); ++index) {
    const double expected_x = 0.1 * static_cast<double>(index);
    EXPECT_NEAR(sample_data.positions[index].x(), expected_x, 1e-12);
    EXPECT_NEAR(sample_data.positions[index].y(), 0.0, 1e-12);
    EXPECT_NEAR(sample_data.risk_sequence[index], 2.0 * expected_x, 1e-12);
    EXPECT_NEAR(sample_data.distance_sequence[index], 1.0 - expected_x, 1e-12);
  }
  EXPECT_NEAR(sample_data.average_risk, 0.3, 1e-12);
  EXPECT_NEAR(sample_data.minimum_clearance, 0.7, 1e-12);

  ASSERT_EQ(candidate.path.size(), original_path.size());
  for (std::size_t index = 0; index < candidate.path.size(); ++index) {
    EXPECT_TRUE(candidate.path[index].isApprox(original_path[index], 0.0));
  }
}

TEST(PathReliabilityEvaluatorTest, DoesNotDuplicateSharedPathVertices) {
  PathReliabilityEvaluator::Parameters parameters;
  parameters.alpha = 1.0;
  parameters.beta = 0.0;
  parameters.gamma = 0.0;
  parameters.sample_resolution = 1.0;

  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d& position) { return position.x(); },
      [](const Eigen::Vector3d&) { return 1.0; });

  PathCandidate candidate;
  candidate.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(1.0, 0.0, 0.0),
                    Eigen::Vector3d(2.0, 0.0, 0.0)};

  const ReliabilityMetrics metrics = evaluator.evaluate(candidate);
  EXPECT_NEAR(metrics.average_risk, 1.0, 1e-12);
  EXPECT_NEAR(metrics.risk_variance, 2.0 / 3.0, 1e-12);
  EXPECT_NEAR(metrics.prs_score, std::exp(-1.0), 1e-12);
}

TEST(PathReliabilityEvaluatorTest, TreatsNonPositiveEsdfAsZeroClearance) {
  PathReliabilityEvaluator::Parameters parameters;
  parameters.alpha = 0.0;
  parameters.beta = 1.0;
  parameters.gamma = 0.0;
  parameters.epsilon = 0.25;

  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d&) { return 0.0; },
      [](const Eigen::Vector3d&) { return -0.5; });

  PathCandidate candidate;
  candidate.path = {Eigen::Vector3d::Zero()};

  const ReliabilityMetrics metrics = evaluator.evaluate(candidate);
  EXPECT_NEAR(metrics.clearance_risk, 4.0, 1e-12);
  EXPECT_NEAR(metrics.prs_score, std::exp(-4.0), 1e-12);
}

TEST(PathReliabilityEvaluatorTest, EmptyOrInvalidCandidateIsUnreliable) {
  PathReliabilityEvaluator::Parameters parameters;
  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d&) {
        return std::numeric_limits<double>::infinity();
      },
      [](const Eigen::Vector3d&) { return 1.0; });

  const ReliabilityMetrics empty_metrics = evaluator.evaluate(PathCandidate());
  EXPECT_TRUE(std::isinf(empty_metrics.average_risk));
  EXPECT_DOUBLE_EQ(empty_metrics.prs_score, 0.0);

  PathCandidate invalid_candidate;
  invalid_candidate.path = {Eigen::Vector3d::Zero()};
  const ReliabilityMetrics invalid_metrics = evaluator.evaluate(invalid_candidate);
  EXPECT_TRUE(std::isinf(invalid_metrics.average_risk));
  EXPECT_DOUBLE_EQ(invalid_metrics.prs_score, 0.0);
}

TEST(PathReliabilityEvaluatorTest, RejectsNonFinitePathWithoutQueryingMaps) {
  PathReliabilityEvaluator::Parameters parameters;
  std::size_t query_count = 0;
  PathReliabilityEvaluator evaluator(
      parameters,
      [&query_count](const Eigen::Vector2d&) {
        ++query_count;
        return 0.0;
      },
      [&query_count](const Eigen::Vector3d&) {
        ++query_count;
        return 1.0;
      });

  PathCandidate candidate;
  candidate.path = {
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0)};

  const PathSampleData sample_data = evaluator.samplePath(candidate);

  EXPECT_FALSE(sample_data.valid);
  EXPECT_TRUE(sample_data.positions.empty());
  EXPECT_TRUE(sample_data.risk_sequence.empty());
  EXPECT_TRUE(sample_data.distance_sequence.empty());
  EXPECT_EQ(query_count, 0U);
}

TEST(PathReliabilityEvaluatorTest, SanitizesNonFiniteWeightsAndRejectsNegativeRisk) {
  PathReliabilityEvaluator::Parameters parameters;
  parameters.alpha = std::numeric_limits<double>::infinity();
  parameters.beta = std::numeric_limits<double>::quiet_NaN();
  parameters.gamma = -1.0;
  PathReliabilityEvaluator evaluator(
      parameters,
      [](const Eigen::Vector2d&) { return -0.1; },
      [](const Eigen::Vector3d&) { return 1.0; });

  EXPECT_DOUBLE_EQ(evaluator.getParameters().alpha, 1.0);
  EXPECT_DOUBLE_EQ(evaluator.getParameters().beta, 1.0);
  EXPECT_DOUBLE_EQ(evaluator.getParameters().gamma, 0.0);

  PathCandidate candidate;
  candidate.path = {Eigen::Vector3d::Zero()};
  const ReliabilityMetrics metrics = evaluator.evaluate(candidate);
  EXPECT_TRUE(std::isinf(metrics.average_risk));
  EXPECT_DOUBLE_EQ(metrics.prs_score, 0.0);
}

}  // namespace
}  // namespace fast_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
