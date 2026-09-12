#include <gtest/gtest.h>

#include <path_searching/risk_aware_path_selector.h>

namespace fast_planner {
namespace {

std::vector<PathSelectionCandidate> makeReliabilityCandidates() {
  PathSelectionCandidate low_reliability;
  low_reliability.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                          Eigen::Vector3d(1.0, 0.0, 0.0)};
  low_reliability.length = 1.0;
  low_reliability.risk = 1.0;
  low_reliability.prs_score = 0.1;

  PathSelectionCandidate high_reliability = low_reliability;
  high_reliability.prs_score = 0.9;
  return {low_reliability, high_reliability};
}

TEST(PathReliabilityRankingTest, RewardsHigherPrsWhenEnabled) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.reliability_enabled = true;
  parameters.lambda_length = 1.0;
  parameters.lambda_risk = 1.0;
  parameters.lambda_prs = 1.0;
  parameters.orientation_tie_threshold = 0.0;
  RiskAwarePathSelector selector(parameters);

  const PathSelectionResult result =
      selector.selectBestPath(makeReliabilityCandidates(), 0.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.best_index, 1U);
  EXPECT_NEAR(result.prs_score, 0.9, 1e-12);
  EXPECT_TRUE(result.reliability_enabled);
}

TEST(PathReliabilityRankingTest, IgnoresPrsAndUsesLegacyRankingWhenDisabled) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.reliability_enabled = false;
  parameters.orientation_tie_threshold = 0.0;
  RiskAwarePathSelector selector(parameters);

  const PathSelectionResult result =
      selector.selectBestPath(makeReliabilityCandidates(), 0.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.best_index, 0U);
  EXPECT_DOUBLE_EQ(result.prs_score, 0.0);
  EXPECT_FALSE(result.reliability_enabled);
}

}  // namespace
}  // namespace fast_planner
