#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dvr/GVG.h"
#include "dvr/dynamicvoronoi.h"

namespace DynaVoro {

struct GridBounds {
  int min_x = 0;
  int min_y = 0;
  int max_x = -1;
  int max_y = -1;

  bool valid() const { return min_x <= max_x && min_y <= max_y; }
  bool contains(int x, int y, int margin = 0) const {
    return valid() && x >= min_x - margin && x <= max_x + margin &&
           y >= min_y - margin && y <= max_y + margin;
  }
  int area() const { return valid() ? (max_x - min_x + 1) * (max_y - min_y + 1) : 0; }
};

struct MapChangeSet {
  uint64_t revision = 0;
  GridBounds bounds;
  std::vector<IntPoint> became_occupied;
  std::vector<IntPoint> became_free;
  bool known_area_expanded = false;
  bool full_map = false;

  bool empty() const { return became_occupied.empty() && became_free.empty(); }
};

struct IncrementalGraphStats {
  size_t graph_total_nodes = 0;
  size_t graph_updated_nodes = 0;
  size_t graph_updated_edges = 0;
  uint64_t full_rebuild_count = 0;
  uint64_t local_repair_count = 0;
  uint64_t frontier_expansion_count = 0;
  uint64_t graph_validation_fail_count = 0;
  size_t local_roi_cells = 0;
  size_t patch_nodes = 0;
  size_t merged_nodes = 0;
  size_t deduplicated_nodes = 0;
  double incremental_update_time_ms = 0.0;
  std::string fallback_reason;
};

enum class FrontierState { ACTIVE, EXPANDING, EXPANDED, BLOCKED, STALE };

struct FrontierNode {
  uint64_t id = 0;
  IntPoint pos;
  FrontierState state = FrontierState::ACTIVE;
  uint64_t last_revision = 0;
};

// Maintains a persistent global graph and repairs only a bounded GVG patch.
// The original GVG extraction pipeline is used inside the patch; graph/cache
// replacement is transactional and falls back to a full build on validation
// failure or an update whose influence cannot be bounded safely.
class IncrementalTopoGraphManager {
 public:
  explicit IncrementalTopoGraphManager(double update_radius_cells = 3.0,
                                       int patch_halo_cells = 8,
                                       int max_patch_expansions = 3,
                                       double max_patch_fraction = 0.35);

  bool update(const DynamicVoronoi& voronoi,
              const std::shared_ptr<gvg::GVG>& global_graph,
              const MapChangeSet& changes);

  const IncrementalGraphStats& stats() const { return stats_; }
  const std::vector<FrontierNode>& frontiers() const { return frontiers_; }

 private:
  bool graphIntersects(const std::shared_ptr<gvg::GVG>& graph,
                       const GridBounds& bounds,
                       size_t& affected_nodes,
                       size_t& affected_edges) const;
  void countGraph(const std::shared_ptr<gvg::GVG>& graph);
  GridBounds inflateAndClamp(const GridBounds& bounds, int amount,
                             int size_x, int size_y) const;
  bool buildAndMergePatch(const DynamicVoronoi& voronoi,
                          const std::shared_ptr<gvg::GVG>& global_graph,
                          const GridBounds& dirty_bounds);
  void updateFrontiers(const MapChangeSet& changes, FrontierState state);

  int update_radius_cells_ = 3;
  int patch_halo_cells_ = 8;
  int max_patch_expansions_ = 3;
  double max_patch_fraction_ = 0.35;
  bool initialized_ = false;
  uint64_t next_frontier_id_ = 1;
  std::vector<FrontierNode> frontiers_;
  IncrementalGraphStats stats_;
};

}  // namespace DynaVoro
