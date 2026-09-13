#include "dvr/incremental_topo_graph_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace DynaVoro {
namespace {

struct EdgeRecord {
  IntPoint from;
  IntPoint to;
  std::vector<IntPoint> path;
};

bool pathIntersects(const EdgeRecord& edge, const GridBounds& bounds) {
  if (bounds.contains(edge.from.x, edge.from.y) ||
      bounds.contains(edge.to.x, edge.to.y)) return true;
  for (const auto& point : edge.path)
    if (bounds.contains(point.x, point.y)) return true;
  return false;
}

std::vector<EdgeRecord> collectEdges(gvg::GVG& graph) {
  std::vector<EdgeRecord> result;
  std::unordered_set<const gvg::GraphNode*> seen_nodes;
  for (const auto& component : graph.getGraphs()) {
    for (const auto& item : component) {
      const auto& node = item.second;
      if (!node || !seen_nodes.insert(node.get()).second) continue;
      const size_t count = std::min(node->neighbors.size(), node->neighbor_paths.size());
      for (size_t i = 0; i < count; ++i) {
        auto neighbor = node->neighbors[i].lock();
        if (!neighbor || node.get() >= neighbor.get()) continue;
        result.push_back({node->pos, neighbor->pos, node->neighbor_paths[i].path});
      }
    }
  }
  return result;
}

void collectNodes(gvg::GVG& graph,
                  std::unordered_map<IntPoint, gvg::GraphNode::NODE_TYPE>& nodes) {
  for (const auto& component : graph.getGraphs()) {
    for (const auto& item : component) {
      if (!item.second) continue;
      auto inserted = nodes.emplace(item.first, item.second->type);
      if (!inserted.second && item.second->type == gvg::GraphNode::Strong)
        inserted.first->second = gvg::GraphNode::Strong;
    }
  }
}

bool nearBoundary(const IntPoint& point, const GridBounds& bounds, int width = 2) {
  return bounds.contains(point.x, point.y) &&
         (point.x <= bounds.min_x + width || point.x >= bounds.max_x - width ||
          point.y <= bounds.min_y + width || point.y >= bounds.max_y - width);
}

}  // namespace

IncrementalTopoGraphManager::IncrementalTopoGraphManager(
    double update_radius_cells, int patch_halo_cells, int max_patch_expansions,
    double max_patch_fraction)
    : update_radius_cells_(std::max(0, static_cast<int>(std::ceil(update_radius_cells)))),
      patch_halo_cells_(std::max(2, patch_halo_cells)),
      max_patch_expansions_(std::max(1, max_patch_expansions)),
      max_patch_fraction_(std::max(0.05, std::min(1.0, max_patch_fraction))) {}

GridBounds IncrementalTopoGraphManager::inflateAndClamp(
    const GridBounds& bounds, int amount, int size_x, int size_y) const {
  if (!bounds.valid() || size_x <= 0 || size_y <= 0) return {};
  return {std::max(0, bounds.min_x - amount),
          std::max(0, bounds.min_y - amount),
          std::min(size_x - 1, bounds.max_x + amount),
          std::min(size_y - 1, bounds.max_y + amount)};
}

bool IncrementalTopoGraphManager::graphIntersects(
    const std::shared_ptr<gvg::GVG>& graph, const GridBounds& bounds,
    size_t& affected_nodes, size_t& affected_edges) const {
  affected_nodes = 0;
  affected_edges = 0;
  if (!bounds.valid()) return false;
  std::unordered_set<const gvg::GraphNode*> dirty_nodes;
  std::unordered_set<const gvg::GraphNode*> visited;
  for (const auto& component : graph->getGraphs()) {
    for (const auto& item : component) {
      const auto& node = item.second;
      if (!node || !visited.insert(node.get()).second) continue;
      const bool node_dirty = bounds.contains(node->pos.x, node->pos.y, update_radius_cells_);
      const size_t count = std::min(node->neighbors.size(), node->neighbor_paths.size());
      for (size_t i = 0; i < count; ++i) {
        auto neighbor = node->neighbors[i].lock();
        if (!neighbor || node.get() >= neighbor.get()) continue;
        bool edge_dirty = node_dirty ||
            bounds.contains(neighbor->pos.x, neighbor->pos.y, update_radius_cells_);
        for (const auto& point : node->neighbor_paths[i].path) {
          if (bounds.contains(point.x, point.y, update_radius_cells_)) {
            edge_dirty = true;
            break;
          }
        }
        if (edge_dirty) {
          ++affected_edges;
          dirty_nodes.insert(node.get());
          dirty_nodes.insert(neighbor.get());
        }
      }
      if (node_dirty) dirty_nodes.insert(node.get());
    }
  }
  affected_nodes = dirty_nodes.size();
  return affected_nodes != 0 || affected_edges != 0;
}

void IncrementalTopoGraphManager::countGraph(const std::shared_ptr<gvg::GVG>& graph) {
  std::unordered_set<const gvg::GraphNode*> nodes;
  for (const auto& component : graph->getGraphs())
    for (const auto& item : component)
      if (item.second) nodes.insert(item.second.get());
  stats_.graph_total_nodes = nodes.size();
}

void IncrementalTopoGraphManager::updateFrontiers(const MapChangeSet& changes,
                                                   FrontierState state) {
  if (state == FrontierState::ACTIVE) {
    for (const auto& occupied : changes.became_occupied) {
      for (auto& frontier : frontiers_) {
        if (std::abs(frontier.pos.x - occupied.x) <= update_radius_cells_ &&
            std::abs(frontier.pos.y - occupied.y) <= update_radius_cells_)
          frontier.state = FrontierState::STALE;
      }
    }
    const size_t stride = std::max<size_t>(1, changes.became_free.size() / 32);
    for (size_t i = 0; i < changes.became_free.size(); i += stride) {
      const auto& point = changes.became_free[i];
      bool exists = false;
      for (auto& frontier : frontiers_) {
        if (std::abs(frontier.pos.x - point.x) <= patch_halo_cells_ &&
            std::abs(frontier.pos.y - point.y) <= patch_halo_cells_) {
          frontier.state = FrontierState::ACTIVE;
          frontier.last_revision = changes.revision;
          exists = true;
          break;
        }
      }
      if (!exists && frontiers_.size() < 128)
        frontiers_.push_back({next_frontier_id_++, point, FrontierState::ACTIVE,
                              changes.revision});
    }
    return;
  }
  for (auto& frontier : frontiers_)
    if (frontier.last_revision == changes.revision && frontier.state != FrontierState::STALE)
      frontier.state = state;
}

bool IncrementalTopoGraphManager::buildAndMergePatch(
    const DynamicVoronoi& voronoi, const std::shared_ptr<gvg::GVG>& global_graph,
    const GridBounds& dirty_bounds) {
  const int size_x = voronoi.getSizeX();
  const int size_y = voronoi.getSizeY();
  GridBounds build_bounds = inflateAndClamp(
      dirty_bounds, update_radius_cells_ + patch_halo_cells_, size_x, size_y);
  if (!build_bounds.valid()) {
    stats_.fallback_reason = "invalid_dirty_region";
    return false;
  }

  std::shared_ptr<gvg::GVG> patch;
  for (int attempt = 0; attempt < max_patch_expansions_; ++attempt) {
    if (static_cast<double>(build_bounds.area()) >
        max_patch_fraction_ * static_cast<double>(size_x * size_y)) {
      stats_.fallback_reason = "roi_exceeds_limit";
      return false;
    }
    patch = std::make_shared<gvg::GVG>();
    global_graph->copyConfigurationTo(*patch);
    patch->createGraph(voronoi, build_bounds.min_x, build_bounds.min_y,
                       build_bounds.max_x, build_bounds.max_y);
    if (global_graph->regionBoundaryMatches(*patch, build_bounds.min_x,
                                            build_bounds.min_y, build_bounds.max_x,
                                            build_bounds.max_y, 2)) break;
    if (attempt + 1 == max_patch_expansions_) {
      stats_.fallback_reason = "unstable_patch_boundary";
      return false;
    }
    build_bounds = inflateAndClamp(build_bounds, patch_halo_cells_ * (attempt + 1),
                                   size_x, size_y);
  }
  stats_.local_roi_cells = build_bounds.area();

  std::unordered_map<IntPoint, gvg::GraphNode::NODE_TYPE> old_nodes;
  std::unordered_map<IntPoint, gvg::GraphNode::NODE_TYPE> patch_nodes;
  collectNodes(*global_graph, old_nodes);
  collectNodes(*patch, patch_nodes);
  stats_.patch_nodes = patch_nodes.size();
  std::unordered_map<IntPoint, gvg::GraphNode::NODE_TYPE> node_types;
  for (const auto& item : old_nodes)
    if (!build_bounds.contains(item.first.x, item.first.y)) node_types.emplace(item);
  for (const auto& item : patch_nodes) {
    auto inserted = node_types.emplace(item);
    if (!inserted.second) {
      ++stats_.deduplicated_nodes;
      if (item.second == gvg::GraphNode::Strong)
        inserted.first->second = gvg::GraphNode::Strong;
    }
  }

  const auto old_edges = collectEdges(*global_graph);
  const auto patch_edges = collectEdges(*patch);
  std::unordered_set<IntPoint> boundary_anchors;
  for (const auto& item : patch_nodes)
    if (nearBoundary(item.first, build_bounds)) boundary_anchors.insert(item.first);
  std::vector<EdgeRecord> merged_edges;
  merged_edges.reserve(old_edges.size() + patch_edges.size());
  bool stitch_failed = false;
  for (const auto& edge : old_edges) {
    if (!pathIntersects(edge, build_bounds)) {
      merged_edges.push_back(edge);
      continue;
    }
    std::vector<IntPoint> sequence;
    sequence.reserve(edge.path.size() + 2);
    sequence.push_back(edge.from);
    sequence.insert(sequence.end(), edge.path.begin(), edge.path.end());
    sequence.push_back(edge.to);
    auto add_connector = [&](bool reverse) {
      const IntPoint endpoint = reverse ? edge.to : edge.from;
      if (build_bounds.contains(endpoint.x, endpoint.y) || !node_types.count(endpoint)) return;
      std::vector<IntPoint> connector_path;
      for (size_t offset = 1; offset < sequence.size(); ++offset) {
        const size_t index = reverse ? sequence.size() - 1 - offset : offset;
        const IntPoint point = sequence[index];
        if (boundary_anchors.count(point)) {
          if (node_types.count(point)) merged_edges.push_back({endpoint, point, connector_path});
          return;
        }
        connector_path.push_back(point);
      }
      stitch_failed = true;
    };
    add_connector(false);
    add_connector(true);
  }
  if (stitch_failed) {
    stats_.fallback_reason = "boundary_stitch_failed";
    return false;
  }
  merged_edges.insert(merged_edges.end(), patch_edges.begin(), patch_edges.end());

  std::unordered_map<IntPoint, gvg::GraphNode::Ptr> nodes;
  for (const auto& item : node_types)
    nodes.emplace(item.first, std::make_shared<gvg::GraphNode>(
                                  item.first.x, item.first.y, item.second));
  for (const auto& edge : merged_edges) {
    auto from = nodes.find(edge.from);
    auto to = nodes.find(edge.to);
    if (from == nodes.end() || to == nodes.end() || from == to) continue;
    from->second->addNeighbor(to->second, edge.path, false);
    std::vector<IntPoint> reverse_path(edge.path.rbegin(), edge.path.rend());
    to->second->addNeighbor(from->second, reverse_path, false);
  }

  bool valid = true;
  for (const auto& item : nodes) {
    const auto& node = item.second;
    if (node->neighbors.size() != node->neighbor_paths.size()) {
      valid = false;
      break;
    }
    for (const auto& weak_neighbor : node->neighbors) {
      auto neighbor = weak_neighbor.lock();
      if (!neighbor || !nodes.count(neighbor->pos)) {
        valid = false;
        break;
      }
      bool reciprocal = false;
      for (const auto& back : neighbor->neighbors) {
        auto back_node = back.lock();
        if (back_node && back_node->pos == node->pos) {
          reciprocal = true;
          break;
        }
      }
      if (!reciprocal) {
        valid = false;
        break;
      }
    }
    if (!valid) break;
  }
  if (!valid) {
    ++stats_.graph_validation_fail_count;
    stats_.fallback_reason = "merged_graph_validation_failed";
    return false;
  }

  std::vector<std::unordered_map<IntPoint, gvg::GraphNode::Ptr>> components;
  std::unordered_set<IntPoint> visited;
  for (const auto& item : nodes) {
    if (!visited.insert(item.first).second) continue;
    components.emplace_back();
    std::queue<gvg::GraphNode::Ptr> queue;
    queue.push(item.second);
    while (!queue.empty()) {
      auto node = queue.front();
      queue.pop();
      components.back()[node->pos] = node;
      for (const auto& weak_neighbor : node->neighbors) {
        auto neighbor = weak_neighbor.lock();
        if (neighbor && visited.insert(neighbor->pos).second) queue.push(neighbor);
      }
    }
  }
  stats_.merged_nodes = nodes.size();
  global_graph->commitRegionalGraph(std::move(components), *patch,
                                    build_bounds.min_x, build_bounds.min_y,
                                    build_bounds.max_x, build_bounds.max_y);
  return true;
}

bool IncrementalTopoGraphManager::update(
    const DynamicVoronoi& voronoi, const std::shared_ptr<gvg::GVG>& global_graph,
    const MapChangeSet& changes) {
  const auto begin = std::chrono::steady_clock::now();
  stats_.graph_updated_nodes = 0;
  stats_.graph_updated_edges = 0;
  stats_.local_roi_cells = 0;
  stats_.patch_nodes = 0;
  stats_.merged_nodes = 0;
  stats_.deduplicated_nodes = 0;
  stats_.fallback_reason.clear();

  if (!initialized_ || changes.full_map) {
    stats_.fallback_reason = !initialized_ ? "initial_build" : "map_resize_or_full_update";
    global_graph->createGraph(voronoi);
    initialized_ = true;
    ++stats_.full_rebuild_count;
    countGraph(global_graph);
    stats_.incremental_update_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
    return true;
  }
  if (changes.empty() && !changes.known_area_expanded) {
    countGraph(global_graph);
    stats_.incremental_update_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
    return false;
  }

  graphIntersects(global_graph, changes.bounds,
                  stats_.graph_updated_nodes, stats_.graph_updated_edges);
  updateFrontiers(changes, FrontierState::ACTIVE);
  updateFrontiers(changes, FrontierState::EXPANDING);
  if (buildAndMergePatch(voronoi, global_graph, changes.bounds)) {
    ++stats_.local_repair_count;
    if (!changes.became_free.empty() || changes.known_area_expanded)
      ++stats_.frontier_expansion_count;
    updateFrontiers(changes, FrontierState::EXPANDED);
  } else {
    updateFrontiers(changes, FrontierState::BLOCKED);
    global_graph->createGraph(voronoi);
    ++stats_.full_rebuild_count;
  }
  countGraph(global_graph);
  stats_.incremental_update_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin).count();
  return true;
}

}  // namespace DynaVoro
