# MyCar：面向二维未知环境的增量拓扑引导路径规划系统

## 1. 文档目的

本文档说明 `/home/hyh/mycar_ws` 中路径规划系统的技术方案、主要创新、软件架构和使用方法。系统以 TGH-Planner 为主体，保留其 TGH 拓扑路径规划、HEC 同伦等价判断、历史拓扑路径容器和 B-spline 局部轨迹优化，并在此基础上加入：

- 二维 Occupancy/ESDF 风险地图；
- 风险感知的拓扑候选路径评价；
- 路径可靠性评分（Path Reliability Score，PRS）；
- 地图变化检测与版本管理；
- EGVG 全局拓扑图的局部增量修复；
- 历史路径的局部验证、修复与选择性 HEC。

本文只描述当前源码中已经存在的功能。这里的“风险”是由二维占据状态、障碍距离、通道宽度和未知区域构成的导航风险，不包含地形高程、坡度、纵向/横向 terrain risk、PCA 地形拟合或 3D elevation map。

## 2. 项目定位

系统面向四轮非完整约束移动机器人在二维未知或动态变化环境中的在线导航。传统的单一路径规划容易在障碍布局变化时频繁切换路线，也可能只考虑路径长度而忽略狭窄通道、未知空间和轨迹可执行性。本项目采用分层结构：

1. Occupancy Map 表示已知自由、占据和未知区域；
2. ESDF 提供机器人到障碍物的连续距离信息；
3. EGVG 提取自由空间骨架，表达环境连通性；
4. TGH 维护不同同伦类别的历史候选路径；
5. 风险与可靠性模块对候选路径排序并生成 Guide Path；
6. Kinodynamic A* 在 Guide Path 引导下生成满足非完整约束的初始轨迹；
7. B-spline 优化器进一步处理平滑性、避障和动力学可行性；
8. FSM 持续检查目标、地图和当前轨迹并触发重规划。

总体数据流如下：

```text
LiDAR / Depth + Odometry
          |
          v
2D Occupancy Map + ESDF
          |
          +--> 2D Risk Map
          |
          v
Map Change Detection (Delta M + revision)
          |
          v
IncrementalTopoGraphManager
  |-- regional EGVG build
  |-- boundary consistency check
  |-- patch/global graph merge
  |-- frontier state management
  `-- safe full-rebuild fallback
          |
          v
TGH topological path generation
          |
          v
Historical path validation/repair + Selective HEC
          |
          v
Length/Risk/PRS candidate ranking
          |
          v
Guide Path
          |
          v
Guide-aware Kinodynamic A* + B-spline optimization
          |
          v
Trajectory publication and vehicle tracking
```

## 3. 核心创新

### 3.1 TGH 分层拓扑引导规划

TGH 将全局连通性选择与局部可执行轨迹生成分开处理。EGVG/TGH 负责回答“从障碍物哪一侧绕行”，局部规划器负责回答“车辆如何以满足速度、加速度和转向约束的方式运动”。这种结构避免直接在高维状态空间中盲目搜索所有拓扑可能性。

实际调用链为：

```text
KinoReplanFSM::callKinodynamicReplan()
  -> FastPlannerManager::TopoPathReplan()
  -> TopologyPRM::findVoroPaths()
  -> TopologyPRM::findGuidePath()
  -> FastPlannerManager::kinodynamicReplan()
  -> KinodynamicAstar::setGuidePath()/search()
  -> B-spline trajectory optimization
```

主要实现位置：

- `TGH_Planner/plan_manage/src/kino_replan_fsm.cpp`
- `TGH_Planner/plan_manage/src/planner_manager.cpp`
- `TGH_Planner/path_searching/src/topo_prm.cpp`
- `TGH_Planner/path_searching/src/kinodynamic_astar.cpp`
- `TGH_Planner/bspline_opt/src/bspline_optimizer.cpp`

### 3.2 EGVG 对自由空间拓扑的紧凑表达

`gvg::GVG::createGraph()` 从 Dynamic Voronoi 栅格中提取图结构，并区分强节点、弱节点与边。构图过程包含 Voronoi 点识别、邻接分类、DFS 图生成、寄生边处理、包络线补全、局部网格结构清理以及最终 GVG 化。

与稠密栅格相比，EGVG 将自由空间压缩为节点和边，使多拓扑路径搜索更多地在连通结构上运行。`obs_clearance` 删除过近的骨架，`obs_clearance_high` 限制离障碍过远的骨架并形成包络表达。

主要实现位置：

- `dynamicvoronoi/include/dvr/GVG.h`
- `dynamicvoronoi/src/dynamicvoronoi.cpp`
- `dynamicvoronoi/src/voronoi_layer.cpp`

### 3.3 HEC 同伦等价判断与候选去重

`TopologyPRM::sameTopoPath()` 是当前 HEC 判断入口。它先处理两条路径的公共前缀和公共后缀，再以相同数量的点对两条路径进行弧长离散化。对应点分别与中点执行可见性检测；若变形带被障碍物阻断，则认为路径不属于同一拓扑类别。

其作用不是简单比较路径长度或坐标相似度，而是判断两条路线能否在无碰撞条件下连续变形，从而保留真正不同的绕障方式并删除同伦重复候选。

### 3.4 跨规划周期的历史拓扑路径复用

历史路径没有在每次重规划后全部清空，而是保存在：

- `TopologyPRM::path_container_front_`：优先使用的近组候选；
- `TopologyPRM::path_container_back_`：后备候选。

`TopoPath` 除路径几何外，还保存长度、风险、曲率代价、总成本、碰撞断点、当前选择状态和地图验证版本。路径容器通过 `preprocess()` 完成碰撞检查、失效路径重连、起点变化衔接、路径缩短、成本更新、排序和同伦去重。

这种机制带来两点收益：

- 环境没有影响某条路线时直接延续其拓扑决策，减少路线抖动；
- 局部障碍使路径失效时保留其安全前后段，只修复中间碰撞区间。

### 3.5 基于地图版本的局部变化检测

`VoronoiLayer::update_by_occupancy_map()` 比较当前二维占据缓存与 `last_occupancy_map_`，记录：

- 由自由变为占据的栅格 `became_occupied`；
- 由占据变为自由的栅格 `became_free`；
- 变化包围盒 `GridBounds`；
- 已知区域是否扩展 `known_area_expanded`；
- 地图是否发生尺寸变化或需要完整处理 `full_map`；
- 单调递增的地图版本号 `revision`。

实时感知流程直接复用 SDFMap 已有的 `local_bound_min_` 和 `local_bound_max_` 作为扫描范围，避免正常情况下重新比较整幅地图。变化记录保存在有界的 `map_change_history_` 中，路径模块可通过 `getMapChangesSince()` 合并取得自上次验证后的所有变化。如果请求的旧版本已从历史队列中淘汰，系统会要求完整验证，避免漏检。

### 3.6 持久化全局图与局部 EGVG 补丁修复

新增的 `DynaVoro::IncrementalTopoGraphManager` 长期维护 `VoronoiLayer::gvg_`，地图变化后只在脏区域周围构建局部补丁：

1. 用更新半径和 halo 扩张 `Delta M`；
2. 调用原 `GVG::createGraph()` 在矩形 ROI 内构建临时图；
3. 使用 `regionBoundaryMatches()` 检查局部骨架与全局图在边界环上的一致性；
4. 若拓扑影响到达边界，则逐步扩大 ROI；
5. 删除全局图中与 ROI 相交的旧节点和旧边；
6. 合入局部补丁节点和边，并按栅格坐标去重；
7. 对跨边界旧边建立 connector，检查邻接表与反向连接；
8. 验证通过后由 `commitRegionalGraph()` 事务式更新图和栅格缓存。

该机制不是使用另一套规划算法替换 TGH，而是在局部仍调用原 EGVG 构图过程，因此保留了原图语义和后续路径生成接口。

### 3.7 安全回退保证图一致性

局部更新不是无条件提交。以下情况会回退到原来的全图 `createGraph(voronoi)`：

- 首次构图或地图尺寸改变；
- ROI 面积超过整图允许比例；
- 多次扩张后补丁边界仍不稳定；
- 跨边界连接无法可靠拼接；
- 合并后的邻居、路径或双向连接验证失败。

因此系统在变化较小时获得增量更新效率，在变化范围过大或无法证明局部更新安全时保持原算法的完整性。

### 3.8 Frontier 状态驱动的新区域扩展

对于从未知变为自由的区域，管理器建立 `FrontierNode` 并维护以下状态：

```text
ACTIVE -> EXPANDING -> EXPANDED
                    `-> BLOCKED
ACTIVE -------------> STALE
```

Frontier 目前用于记录新自由空间、驱动对应脏区的局部 EGVG 构建并统计扩展次数。它不是独立的 TRG frontier 搜索器，也不会引入新的 A*；局部扩展继续复用原 EGVG 构图逻辑。

### 3.9 历史路径的分级验证与局部失效修复

`TopoPath::PATH_STATE` 将历史路径划分为：

- `VALID`：地图变化未与路径相交，或者已在当前版本验证通过；
- `AFFECTED`：路径经过脏区域，需要局部碰撞检查或 HEC；
- `INVALID`：脏区中的路径片段已经碰撞。

`TopologyPRM::pathIntersectsDirtyRegion()` 先用带安全裕量的包围盒进行路径段相交检测。未相交的路径直接复用；相交路径由 `checkPathObstacle3()` 只在脏区附近查询 ESDF。若发现碰撞，则保存碰撞区之前和之后的安全片段，随后由 `reconnectTopoPaths()` 和 `reconnectBreakPath()` 使用已有二维 A* 接回断裂路径。

这里的 A* 只承担历史路径局部断裂段的修复，不替换 TGH/EGVG 的拓扑路径生成。

### 3.10 选择性 HEC

完整的两两 HEC 会随候选数量增长产生明显开销。当前实现为每条历史路径保存 `path_id`、`state`、`validated_map_revision` 和 `geometry_version`，在 `updateAllPaths()` 中只把下列路径加入 HEC 脏集合：

- 新生成路径；
- 地图变化影响的路径；
- 局部修复后几何发生变化的路径；
- 起点连接发生变化的路径。

两条均为 `VALID` 且几何未变化的历史路径不重复执行 HEC；只要比较双方至少一条为脏路径，仍执行原 `sameTopoPath()`，并且包含 front/front、back/back 和 front/back 三类比较，保证最终候选集合不保留已知的同伦重复路径。

### 3.11 二维风险地图

`RiskMapManager` 从二维占据栅格和 ESDF 计算每个栅格的导航风险。自由通道宽度取水平连续自由宽度与垂直连续自由宽度的较小值：

```text
w(x) = min(w_horizontal(x), w_vertical(x))
```

当前栅格风险为：

```text
r(x) = lambda_distance / (d_esdf(x) + epsilon)
     + lambda_corridor / (max(0, w(x) - robot_width) + epsilon)
     + lambda_unknown * I_unknown(x)
```

该设计同时惩罚靠近障碍物、剩余空间不足以及未知区域。原始 double 风险供规划使用，压缩到 `[0, 100]` 的 `nav_msgs/OccupancyGrid` 仅用于 RViz 显示，话题为 `/risk_map/risk_2D`。

### 3.12 风险感知的候选路径评价

`RiskAwareEdge` 沿每条路径边均匀采样二维风险，并计算：

```text
J_path = alpha * path_length
       + beta  * average_risk
       + gamma * average_turn_cost
```

`RiskAwarePathSelector` 将候选长度与风险归一化，在默认模式下最大化：

```text
Utility = w1 * normalized_length_score - w2 * normalized_risk
```

当平均风险升高时可动态提高风险权重；当环境较开阔时可动态提高长度权重。效用接近时，以候选路径初始方向和车辆当前航向之间的误差作为 tie-breaker，降低四轮车起步时不必要的大转向。

### 3.13 路径可靠性评分 PRS

可选的 `PathReliabilityEvaluator` 沿候选路径按弧长均匀采样，结合平均风险、ESDF 净空风险和风险方差计算：

```text
PRS = exp(-alpha * average_risk
          -beta  * clearance_risk
          -gamma * risk_variance)
```

启用后，最终候选效用变为：

```text
Utility = lambda_length * normalized_length_score
        - lambda_risk   * normalized_risk
        + lambda_prs    * PRS
```

这使路径选择不仅关注平均风险，还关注最小净空和风险沿路径的稳定程度。

### 3.14 Guide Path 与非完整约束轨迹优化

`TopologyPRM::findGuidePath()` 从历史与新增候选中选出拓扑引导路径并发布 `/TopoPlan/guide_path`。该路径传给 `KinodynamicAstar::setGuidePath()`，用于约束状态空间搜索方向。

对于多拓扑局部绕障流程，`FastPlannerManager::optimizeTopoBspline()` 采用两阶段优化：

1. `GUIDE_PHASE`：平滑项与 Guide Path 项共同作用，使控制点进入目标拓扑通道；
2. `NORMAL_PHASE`：优化平滑性、障碍距离和动力学可行性。

随后按 jerk 选择较优轨迹，并通过 `refineTraj()` 调整时间参数和物理约束。全局层只提供几何/拓扑引导，最终输出仍是适合车辆跟踪的非均匀 B-spline。

## 4. 关键数据结构

| 数据结构 | 作用 | 位置 |
|---|---|---|
| `MapChangeSet` | 保存地图版本、变化包围盒、占据/释放栅格和已知区域扩展信息 | `dynamicvoronoi/include/dvr/incremental_topo_graph_manager.h` |
| `GridBounds` | 表示局部更新矩形区域，支持合法性、包含和面积检查 | 同上 |
| `IncrementalGraphStats` | 保存图更新、回退、合并和耗时统计 | 同上 |
| `FrontierNode` | 保存新自由区域代表点、版本和扩展状态 | 同上 |
| `gvg::GraphNode` | EGVG 节点、邻居和节点间栅格路径 | `dynamicvoronoi/include/dvr/GVG.h` |
| `TopoPath` | 历史拓扑路径及长度、风险、断点、状态和验证版本 | `TGH_Planner/path_searching/include/path_searching/topo_prm.h` |
| `RiskEdge` / `RiskPathCost` | 边级与路径级长度、风险、曲率和总成本 | `TGH_Planner/path_searching/include/path_searching/risk_aware_edge.h` |
| `ReliabilityMetrics` | 平均风险、净空风险、风险方差和 PRS | `TGH_Planner/path_searching/include/path_reliability/PathReliabilityEvaluator.h` |

没有单独新增 `GraphNodeState` 或 `GraphEdgeState`。当前实现通过脏区与图几何相交计算本周期受影响节点和边，再用局部替换事务更新图，避免与已有 `gvg::GraphNode` 重复建模。

## 5. 主要模块与源码索引

### 5.1 地图与 ESDF

- 地图入口与二维缓冲：`TGH_Planner/plan_env/include/plan_env/sdf_map.h`
- 传感器融合、局部更新边界和 ESDF 更新：`TGH_Planner/plan_env/src/sdf_map.cpp`
- EDT 查询封装：`TGH_Planner/plan_env/include/plan_env/edt_environment.h`

### 5.2 EGVG 与增量拓扑图

- Dynamic Voronoi：`dynamicvoronoi/include/dvr/dynamicvoronoi.h`
- EGVG 构建、区域构建和提交：`dynamicvoronoi/include/dvr/GVG.h`
- 地图变化检测和全局图持有者：`dynamicvoronoi/src/voronoi_layer.cpp`
- 增量管理接口：`dynamicvoronoi/include/dvr/incremental_topo_graph_manager.h`
- 局部补丁、拼接、验证和回退：`dynamicvoronoi/src/incremental_topo_graph_manager.cpp`

### 5.3 TGH、HEC 与历史路径

- 路径及容器定义：`TGH_Planner/path_searching/include/path_searching/topo_prm.h`
- 候选生成、HEC、历史复用、局部修复和 Guide Path：`TGH_Planner/path_searching/src/topo_prm.cpp`

### 5.4 风险与可靠性

- 二维风险地图：`TGH_Planner/plan_env/src/risk_map_manager.cpp`
- 路径边风险：`TGH_Planner/path_searching/src/risk_aware_edge.cpp`
- 候选排序：`TGH_Planner/path_searching/src/risk_aware_path_selector.cpp`
- PRS：`TGH_Planner/path_searching/src/path_reliability/PathReliabilityEvaluator.cpp`
- 参数文件：`TGH_Planner/plan_env/config/risk_map.yaml`

### 5.5 重规划与局部轨迹

- FSM 和重规划触发：`TGH_Planner/plan_manage/src/kino_replan_fsm.cpp`
- 总体规划调度：`TGH_Planner/plan_manage/src/planner_manager.cpp`
- Kinodynamic A*：`TGH_Planner/path_searching/src/kinodynamic_astar.cpp`
- B-spline 优化器：`TGH_Planner/bspline_opt/src/bspline_optimizer.cpp`
- 非均匀 B-spline：`TGH_Planner/bspline/src/non_uniform_bspline.cpp`

## 6. 重规划过程

### 6.1 地图更新阶段

实时点云或深度数据进入 `SDFMap` 后，系统完成 raycast、占据概率融合、局部膨胀和 ESDF 更新。`SDFMap` 把占据数组及本次局部更新边界传给 `VoronoiLayer::update_by_occupancy_map()`。

若没有占据变化也没有已知区域扩展，EGVG 不重建，只发布已有图；否则生成新的 `MapChangeSet` 并调用增量图管理器。

### 6.2 拓扑图更新阶段

首次规划执行全图构建。后续小范围变化执行区域构图和合并；局部结果不满足边界一致性或图一致性时执行安全全量回退。更新后的 `gvg_` 始终作为长期全局图继续使用。

### 6.3 历史候选预处理阶段

`TopologyPRM::preprocess()` 获取自上次处理版本以来的合并脏区：

1. 未与脏区相交的路径标记 `VALID` 并直接复用；
2. 相交路径标记 `AFFECTED`；
3. 只检查相交片段的 ESDF；
4. 碰撞路径标记 `INVALID` 并提取安全前后段；
5. 尝试局部重连；
6. 仅对几何变化路径重新 shortcut、更新成本并执行 HEC。

### 6.4 新路径生成与融合阶段

`SDFMap::voro_plan()` 在当前持久化 EGVG 上生成拓扑路径。新路径经过 HEC 去重后加入现有 front/back 容器，再根据成本限制、容器容量和排序规则保留候选。

### 6.5 Guide Path 与局部轨迹阶段

候选经风险/PRS 排序后生成 Guide Path。Kinodynamic A* 根据车辆当前状态和 Guide Path 搜索初始轨迹，B-spline 优化器再生成连续、平滑且满足安全距离及动力学约束的局部轨迹。

## 7. 关键 ROS 参数

### 7.1 增量拓扑图参数

配置位置：`TGH_Planner/plan_manage/launch/kino_algorithm.xml`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `incremental_topo/update_radius` | `2.0 m` | 判断脏节点、脏边以及扩张 ROI 的影响半径 |
| `incremental_topo/map_change_history_size` | `64` | 保留的地图变化版本数量 |
| `incremental_topo/patch_halo` | `1.0 m` | 局部 EGVG 构建区外围缓冲 |
| `incremental_topo/max_patch_expansions` | `3` | 边界不一致时允许扩大 ROI 的最大次数 |
| `incremental_topo/max_patch_fraction` | `0.35` | ROI 相对整图的最大面积比例，超过后全量回退 |

`VoronoiLayer` 会根据 `voro_map_resolution` 把米制半径转换为栅格数量。

### 7.2 风险地图与可靠性参数

配置位置：`TGH_Planner/plan_env/config/risk_map.yaml`，由 `kino_algorithm.xml` 加载。

| 参数组 | 主要参数 | 作用 |
|---|---|---|
| `risk_map` | `risk_enable`、三个 `lambda`、`robot_width` | 生成二维栅格风险 |
| `risk_aware_graph` | `alpha`、`beta`、`gamma` | 路径长度、风险、转向代价权重 |
| `risk_aware_path_selector` | `w1`、`w2`、阈值和动态增益 | 候选路径最终排序 |
| `path_reliability` | `enable`、三个 `lambda`、`alpha/beta/gamma` | PRS 计算及融合 |

风险模块可以通过 `risk_map/risk_enable=false` 关闭；PRS 可以通过 `path_reliability/enable=false` 关闭。关闭后仍保留原 TGH、HEC、历史路径和 B-spline 主流程。

## 8. 重要 ROS 接口

| 接口 | 类型/用途 |
|---|---|
| `/sdf_map/occupancy_2D` | 二维占据地图可视化与地图保存输入 |
| `/risk_map/risk_2D` | 二维风险地图 |
| `/voronoi/gvg_markers` | EGVG 节点和边可视化 |
| `/voronoi/path_topo` | Voronoi 拓扑候选路径 |
| `/TopoPlan/guide_path` | 最终拓扑 Guide Path |
| `/planning/bspline` | 优化后的 B-spline 轨迹 |
| `/planning/replan` | 重规划通知 |
| `/move_base_simple/goal` | RViz 目标点输入 |

## 9. 启动方式

编译并加载工作空间：

```bash
cd /home/hyh/mycar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

启动四轮车仿真、TGH 和 B-spline 局部规划：

```bash
roslaunch plan_manage kino_replan.launch \
  B_Spline_LocalPlanner:=true \
  sim_pose:=true
```

在 RViz 中使用 `2D Nav Goal` 设置目标。系统会发布 EGVG、拓扑候选、Guide Path 和 B-spline 轨迹。

## 10. 如何验证创新模块

### 10.1 验证增量图更新

将 ROS 日志级别设为 Debug，观察 `[IncrementalTopo]` 日志。局部障碍变化时应满足：

- `graph_updated_nodes < graph_total_nodes`；
- `local_repair_count` 增加；
- `local_roi_cells` 明显小于全图栅格数量；
- `full_rebuild_count` 不在每次地图更新时增加；
- `fallback_reason` 通常为空。

首次构图时 `full_rebuild_count` 增加是正常现象。大范围地图改变、地图尺寸改变或边界拼接失败时发生全量回退也属于预期安全行为。

### 10.2 验证历史路径复用

在机器人路线之外改变障碍物，观察：

- `reused_history_paths` 大于零；
- `invalidated_history_paths` 不增加；
- `HEC_check_count` 小于对全部候选两两检查的理论次数。

随后在某条历史路径上加入障碍，应看到对应路径变为 `AFFECTED/INVALID`，失效数量增加并触发局部重连，而其他路径继续复用。

### 10.3 验证风险与可靠性选择

设置一条较短但狭窄的路径和一条稍长但开阔的路径，观察 `[RiskPathSelector]` 日志中的：

- `length`；
- `risk`；
- `normalized_length`；
- `normalized_risk`；
- `prs_score`；
- `average_corridor_width`；
- `orientation_error`。

通过分别关闭 `risk_map/risk_enable` 和 `path_reliability/enable` 进行消融实验，可以比较纯长度、长度加风险、长度加风险加 PRS 三种路径选择结果。

### 10.4 推荐统计指标

- `graph_total_nodes`
- `graph_updated_nodes`
- `graph_updated_edges`
- `local_roi_cells`
- `patch_nodes`
- `merged_nodes`
- `deduplicated_nodes`
- `reused_history_paths`
- `invalidated_history_paths`
- `HEC_check_count`
- `local_repair_count`
- `frontier_expansion_count`
- `graph_validation_fail_count`
- `full_rebuild_count`
- `incremental_update_time`
- `total_replanning_time`

建议实验同时报告均值、标准差、P95 和最大值，并按静态地图、小范围动态变化、大范围变化三类场景分别统计。

## 11. 新旧流程对比

| 环节 | 原始流程 | 当前流程 |
|---|---|---|
| 地图变化 | 使用最新占据/ESDF | 增加 Delta M、版本号和有界变化历史 |
| EGVG 更新 | 地图改变后全图构建 | 小范围区域构建与全局合并，失败时安全全量回退 |
| 新自由空间 | 随全图构建被动纳入 | Frontier 状态记录并驱动局部扩展 |
| 历史路径 | 容器复用，但预处理范围较大 | 按地图版本和脏区分为 VALID/AFFECTED/INVALID |
| 碰撞验证 | 对历史路径完整检查 | 未相交路径跳过，相交路径只检查脏区片段 |
| HEC | 候选间广泛重复比较 | 对新增、修复和受影响路径选择性执行 |
| 路径排序 | 主要偏向较短路径 | 可融合长度、二维风险、PRS 和初始航向 |
| 局部轨迹 | TGH Guide Path + B-spline | 保持原接口和优化结构不变 |

## 12. 设计边界与注意事项

1. 当前系统是二维导航方案，不应把二维风险地图描述为地形风险图。
2. Frontier 是增量 EGVG 管理中的边界状态，不是完整的探索任务分配器。
3. 局部补丁无法安全拼接时会全量重建，因此不能宣称任何地图变化都只需常数时间更新。
4. HEC 仍使用原 `sameTopoPath()` 判据；选择性策略减少调用次数，但不改变同伦定义。
5. 历史路径修复使用已有 `Astar2D` 连接断裂区间，它不是新的全局规划器。
6. 风险和 PRS 是可配置扩展。进行性能比较时应记录配置文件和权重，否则不同实验不可直接比较。
7. 当前变化区域以轴对齐包围盒合并。多个相距很远的变化同时发生时，合并 ROI 可能偏大并触发全量回退，这是安全但保守的行为。
8. “创新点”应按来源区分：TGH、EGVG 与原历史路径机制属于上游项目核心；增量图管理、局部历史验证、选择性 HEC、二维风险和 PRS 是当前工作区中的扩展。用于论文、专利或成果申报时，应结合提交历史、作者贡献和引用要求准确表述。

## 13. 可用于项目摘要的表述

本项目面向四轮非完整约束移动机器人在二维未知环境中的在线导航，构建了 Occupancy/ESDF、EGVG 拓扑表达、TGH 多同伦候选管理、风险可靠性评价和 B-spline 局部优化组成的分层规划系统。在保持原 TGH 路径生成与 HEC 判据不变的基础上，系统通过地图版本和局部变化区域识别受影响的拓扑结构，采用带边界一致性验证的区域 EGVG 构建与事务式子图合并维护长期全局图，并在无法保证一致性时安全回退到全图重建。历史候选路径按照 VALID、AFFECTED 和 INVALID 分级，仅对受变化影响的片段执行碰撞检测与局部修复，并对新增或几何改变的路径选择性执行 HEC。最终候选可结合长度、二维障碍风险、通道宽度、未知区域、路径可靠性和车辆初始航向生成 Guide Path，再由 Kinodynamic A* 与两阶段 B-spline 优化生成可执行轨迹。该方案的重点是减少重复构图、重复碰撞检测和重复 HEC，同时保持原有拓扑规划与局部轨迹接口兼容。

