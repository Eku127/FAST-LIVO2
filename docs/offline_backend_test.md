# 离线 Backend (PGO + 回环) 测试说明

## 前置条件

1. **已成功编译**（含 backend 时会有 "Building backend PGO library"）
   ```bash
   cd /home/invs/projects/livo2_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select fast_livo
   source install/setup.bash
   ```

## 运行离线建图（含 Backend）

```bash
# 方式一：ros2 run
ros2 run fast_livo run_livo2_offline <bag路径> <配置文件路径> [输出目录]

# 方式二：直接调用可执行文件
./install/fast_livo/lib/fast_livo/run_livo2_offline <bag路径> <配置文件路径> [输出目录]
```

**示例：**

```bash
cd /home/invs/projects/livo2_ws
source install/setup.bash

# 使用 mid360 配置（已默认开启 backend）
ros2 run fast_livo run_livo2_offline \
  /path/to/your.bag \
  src/FAST-LIVO2/config/mid360.yaml \
  ./output_offline
```

- **bag 路径**：ROS2 bag 文件或目录（mcap / sqlite3）。
- **配置文件**：使用 `config/mid360.yaml`（或 xt32、mid360_outdoor 等）即可，其中已包含 `backend.enabled: true`。
- **输出目录**：可选，默认 `./output`；结果会写在该目录下。

## 启用/关闭 Backend

在 YAML 里用 `backend.enabled` 控制：

```yaml
# config/mid360.yaml 中
backend:
  enabled: true   # 改为 false 则只跑 LIO，不做 PGO 和回环
  # ...
```

关闭后仍会正常建图，只是不会做 pose graph 优化和回环检测，也不会写 g2o / loop_constraints。

## PGO 噪声模型说明

参考 [FAST-LIO-SAM-SC-QN](https://github.com/engcang/FAST-LIO-SAM-SC-QN) 的 GTSAM 用法，PGO 使用以下噪声模型：

### 噪声参数配置（`backend.pgo`）

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `prior_rot_noise` | Prior 因子旋转方差 (rad²) | 1e-4 |
| `prior_trans_noise` | Prior 因子平移方差 (m²) | 1e-2 |
| `odom_rot_noise` | Odometry 因子旋转方差 (rad²) | 1e-4 |
| `odom_trans_noise` | Odometry 因子平移方差 (m²) | 1e-2 |
| `loop_noise_min` | 回环噪声下限（防止数值问题） | 1e-6 |

### 噪声构造方式

- **Prior 因子**：首帧位姿使用 6 维对角方差 `(rot, rot, rot, trans, trans, trans)`
- **Odometry 因子**：连续关键帧间相对位姿，同样 6 维对角方差
- **回环因子**：**使用 ICP fitness_score 作为方差**（分数越低越可信），即 `(score, score, score, score, score, score)`

### ISAM2 优化策略

- 每次有新关键帧时调用 `isam2_->update(graph, values)`
- **当本轮有回环约束时，额外调用 3 次 `isam2_->update()`** 以帮助收敛
- 参考：[LIO-SAM Issue #5](https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936)

## 运行后输出（Backend 开启时）

在输出目录下会有：

| 文件 | 说明 |
|------|------|
| `poses.txt` | 全轨迹，TUM 格式 |
| `keyframes.txt` | 关键帧位姿（**PGO 优化前**，纯里程计链） |
| `keyframe_opt.txt` | 关键帧位姿（**PGO 优化后**，有回环时与 keyframes.txt 会有明显差异） |
| `keyframes/*.pcd` | 各关键帧点云 |
| `map.pcd` | 全局地图（用优化后的关键帧位姿拼接） |
| **`pose_graph.g2o`** | Pose graph（顶点+边），可用于离线分析或外部工具 |
| **`loop_constraints.txt`** | 检测到的回环对 `target_id source_id`（有回环时才会生成） |

终端里若检测到回环，会看到类似：

```text
[OfflineLIVMapper] Loop closure: 12 <-> 45 score=0.08
```

程序结束时会打印 **Loop closures detected: N**。若 N=0，说明未检测到回环。

## 为什么优化前后位姿几乎没差别？

**原因：PGO 图里只有里程计边、没有回环边时，优化解就是当前轨迹本身。**

- **keyframes.txt**：按前端 LIO 的里程计链逐帧递推得到的位姿（优化前）。
- **keyframe_opt.txt**：PGO 优化后的位姿。
- 图结构：第一帧有一个 prior 边，相邻关键帧之间有 odometry 边。**只有检测到回环时才会加入 loop 边**。
- 若**从未检测到回环**，图中只有 prior + odometry，初始位姿（里程计链）已经满足这些约束，优化器几乎不会改动，所以 `keyframes.txt` 与 `keyframe_opt.txt` 几乎一致。
- 若**检测到回环**，loop 边会拉近“再次经过同一地点”的两帧，修正漂移，优化前后就会有明显差异。

因此：**要看到优化前后差异，必须使用会“回到曾经过的地方”的轨迹**（闭环或往返），并让回环检测成功（见下节参数与日志）。

## 如何更容易触发回环

回环检测依赖：**轨迹形成闭环** 且 **时间/空间满足配置**。建议：

1. 使用**有闭环**的 bag（例如绕一圈回到起点）。
2. 若一直没回环，可适当放宽参数（在 `config/mid360.yaml` 的 `backend.loop_closure` 下）：
   - `search_radius`: 调大（如 20.0），扩大搜索范围
   - `time_threshold`: 调小（如 30.0），允许更短时间间隔形成回环
   - `fitness_threshold`: 调大（如 0.25），放宽 ICP 接受条件

改完后重新运行同一 bag 即可验证。

## 仅验证 Backend 是否参与（无回环也可）

只要配置里 `backend.enabled: true` 且编译了 backend：

- 输出目录里会有 **`pose_graph.g2o`**（无回环时仅含 odometry 边，无 loop 边）。
- `keyframes.txt` 为优化前位姿，`keyframe_opt.txt` 为优化后位姿；无回环时两者几乎相同。`map.pcd` 使用优化后位姿拼接。

因此用任意一段 bag + `config/mid360.yaml` 跑一遍，检查是否生成 `pose_graph.g2o` 和结束时的 **Loop closures detected: N**，即可确认 backend 已执行；N=0 时优化前后差异小是正常现象。
