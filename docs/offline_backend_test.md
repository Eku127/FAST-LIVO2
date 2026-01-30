# 离线 Backend (PGO + 回环) 测试说明

## 前置条件

1. **已安装 GTSAM**（否则 offline 仍可运行，但无 PGO/回环）
   ```bash
   sudo apt install libgtsam-dev
   ```
2. **已成功编译**（含 backend 时会有 "Building backend PGO library"）
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

## 运行后输出（Backend 开启时）

在输出目录下会有：

| 文件 | 说明 |
|------|------|
| `poses.txt` | 全轨迹，TUM 格式 |
| `keyframes.txt` | 关键帧位姿（**已做 PGO 时为优化后**） |
| `keyframes/*.pcd` | 各关键帧点云 |
| `map.pcd` | 全局地图（用优化后的关键帧位姿拼接） |
| **`pose_graph.g2o`** | Pose graph（顶点+边），可用于离线分析或外部工具 |
| **`loop_constraints.txt`** | 检测到的回环对 `target_id source_id`（有回环时才会生成） |

终端里若检测到回环，会看到类似：

```text
[OfflineLIVMapper] Loop closure: 12 <-> 45 score=0.08
```

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

- 输出目录里会有 **`pose_graph.g2o`**（仅含 odometry 边，无 loop 边）。
- `keyframes.txt` 与 `map.pcd` 会按**每帧优化后的位姿**更新（与不开 backend 的轨迹可能略有不同）。

因此用任意一段 bag + `config/mid360.yaml` 跑一遍，检查是否生成 `pose_graph.g2o`，即可确认 backend 已执行。
