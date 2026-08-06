# Navigation 2 Perception 卡片

Canvas 展示名为 **Navigation 2**。它是 Perception Bundle 中的单实例
`processor` 卡片；稳定工具标识为 `navigation2`，用于保存画布、MCP 调用
和可信导航租约。Draft 期间的 `general_navigation` 卡片会在下次 owner wire 时
原地迁移为 `navigation2`。`controlled_spatial` 是 14-action
业务规范，不是卡片名。

设计见 [通用导航整体方案](通用导航整体方案.html)，Nav2 运行资产见
[Nav2-only MVP](nav2/README.md)，执行边界见
[N5 安全执行门禁](nav2/N5安全执行门禁.md)。

## 产品链路

首版使用 SLAM Toolbox + AMCL + NavFn + DWB，不依赖 FAST-LIVO2 或 EGO-Planner。
画布上必须是下列两卡链路：

```text
Driver.loco_state ──┐
                     ├──> navigation2 ── velocity_proposal.v1 ──> Driver.loco ──> G1
Driver.lidar_cloud ─┘
goal_pose.v1 (可选) ──────────────────┘
```

`navigation2` 只产生结构化速度提案，不导入 Unitree SDK，不直接执行运动。
Agent Core 是可信控制面：每个导航任务开始前生成 `nav_id`，先把该 ID
绑定到画布上精确连接的 `loco` Driver，再下发目标。到达、停止、暂停或错误
后必须先获得 Driver 停车确认，才释放租约。

## 容器生命周期

Navigation 2 需要一个独立 Nav2 companion，但卡片进程不调用 Docker，也不挂载
宿主 Docker socket。Perception 镜像内的 `/deploy/service.yml` 同时声明
`perception` 与 `nav2` 两个 Compose service，且 `perception depends_on nav2`。
Agent Core 部署 Perception 时执行普通 `docker compose up perception`，Compose 会先
拉起 `phanthy-nav2-shadow`，再拉起 `embodied-perception`；两个服务都使用
`unless-stopped`，因此机器人重启时也遵循同一持久生命周期。

Canvas 项目的启动/停止只负责插件状态、导航任务和 Driver 租约，不负责创建或
销毁基础容器。这样避免卡片获得宿主容器控制权，也避免停止 Canvas 时误杀地图与
定位运行时。

上海 G1 的 owner 部署入口为：

```bash
bash perception/plugins/general_navigation/deploy/scripts/build.sh

PREFLIGHT_ONLY=1 G1_HOST=g1-sh-wifi \
  bash perception/plugins/general_navigation/deploy/scripts/owner-upgrade-g1.sh

I_AM_G1_OWNER=1 G1_HOST=g1-sh-wifi \
  bash perception/plugins/general_navigation/deploy/scripts/owner-upgrade-g1.sh
```

脚本在写入前要求 Canvas 已停止，并同时保留旧的 Perception 和 Nav2 容器
作为 rollback；不会替换 Driver 或 Agent Core。

## Driver 输入合同

当前 [Driver main](https://github.com/He2y/phanthymotus-driver/tree/main) 已发布的传感器
payload 仍是 legacy 格式；导航 adapter 显式兼容，不伪装为带源时间戳的 v2。

| port | topic | ROS 外层 | 当前 schema | 时间/frame 来源 |
|---|---|---|---|---|
| `loco_state` | `/ubuntu/loco/state` | `std_msgs/msg/String` | `unitree.g1.loco_state.legacy` | adapter 接收时间 / `odom_source` 合同 |
| `lidar` | `/ubuntu/lidar/cloud` | `std_msgs/msg/UInt8MultiArray` | `unitree.g1.pointcloud.legacy` | adapter 接收时间 / `livox_frame` 合同 |

- `loco_state` JSON 使用 Driver 现有 `position` / `velocity` / `yaw_speed` / `imu.rpy`。
- `lidar_cloud` 使用 little-endian `<uint32 point_step><uint32 point_count><raw points>`。
- adapter 严格检查字节长度、点数、point step、数值有限性和 500 ms 接收新鲜度。
- 若 Driver 后续发布 `phanthy.g1.loco_state.v2` / `phanthy.sensor.pointcloud.v2`，adapter
  会保留其源时间戳和 frame；这两个 v2 schema 不是当前上机前置条件。

## 外参与 TF

```text
map → odom → base_link → livox_frame
```

`NAV2_LIDAR_X/Y/Z/ROLL/PITCH/YAW` 已从 Driver main `cfb8efe` 的
`g1_model.urdf` 冻结为 pelvis / zero-waist 到 MID360 的名义外参：
`(-0.00368, 0.00003, 0.46018, 0, 0.04014257279586953, 0)`。owner 升级
会校验六项有限小数和来源标记；离线 smoke 中的零值仅是合成夹具。

## Driver 执行合同

`loco` actuator 公开以下唯一输入：

```json
{
  "port": "velocity_proposal",
  "topic": "/ubuntu/navigation/nav2/velocity_proposal",
  "format": "data/json",
  "ros_type": "std_msgs/msg/String",
  "schema": "phanthy.navigation.velocity_proposal.v1"
}
```

Agent Core 调用 Driver 私有 lifecycle：

- 任务开始：`loco.start(input_topic=<固定 topic>, expected_nav_id=<可信 ID>)`；
- standby：`loco.info` 必须为 `enabled=true, connected=false, armed=false`；
- 任务结束：`loco.stop` 必须返回 `connected=false, stop_confirmed=true, state=idle`。

Driver 对 proposal 执行 schema/frame/flag/速度/TTL、单调 sequence、精确 `nav_id`、
主运控与停车确认门禁。不再依赖不存在的 `x-navigation-execution` 或
`phanthy.navigation.execution_receipt.v1`。

## 导航卡片合同

公开 action 固定为 14 个：

```text
start_mapping / stop_mapping / tag_place / untag_place
list_tags / list_maps / delete_map / load_map
navigate_to_tag / navigate_to_pose / wait_navigation_done
pause_nav / resume_nav / stop_nav
```

`start_mapping(map_name)` 可在 localization 模式下直接调用：卡片会请求
Nav2 supervisor 切换到 mapping，等待建图运行时 ready，然后自动重试同一
action。`stop_mapping` 完成地图和 pose graph 持久化后，会自动切回
localization 并加载刚保存的地图；两个 action 都不直接调用 Driver。

卡片还有一个可选 `goal_pose` Topic 输入。连线后 Core 订阅上游的
`std_msgs/msg/String` JSON：

```json
{
  "schema": "phanthy.navigation.goal.v1",
  "goal_id": "room-a-001",
  "x": 1.2,
  "y": -0.8,
  "yaw": 0.0,
  "speed": 0.4,
  "mode": 0
}
```

`goal_id` 必须唯一；`x/y/yaw` 使用 `map` 坐标系。Core 把消息转成普通
`navigate_to_pose` MCP 调用，因此仍需经过同一套 Driver lease、停车确认和
终态零速门禁。未连线时 Core 不订阅默认 topic。

Canvas 对外只保留 `velocity_proposal` 输出端口。`status` / `odom` / `map` /
`plan` 仍在 ROS 2 图中供 Nav2 运行和调试，但不再作为 Canvas 卡片输出。

Nav2 command/status、Odometry/TF、PointCloud2/LaserScan、map/path 和
`velocity_proposal` 的细节见 [Nav2 README](nav2/README.md)。对外 proposal 带 `nav_id`、
递增 `sequence`、250 ms TTL；终态必须是零速。

## 当前状态

| 项目 | 状态 |
|---|---|
| N1 14-action、Bundle/Core 注册 | 代码完成 |
| N2 legacy Driver 输入 adapter | card5 ARM64 已构建，mapping/localization smoke 通过；待上机 |
| N3 建图、断电恢复、AMCL globalize、tag | 上海 G1 真机通过 |
| N4 Nav2 goal/path/cancel | 上海 G1 shadow 通过 |
| N5 Driver proposal gate | Driver main 已实现；Agent Core 编排与 ARM64 镜像 smoke 完成 |
| N6 画布闭环 | 待部署新 Driver/Nav2/Perception/Agent Core 后真机验收 |

代码级验证当前为 111 项：Agent Core 11、Agent Core 部署 5、
Perception 18、Perception 部署 14、Nav2 63。Driver main 上游另有 31 项通过。
四个 ARM64 候选镜像已在本机构建，Core / Perception / Nav2 容器 smoke 通过；
这仍不等于 N6 真机已通过。

## 本地验证

```bash
agent-core/.venv/bin/python -m unittest discover -s agent-core/tests -v
agent-core/.venv/bin/python -m unittest discover -s agent-core/deploy/tests -v

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=perception \
  agent-core/.venv/bin/python -m unittest discover \
  -s perception/plugins/general_navigation/tests -v

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=perception/plugins/general_navigation/nav2/g1_nav2 \
  agent-core/.venv/bin/python -m unittest discover \
  -s perception/plugins/general_navigation/nav2/tests -v

agent-core/.venv/bin/python -m unittest discover \
  -s perception/plugins/general_navigation/deploy/tests -v
```

依赖版本、许可证和清华镜像见 [THIRD_PARTY.md](nav2/THIRD_PARTY.md)。

## G1 Perception 容器调试

`embodied-perception` 的 Compose 根文件系统设为可写，便于在 G1 上通过
`docker exec` 做短时间容器内调试。这不增加 `privileged`、设备映射、宿主机
volume 或 Linux capability；容器仍保持 `cap_drop: ALL` 和
`no-new-privileges:true`。

容器内的修改只存在于当前容器可写层，`docker compose up` 重建容器后会丢失。
需要保留的修复必须回填源码、通过测试并重建镜像，不得把容器内临时文件当作
可交付产物。

## G1 画布验收

机器人写操作由 owner 手动执行：

```bash
STAGE=driver-preflight /private/tmp/g1-general-navigation-owner.sh
I_AM_G1_OWNER=1 STAGE=driver /private/tmp/g1-general-navigation-owner.sh

STAGE=core-preflight /private/tmp/g1-general-navigation-owner.sh
I_AM_G1_OWNER=1 STAGE=core /private/tmp/g1-general-navigation-owner.sh

I_AM_G1_OWNER=1 STAGE=nav2-start /private/tmp/g1-general-navigation-owner.sh
STAGE=nav2-preflight /private/tmp/g1-general-navigation-owner.sh
I_AM_G1_OWNER=1 STAGE=nav2 /private/tmp/g1-general-navigation-owner.sh

STAGE=perception-preflight /private/tmp/g1-general-navigation-owner.sh
I_AM_G1_OWNER=1 STAGE=perception /private/tmp/g1-general-navigation-owner.sh

STAGE=canvas-preflight /private/tmp/g1-general-navigation-owner.sh
I_AM_G1_OWNER=1 STAGE=canvas /private/tmp/g1-general-navigation-owner.sh
```

画布接线后由 owner 在 UI 手动启动项目，再运行：

```bash
STAGE=card-preflight MAP_NAME=g1-n3-acceptance \
  /private/tmp/g1-general-navigation-owner.sh

I_AM_G1_OWNER=1 I_HAVE_G1_REMOTE=1 \
  STAGE=move MAP_NAME=g1-n3-acceptance \
  /private/tmp/g1-general-navigation-owner.sh
```

`move` 必须通过 Agent Core 的画布调用链下发，不得直连 Perception 或 Driver。
通过标准是：两路真实输入、精确三条必需画布连线（可另连一条
`goal_pose`）、同一 `nav_id`、实测移动、Nav2
`arrived`、终态零速以及 Driver 停车确认全部成立。

## 命名约束

Perception Bundle 会按工具名第一个 `_` 拆 PREFIX。`navigation2` 不含下划线，
因此工具名和插件 PREFIX 可以使用同一稳定 ID：

```python
class GeneralNavigationPlugin:
    PREFIX = "navigation2"

    def get_tools(self):
        return [{"name": "navigation2", "type": "processor", ...}]
```

Bundle 对外仍为 `navigation2`。框架内部 `info/config/start/stop` 不计入 14 个
业务 action。
