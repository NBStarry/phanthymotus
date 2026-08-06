# G1 Nav2-only MVP

本目录是 `navigation2` 的 ROS 2 Humble companion。首版使用 SLAM Toolbox +
AMCL + NavFn + DWB，不依赖 FAST-LIVO2 或 EGO-Planner；物理执行仍归 Driver。

## 运行链路

```text
/ubuntu/loco/state (unitree.g1.loco_state.legacy)
  → g1_loco_odom_bridge
  → /ubuntu/navigation/nav2/odom + odom→base_link

/ubuntu/lidar/cloud (unitree.g1.pointcloud.legacy)
  → g1_canvas_pointcloud_bridge
  → /ubuntu/navigation/nav2/cloud
  → pointcloud_to_laserscan
  → /ubuntu/navigation/nav2/scan

mapping:      SLAM Toolbox → /map + map→odom
localization: Map Server + AMCL → /map + map→odom

/ubuntu/navigation/nav2/command
  → NavigateToPose → NavFn → DWB → velocity smoother
  → cmd_vel_shadow → proposal wrapper
  → /ubuntu/navigation/nav2/velocity_proposal
```

容器 PID 1 是 `g1_nav2_runtime_supervisor`，它只管理上述 ROS 子运行时的启停。
`start_mapping` 和 `stop_mapping` 通过私有
`/ubuntu/navigation/nav2/runtime_switch` 请求在 mapping/localization 之间切换。
supervisor 不发布速度、不连 Driver，切换失败时尝试恢复前一 ROS 运行时。

根 `/cmd_vel` 必须不存在。`cmd_vel_shadow` 只允许 companion 内部 proposal wrapper 订阅；
Driver 只消费 `phanthy.navigation.velocity_proposal.v1`。

## 当前 Driver 输入

Driver main 的现行 payload 不带源时间戳和 frame，adapter 以可审计方式兼容：

- loco JSON 使用 adapter ROS 接收时间，标记
  `timestamp_source=adapter_receive`、`frame_source=adapter_contract`；
- pointcloud 解码 `<II>` header，使用 adapter ROS 接收时间和显式
  `legacy_frame_id=livox_frame`；
- Driver 点云和 adapter 输出都使用 ROS sensor-data QoS（BEST_EFFORT），与
  MID360 高频数据发布契约一致；
- 两路数据均以 500 ms 接收新鲜度 fail closed；
- 带源时间戳的 v2 loco/PCV2 仍可解码，但不是当前 Driver 必需项。

legacy frame 只能在已知发布器是 G1 MID360 时配置，不从不受信的 payload 猜测。

## 外参

`NAV2_LIDAR_X/Y/Z/ROLL/PITCH/YAW` 六项已从 Driver main `cfb8efe` 的
`g1_model.urdf` 冻结为 pelvis / zero-waist 到 MID360 的名义外参。
owner 升级脚本会在 Docker/SSH 写操作前拒绝缺失、非数值或无来源标记的外参。
离线 smoke 使用六个零构建合成环境，不会进入真机 owner 路径。

## readiness 与停止

- runtime readiness：新鲜 odom/scan、controller/planner/BT navigator/velocity smoother active、
  NavigateToPose action server 可用。
- navigation readiness：runtime 通过后，再要求 `/map` 和 `map → base_link` TF。
- `pause_nav`、`stop_nav` 和 stall timeout 必须等 Nav2 cancel result 进入终态。
- 物理安全的最后确认由 Driver `loco.stop` 返回 `stop_confirmed=true`。

## 构建和升级

```bash
NAV2_LIDAR_X=0 NAV2_LIDAR_Y=0 NAV2_LIDAR_Z=0 \
NAV2_LIDAR_ROLL=0 NAV2_LIDAR_PITCH=0 NAV2_LIDAR_YAW=0 \
docker compose --env-file source-lock.env \
  -f compose.nav2-shadow.yml build nav2-shadow
```

Dockerfile 固定 ROS base digest、7 个直接 apt 包版本、Ubuntu ports 和 ROS 2 清华源，
并在镜像内保存 `/opt/g1-nav2-package-lock.txt`。依赖与许可证见
[THIRD_PARTY.md](THIRD_PARTY.md)。

当前 source lock 固定为 `phanthy-nav2:g1-humble-nav2card5`。后续更新原地
刷新该标签，部署脚本按镜像 ID 判断是否替换，并只保留一个固定
rollback 容器。当前线上 `velocity_proposal` 已与 Driver 的
`schema/frame/nav_status` 契约对齐；G1 是否已部署仍必须以当次真机证据为准。

card3/card4 首次升级或 card5 原地刷新的 owner 入口：

```bash
I_AM_G1_OWNER=1 ./scripts/owner-upgrade-driver-inputs.sh
```

脚本先用 `driver_input_contract_probe.py` 自动识别并精确校验现行 legacy 或未来 v2
payload，保留升级前的 card3/card4 rollback。

## 本地验证

```bash
./scripts/test.sh
./scripts/smoke-test.sh
```

当前 Nav2 测试 65 项，覆盖 legacy/v2 解码、10Hz readiness receipt、
地图保存有界重试、N3 地图、N5 proposal、owner
授权/回滚约束与 Agent Core 框架验收入口。

## G1 框架验收

```bash
./scripts/loco-integration-readiness.sh

STAGE=preflight MAP_NAME=g1-n3-acceptance \
  ./scripts/owner-loco-card-acceptance.sh

I_AM_G1_OWNER=1 I_HAVE_G1_REMOTE=1 \
  STAGE=move MAP_NAME=g1-n3-acceptance \
  ./scripts/owner-loco-card-acceptance.sh
```

`owner-loco-card-acceptance.sh` 在 Nav2 容器中观测 costmap/TF/proposal，但所有
`navigation2` 调用都通过 Agent Core HTTPS API，因此会真正验证画布三条
连线、可信 `nav_id` 绑定、Driver 执行与停车释放。

项目启动后的 `loco` 卡片处于可信 standby：Driver 已注册且保持
`connected=false`、`armed=false`，ROS proposal 订阅尚未创建。只有一次导航动作
取得 Agent Core 签发的 `nav_id` lease 后，Core 才调用 Driver `start` 建立订阅并
武装；动作结束后必须停车确认、释放 lease 并恢复 standby。

## 操作边界

- 不加 crontab；正式 Perception Compose 通过 `depends_on` 统一拉起
  `nav2` 与 `perception`，两者使用 `restart: unless-stopped`；
- 部署、容器切换、建图和运动都由机器人 owner 手动执行；
- FastDDS 固定 `ROS_DOMAIN_ID=42` + `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`；
- 输入、外参、地图/TF、画布连线或 Driver 停车确认任一缺失都 fail closed；
- FAST-LIVO2 和 EGO-Planner 不是本 MVP 的运行依赖。
