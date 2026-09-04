# Phanthy Motus 仿真开发环境

本目录用于在 wlcb-23 上运行隔离的 Phanthy Motus x86 仿真开发栈。它不连接真实机器人，也不复用机器上已有的 ROS Domain、端口或容器。

## P0 范围

- 锁定 Phanthy Motus、Driver、MuJoCo、Unitree MuJoCo 和 SDK2 的上游版本。
- 构建 x86 原生 ROS 2 Humble、Agent Core 和 Perception 基线镜像。
- 使用独立 Docker Compose project、bridge network、ROS Domain 83 和本机回环端口。
- Agent Core 与正式版一样挂载 Docker socket、安装 Docker CLI/Compose，并拥有可写
  Compose 目录；它可以管理本机其他容器，不是受限版 Core。
- 运行时清单将本项目的 Core、Perception、MuJoCo Driver 和 Gazebo Navigation
  显示在「部署服务 → 我的服务」，并随容器重建刷新实际镜像与状态。
- 以 G1 Orin NX 16 GB 为应用栈硬上限；A100 默认不暴露给应用栈。
- 验证 Core WebUI、Perception MCP 注册、资源限制与隔离属性。

P0 不声称具备机器人动作、地图、导航或真机验收能力。Simulated G1 Driver、确定性传感器、MuJoCo 和 Gazebo 分别在 P1、P2 及后续阶段加入。

P0 Core 不安装 `ffmpeg`，因此远程音频文件转码不在当前验收范围内。P1 的 `mic` 直接生成符合 ASR 契约的 PCM，不依赖 `ffmpeg`。

Agent Core、Perception 和仿真运行文件来自 `NBStarry/phanthymotus:sim` 的同一干净 HEAD。
该分支基于实时上游 `main`，保存不以合并进上游为目标的仿真集成修改。
wlcb-23 从同一 commit 原生构建 `linux/amd64`；构建保持官方 `uv lock && uv sync`
依赖流程，并验证 Feishu Channel SDK 导入。Feishu 代理和数据流详情 MCP 上下文
已是该分支内经测试的源码，构建时不再应用本地 patch。有启用的飞书 Channel 时，
P0 验收会等待其真实连接，不仅检查 SDK 导入。源码和依赖都在 wlcb-23
远端获取，禁止先下载到 Mac 再上传。

## P1 范围

P1 增加独立的 `Simulated G1 (Protocol)` Driver，用于在物理引擎之前验证 Phanthy Motus 的卡片、MCP、ROS2 DDS 和 WebUI 数据面：

- `mic`：PCM_S16_LE、16000 Hz、mono，每块 1024 bytes 的确定性音调。
- `camera_rgb`：640x360 JPEG，画面标注协议级位姿、序号和故障模式。
- `imu` / `battery` / `joints` / `model`：JSON 状态、G1 关节名和锁定 Driver 中的 G1 URDF。
- `loco_state` / `loco`：有界速度命令与可重复位姿积分闭环。
- `sim_control`：reset、pause/resume 和有界故障注入。

所有卡片都显式标注 `SIMULATION ONLY`。`loco` 的位姿变化是协议级状态机，不是双足动力学，不得当作 MuJoCo 步态或真机能力证据。P1 不伪造接触力：`loco_state` 会声明 `simulation_backend=protocol_only_no_physics`、`physical_telemetry.valid=false`，并返回 `foot_force=null`；上层不得据此判断平衡、电机负载、损坏或硬件安全。

## P2 范围

P2 将同一 Simulated G1 Driver 切换为锁定的 Unitree G1 29DoF MuJoCo 模型，保留 P1 的传感器与状态接口，但只暴露已经具备物理和语义验收证据的动作：

- 真实 MuJoCo 刚体、关节、IMU 与双脚接触力；默认是可重复的屈膝站姿。
- `gesture`：使用关节位置伺服依次抬起左臂、摆动手腕、平滑收臂；WebUI 骨架可直接观察动作语义。
- `sim_control`：可关闭虚拟稳定辅助并施加有界外力，验证跌倒检测与 reset 恢复。

P2 的接触和跌倒数据来自物理引擎，但当前站立依赖显式标注的 `virtual_base_pose_servo`，因此 P2 **不是自主平衡或双足步态证明**。P2 不注册 `loco` 卡片，直接调用也会失败；不能再把浮动基座拖动解释成“行走”。地图、TF、定位、规划、避障和导航任务属于后续 Gazebo 阶段。

## P3 范围

P3 新增独立的 `Simulated Navigation (Gazebo)` 服务，不把差速底盘伪装成 G1 双足行走：

- Gazebo Fortress headless 世界提供平面底盘、房间、静态障碍和 2D LiDAR。
- `ros_gz_bridge` 输出标准 `/scan`、`/odom`、`/tf`、`/clock`，Nav2 使用标准接口完成规划和避障。
- P3 基线使用 Gazebo DiffDrive 的理想轮式 `/odom`，并固定 `map→odom` 单位变换，以确定性验证导航控制闭环；验收同时对比 Gazebo 世界真值，防止车轮物理方向与里程计方向不一致。AMCL 激光定位和带噪定位属于后续独立验收，不能混入基础导航结果。
- `navigation_map` 卡片将 OccupancyGrid 和当前位姿转换为 Core 已支持的 `sensor/mapping` 可视化流。
- `navigation` 卡片提供 `navigate_to_pose` 和 `cancel`；地图外、未知或已占用目标必须明确拒绝。
- 导航成功终态将 `distance_remaining` 归零；用户取消使用 `state=canceled`、`cancel_reason=user_requested`，不会把 ROS2 `GoalStatus=5` 伪装成错误。

P3 与 P2 同时运行：P2 继续证明 G1 关节和物理动作，P3 只证明平面导航软件闭环。P3 不证明双足步态、自主平衡或真机导航。
Gazebo 的导航算法、卡片和数据面仍全部位于独立服务；Core 的仿真分支只增加由
`LOCAL_SERVICES_MANIFEST` 显式开启的通用本地服务发现与 Docker 生命周期展示，
没有把 Gazebo 业务逻辑写入 Core。

## P4 范围

P4 在同一 Gazebo Navigation 服务中增加 AMCL 激光定位：

- P3 的静态 `map→odom` 只在默认 `ground_truth` 模式存在；P4 由 AMCL 根据
  `/scan`、`/odom` 和静态地图发布 `map→odom`。
- `navigation_map` 使用 `/amcl_pose`，并显示定位生命周期、新鲜度、协方差及相对
  Gazebo `dynamic_pose` 真值的验收误差。Gazebo 真值只用于测试对照，不反馈给 AMCL 或导航位姿。
- AMCL 未激活或定位超过 2 秒未更新时，`localization_ready=false`，地图停止发布新
  位姿，导航目标返回 `navigation_not_ready`；重新激活 AMCL 后可以恢复。

P4 仍使用 Gazebo 的理想轮式里程计作为 AMCL 运动模型输入；确定性噪声、绑架恢复、
动态障碍、SLAM 和 G1 双足移动不属于本阶段。

## P5 范围

P5 验证 AMCL 在有界里程计漂移和位姿突变后的恢复能力：

- P5 overlay 将 `/odom` 的线位移放大 4%、角位移放大 3%，形成可重复且有界的
  `deterministic_scale` 漂移；P3/P4 默认仍使用理想里程计。
- `navigation` 增加 `relocalize`，复用 AMCL 原生全局重定位服务，并以 0.35 rad/s
  执行 18 秒有界原地扫描，让粒子滤波器获得多视角激光观测。重定位期间
  `localization_state=relocalizing`、`ready=false`，导航目标必须返回
  `navigation_not_ready`。
- 验收通过 Gazebo 原生 `set_pose` 将平面底盘移动到具有唯一障碍几何的未知位置，
  避免用不可观测的对称位姿冒充算法失败；收敛后再执行一次真实 Nav2 目标。
  Gazebo 真值只用于测量误差，不反馈定位结果。

P5 不声称自动检测机器人被搬动，也不模拟轮滑、打滑或时间相关随机噪声；动态障碍、
SLAM 和 G1 双足移动继续作为后续独立阶段。

### 为什么 P2 暂无行走卡片

已在 wlcb-23 远端下载并校验官方 `unitree_rl_lab` G1 29DoF 速度策略，版本和哈希记录在 `versions.lock.yaml`。直接 ONNX 探针能加载策略并保持零速站立，但在省略官方完整启动流程后发送速度命令会失稳。官方 sim2sim 流程还包含弹性吊带、`Passive → FixStand → Velocity` 状态转换和随后解除吊带；当前 Driver 尚未完整复现并通过这条链路。

在完成以下验收前，不把策略挂到 `loco`：从静止站立平滑进入策略、不同方向和速度下持续稳定、停止后恢复站立、跌倒与超时明确失败。策略文件存在不等于步态已经可用。

## 目录

```text
config/                         P0 运行配置
docker/                         x86 原生镜像定义
resource-profiles/              G1 机载资源约束
scripts/p0-remote.sh            wlcb-23 实际构建、启动与验收入口
scripts/p1-remote.sh            Sim Driver 构建、部署与跨容器验收入口
scripts/p2-remote.sh            MuJoCo 构建、回滚、验收与 WebUI 演示入口
scripts/p3-remote.sh            Gazebo Fortress + Nav2 构建、部署与验收入口
scripts/p4-remote.sh            在 P3 基线上启用 AMCL 并验证失效与恢复
scripts/p5-remote.sh            加入里程计漂移并验证全局重定位
scripts/render-local-services.py 从 Docker 实际状态生成「我的服务」运行时清单
sim-driver/                     协议级与 MuJoCo G1 backend、MCP server 和 ROS2 publishers
gazebo-nav/                     P3/P4/P5 世界、地图、Nav2/AMCL、MCP 与 ROS2 适配节点
compose.p0.yaml                 隔离 Compose 栈
compose.p1.yaml                 P1 Sim Driver Compose overlay
compose.p2.yaml                 P2 MuJoCo backend Compose overlay
compose.p3.yaml                 P3 Gazebo Navigation Compose overlay
compose.p4.yaml                 P4 AMCL localization overlay
compose.p5.yaml                 P5 deterministic odometry drift overlay
versions.lock.yaml              上游与仿真版本锁
```

远端工作区固定为主仓的 `sim` 分支检出：

```text
/mnt/data/hanzebei/projects/phanthymotus/
```

源码仓必须通过远端已验证 GitHub 镜像获取。模型、数据集、镜像归档、bag 和仿真资源等大文件必须在远端直接下载到 JuiceFS 或 Docker 本地卷，禁止先下载到 Mac 再上传。

wlcb-23 当前无法解析腾讯云 Docker Hub 镜像域名，且公司镜像仓的 blob CDN 被远端网络拒绝。P0 因此以机器上已存在且验证为 `amd64` 的 ROS Humble 镜像 `local/phanthy-motus/ros-base:humble-amd64-c124798-v3` 为构建基底，再用锁定的最新上游源码重建 `audio_msgs`。基础镜像 ID 和创建时间记录在 `versions.lock.yaml`，不得使用同名但 ID 不同的镜像冒充。

## 端口与隔离

| 服务 | 容器端口 | wlcb-23 监听 |
|---|---:|---:|
| Agent Core WebUI | 15678 | `127.0.0.1:16678` |
| Perception MCP | 15720 | `127.0.0.1:16720` |
| Perception WebSocket | 15721 | `127.0.0.1:16721` |
| Simulated G1 MCP | 15730 | `127.0.0.1:16730` |
| Gazebo Navigation MCP | 15731 | `127.0.0.1:16731` |

- Compose project：`phanthymotus-sim-p0`
- ROS Domain：`83`
- 网络：专用 bridge；禁止 host / ipc / pid 共享
- 设备：禁止 `/dev` 挂载、`privileged` 和 GPU 注入
- 容器管理：Agent Core 与正式版一样挂载 `/var/run/docker.sock`，因此技术上拥有
  Docker daemon 级权限；专用网络和资源限制不是权限隔离。WebUI 清单只列本项目的
  四个固定容器，但这不是 Docker 安全边界。
- 出网：需要代理时由运行者显式设置 `PHANTHY_SIM_RUNTIME_PROXY`；`localhost` 与 Compose 服务名保持直连。仓库不保存私网代理地址，每次部署须实时验证。

通过 SSH 隧道访问 WebUI：

```bash
ssh -N -L 16678:127.0.0.1:16678 wlcb-23
```

然后打开 `https://127.0.0.1:16678`。P0 使用自签名证书，浏览器首次访问需要确认。

## 远端执行

在 wlcb-23 上：

```bash
cd /mnt/data/hanzebei/projects
GIT_PROXY=http://<proxy-host>:<port>
git -c http.proxy="$GIT_PROXY" -c https.proxy="$GIT_PROXY" clone \
  --filter=blob:none --branch sim \
  https://ghfast.top/https://github.com/NBStarry/phanthymotus.git \
  phanthymotus

cd /mnt/data/hanzebei/projects/phanthymotus/simulation
bash scripts/p0-remote.sh lock-contract
bash scripts/p0-remote.sh preflight
bash scripts/p0-remote.sh build-core  # 只重建 Agent Core
bash scripts/p0-remote.sh deploy-core-and-verify  # 构建、只替换 Core、失败自动回滚
bash scripts/p0-remote.sh build
bash scripts/p0-remote.sh up
bash scripts/p0-remote.sh verify
```

仅停止本项目：

```bash
bash scripts/p0-remote.sh down
```

脚本只操作 `phanthymotus-sim-p0` 这一 Compose project，不得停止或重建 wlcb-23 上已有的 Phanthy Motus 和其他用户容器。

P1 构建和完整自动验收：

```bash
cd /mnt/data/hanzebei/projects/phanthymotus/simulation
bash scripts/p1-remote.sh deploy-and-verify
```

P1 验收会同时覆盖两条路径：一条通过 Agent Core 的 `/api/mcp/{mcp_id}/call` 调用 Sim Driver，另一条从 Agent Core 容器内直接执行 MCP 协议与 ROS2 topic 断言。实际校验项包括 Core 注册与中转调用、非零 PCM、完整 JPEG、JSON、G1 关节名、位姿变化、生命周期拒绝和断流故障注入。

P2 构建和完整自动验收：

```bash
cd /mnt/data/hanzebei/projects/phanthymotus/simulation
bash scripts/p2-remote.sh deploy-and-verify
```

P2 会在镜像内加载官方 G1 模型，并验证站立、29 个有效关节、IMU、双脚接触、语义挥手轨迹、未验收 `loco` 的拒绝、跌倒、reset 恢复、Core 注册和容器隔离；失败时自动恢复先前 Sim Driver 镜像。挥手验收会同时约束左肩抬高、左腕往复、右臂与腰的被动响应上限，以及全过程稳定状态，避免仅凭“任意关节发生变化”误判通过。

P3 ground-truth 导航、P4 AMCL 定位与 P5 重定位验收：

```bash
cd /mnt/data/hanzebei/projects/phanthymotus/simulation
bash scripts/p3-remote.sh deploy-and-verify
bash scripts/p4-remote.sh deploy-and-verify
bash scripts/p5-remote.sh deploy-and-verify
```

P4 会验证命令、里程计和 Gazebo 真实运动方向一致，随后验证 AMCL 位姿、协方差、相对 Gazebo 真值的定位误差、导航成功，以及停用 AMCL 后拒绝目标并在
重新激活后恢复；只替换 Gazebo Navigation 容器，不重建 P0–P2。

P5 额外验证确定性里程计漂移非零、Gazebo 位姿突变后 `relocalize` 的导航拒绝、AMCL
全局收敛，以及恢复后的真实导航成功；仍只替换 Gazebo Navigation 容器。

WebUI 打开 `Simulated G1 (MuJoCo)` 后，将 `joints` 拖到画布并查看数据流。保持页面打开，在 wlcb-23 的 `hzb_dev` 中执行以下命令，可确定性触发页面演示：

```bash
bash scripts/p2-remote.sh demo wave
bash scripts/p2-remote.sh demo fall
bash scripts/p2-remote.sh demo reset
bash scripts/p2-remote.sh demo stop
```

验收应看到机器人先抬起左臂，主要以左腕左右摆动，再平滑收回站姿；右臂和腰只能有小幅物理耦合，不能与左臂同步大幅摆动。`fall` 应使机器人受推后倒地，`reset` 应恢复站姿。`fall` 会有意留下跌倒状态，离开页面前必须依次执行 `demo reset` 和 `demo stop`。自动验收覆盖实际 MuJoCo 关节和稳定性；页面视觉仍需人工确认，不以 ROS/API 结果冒充页面已通过。

WebUI 视觉验收时，数据流详情和监控面板必须使用同一 Driver 的 G1 URDF，不得一个显示通用备用骨架、另一个显示真实模型。默认和 reset 后必须保持原点静止站立；P2 页面不应出现 `loco` 卡片。

如果 wlcb-23 的代理入口变化，只对本仿真栈临时覆盖，不修改共享机器的全局代理：

```bash
PHANTHY_SIM_RUNTIME_PROXY=http://<proxy-host>:<port> bash scripts/p1-remote.sh up
```

`p0-remote.sh verify` 会从 Agent Core 容器经运行时代理访问 `router.phanthy.com/v1/models`；无 key 返回 HTTP 401 代表 DNS、TCP、CONNECT 和 TLS 路径均已打通，不代表 LLM 凭证本身已验收。

## 验收边界

各阶段证据边界如下：

- P0：x86 基线、MCP 注册、WebUI 可达、DDS 运行时和资源隔离。
- P1：Simulated G1 的 MCP、ROS 数据面和 WebUI 卡片/renderer；没有物理意义。
- P2：MuJoCo 关节、IMU、接触、站立、可视关节动作、跌倒与复位；没有自主平衡和双足步态。
- P3：Gazebo 传感器、ground-truth 全局定位、规划、静态避障和导航任务闭环。
- P4：AMCL 激光全局定位、定位失效拒绝和恢复；尚未加入里程计噪声与动态障碍。
- P5：有界确定性里程计漂移、显式全局重定位和恢复后导航；尚未自动检测绑架或加入动态障碍。
- 真机：任何硬件、时序、安全链和实际机器人能力。
