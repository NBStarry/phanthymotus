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

公共层镜像必须从 `4paradigm/phanthymotus` 的干净 `main` 检出构建；`sim`
分支只提供 AMD64 打包和仿真 Driver，不作为 Core、Perception、ActuCore 的
业务源码。以下命令复用现有仓库，并用 `sim-` 标签避免和 ARM64 正式发布镜像混淆：

```bash
export PLATFORM_SOURCE_DIR=/mnt/data/hanzebei/projects/phanthymotus-main
export PLATFORM_REVISION="$(git -C "$PLATFORM_SOURCE_DIR" rev-parse origin/main)"
export BUILD_PROXY=http://<proxy-host>:<port>
bash scripts/build-platform-images.sh build

# 先在当前执行用户下完成 docker login，再推送；脚本不读取或保存凭证。
docker login bj-warehouse.tencentcloudcr.com
bash scripts/build-platform-images.sh push
```

镜像格式为
`bj-warehouse.tencentcloudcr.com/phanthy-motus/{core,perception,actucore}:sim-main-<commit>-amd64`。
推送只发布镜像，不向 Resource Center 注册服务。

`compose.platform.yaml` 是不修改公共源码的部署定义：Perception/ActuCore 使用
`network_mode: service:agent-core`，上游的 `localhost` 注册地址无需改写；
Driver/Gazebo 继续位于原仿真网络。2026-09-15 已用它替代下述旧公共层部署。
必须显式提供三个已校验镜像和适配宿主架构的 `PERCEPTION_CONFIG`，不能把
Jetson TensorRT 引擎直接用于 x86。Core 重建时应同时重建这两个共享网络命名空间的服务。

`build-platform-images.sh` 构建后通过 `verify-platform-sources.py` 逐文件比较
镜像与指定 main commit；源码不一致或漏打包即失败。
修改打包内容时用 `PLATFORM_IMAGE_TAG` 指定新标签，避免覆盖已部署镜像。
2026-09-15 已用旧 ActuCore 镜像反向验证：门禁准确拒绝被修改的 `/work/main.py`。
同日 `sim-main-125db64fb860-amd64-native1` 三个候选镜像均通过源码一致性门禁
（Core 157、Perception 51、ActuCore 4 个文件）；Core 的 ffmpeg、Docker/Compose、
飞书 SDK 依赖检查也通过。native1 的 Perception 尚不含完整推理依赖，不能作为
业务卡片验收镜像。完整依赖 `sim-main-125db64fb860-amd64-native4` 已构建完成，
Perception 51 个源码文件一致、推理依赖导入通过。
CLIP 使用固定 revision，并配套 setuptools 75.8.0 / packaging 24.2，避免旧构建工具
生成 UNKNOWN 包或触发 `canonicalize_version(strip_trailing_zero)` 异常。

### wlcb-23 公共层切换记录（2026-09-15）

当前运行：Core/ActuCore 使用 `sim-main-125db64fb860-amd64-native1`，
Perception 使用 `sim-main-125db64fb860-amd64-native4`，统一 Compose 项目为
`phanthymotus-sim-public-ipc`（部署使用 `-p` 显式指定，保留旧项目的回滚容器）。
Core 为 `ipc: shareable`，Perception/ActuCore 同时共享 Core 的网络和 IPC，
避免 Fast DDS 将同网络命名空间识别为同主机后访问不到对端共享内存。
已验证三容器运行且零重启、ASR/TTS/VOP/Face 工具列表、
Perception/ActuCore 向 Core 注册成功，智能控制为关闭。回滚容器及数据库位于远端
`/mnt/data/hanzebei/projects/phanthymotus-sim/runtime/ipc-switch-20260915-220554/`；
该目录包含私有配置，不上传或提交。部署 Compose 和 CPU 配置暂位于远端
`/mnt/data/hanzebei/tmp/sim-native-deploy-20260915/`，配置仍为运行挂载，不能删除。
此次公共层切换没有替换 Driver/Gazebo；ASR 数据流 GUI 已验证，其他 GUI 和动作回归未完成。

统一部署入口（Driver 和 Perception 已迁入持久配置/素材目录）：
`/mnt/data/hanzebei/projects/phanthymotus-sim/runtime/public-platform/`。
其 `.env` 引用已测镜像及本目录内的 CPU 配置、校验过 SHA-256 的只读回放素材；
执行 `compose.platform.yaml + compose.p1.yaml + compose.p2.yaml + compose.fixtures.yaml`
的 `docker compose config --format json` 已确认只有四项服务、ROS domain 83、
共享 IPC、`MUJOCO_GL=disable` 和 loopback 端口。
Driver 已迁入 `phanthymotus-sim-public-ipc`，回放素材改为本目录 `fixtures/`；
旧容器及检查信息保留在 `driver-adoption-20260915-152123` 对应备份。
迁移时三个公共容器 ID 均未变；迁移后 ASR/VOP 再次输出预期中文、公交车检测结果，
测试实例已停止。Perception 的 CPU 配置已挂载本目录 `config/perception-cpu.yaml`，
镜像 ID、模型卷均不变；备份在 `perception-migration-20260915-153039`，
包含容器检查信息、日志和 Ultralytics 设置，并保留不受 Compose 管理的停止态回滚容器。
回滚仍依赖旧 tmp 配置，暂不能删除该路径。
迁移后四卡真实 ROS 回归全部通过：ASR 中文文本、VOP 公交车、人脸检测两张未知低质量脸、
TTS 96314 bytes 非静音 PCM 和 EOF；每项结束均停止实例。
Core/ActuCore 容器没有重建；若后续重建 Core，必须同时重建共享其网络/IPC 的服务。
禁止旧 RoboValue Compose 和 `--remove-orphans`。

原版 Perception 不提供 `/health`；健康探针须 POST `/mcp` 调用 JSON-RPC
`tools/list`，并检查 `asr/tts/vop/face_recognition`。404 不是崩溃证据。
本次错误 `/health` 探针曾触发回滚，修正后实际执行上述 MCP 探针和四卡推理通过。

四个镜像均已推送至腾讯云，完整摘要见 `versions.lock.yaml` 的
`public_applications` / `published_sim_driver`；机上 `.env` 已固定这些摘要，
并核对每个摘要与原运行镜像的 Image ID 一致。临时 Docker 认证目录已清除，
没有向 Resource Center 注册，也没有推送 Git 提交。Perception 已按摘要重建，
其余运行镜像 ID 与锁定摘要一致。

以下为本日较早的旧基线切换记录，镜像与 ActuCore 挂载现已被上述部署替代：

- Core、Perception、ActuCore 已切换到上述仓库的
  `sim-main-125db64fb860-amd64`，这是已发布的 main 基线，不代表当前最新 main。
- 保留现有 MuJoCo Driver、Gazebo Navigation、VOP、Robo-Value policy/station；
  公共 Perception 未启用业务插件，公共 ActuCore 为上游空插件配置。
  原 Robo-Value `policyobs` / 执行插件不随公共基线保留，不能据此声称 Robo-Value 链路可用。
- Core 数据和原容器检查信息保存在远端
  `/mnt/data/hanzebei/projects/phanthymotus-sim/runtime/public-switch-20260915-121720/`。
  该目录含私有配置，只允许机上使用，不上传或提交。
  三个旧容器追加 `-backup-public-switch-20260915-121720` 后缀，已停止并禁用自动重启。
- 新服务沿用原端口、网络及持久卷；ActuCore 通过该备份目录下的
  `actucore.yaml` 保持内部端口 15740。**该目录也是当前挂载来源，不可删除。**
- 已验证：公共层启动与 MCP 健康检查、Core 保持停止控制、P2 站立/29 关节与 IMU/
  语义挥手/跌倒检测/重置恢复。P2 仍依赖虚拟基座稳定辅助，不证明自主步态。
- Gazebo 在 AMCL + 确定性里程计漂移模式下通过目标到达、占用目标拒绝和取消回归，
  终态定位误差 0.1135 米；本轮没有重跑 P5 位姿突变与重定位扫描。
  验收后导航和地图发布已停止，三个公共容器均为 running、restart_count=0。
- 尚未完成：浏览器 GUI 回归（宿主 SSH 隧道断连），以及将本次运行态切换收敛到
  唯一部署 Compose。旧 Robo-Value Compose 不得直接重跑，否则会恢复旧公共层镜像；
  备份容器仍带原 Compose 标签，不要执行 `--remove-orphans`。

此前的旧 AMD64 镜像还包含桥接网络的 MCP 广播地址适配，Core 不含 `ffmpeg`
和上游 `tools/` 目录；因此镜像是 main 基线的仿真打包，不是正式镜像的等价复刻。
补齐打包差异并验证前，不声称公共层所有功能均已通过。

### 感知样本输入（ROS 链路及 ASR 数据流 GUI 已验证）

Driver 可通过 `compose.fixtures.yaml` 可选 overlay 挂载只读 `/fixtures`：
`SIM_FIXTURE_DIR` 指向远端素材目录，`SIM_AUDIO_FIXTURE=/fixtures/speech.wav`、
`SIM_IMAGE_FIXTURE=/fixtures/scene.jpg` 选择回放素材；留空保留原示意图/音调。
素材必须在远端准备并记录授权来源，不使用私人照片或真实语音作默认样本。

WAV 必须为 PCM_S16_LE/16000 Hz/mono，最长 300 秒；JPEG 至多 4096×4096 像素，
单文件至多 32 MiB，路径不得逃出挂载目录。音频按 32 ms 块循环，末块补零；
语音素材应自带句间静音，便于 VAD 分段。每次发布使用新的 ROS 时间戳。
`info.input_source` 报告回放标识、文件名和 SHA-256，不把回放冒充物理相机或麦克风。
这仅提供输入，不生成 ASR 文本、检测框等假结果，输出必须来自原版 Perception。
2026-09-15 在无网络候选 Driver 容器中，以 Ultralytics 自带 `bus.jpg` 验证了
两帧真实 ROS 发布：JPEG SHA-256 一致、时间戳递增、回放来源明确；测试容器已停止。
SenseVoice、Matcha、声码器、VAD 和 buffalo_sc 已准备到仿真模型卷，后者通过主仓
固定大小及 SHA-256 校验。native4 无网络 CPU 容器已实测原版适配器：
Matcha 合成后 SenseVoice 识别出“你好，这是机器人仿真测试，请识别前方的公交车。”；
VOP 在公开 bus.jpg 上识别公交车和行人，FaceAnalyzer 检测到 2 张脸。
这些是模型推理证据，不是 Driver→卡片→WebUI 端到端验收，也未验证人脸身份识别。
同日回放候选 Driver 已部署，保留 MuJoCo 后端，镜像为
`phanthymotus-sim/sim-driver:replay-candidate-20260915`。原容器及配置备份位于
`/mnt/data/hanzebei/projects/phanthymotus-sim/runtime/replay-switch-20260915-214212/`。
跨容器 ROS 验收已收到上述 ASR 文本，以及 VOP 的公交车（置信度 0.88）和行人结果；
TTS 卡片 `speak` 已发布至少 32000 bytes 非静音 ROS 音频；Face 卡片已输出 2 张人脸，
均正确标记为 `known=false / low_quality`，不代表身份识别正样本验收。
测试后 mic/camera_rgb 与 ASR/VOP/TTS/Face 测试实例全部停止。
GUI 部分验证：原 SSH 入口在握手前关闭；经已授权的 40022 容器作 TCP 跳板后，
Chrome 正常打开 WebUI。通过页面保存 ASR CPU/VAD 配置（关闭录音落盘），
拖入 mic `card-mu2ql1sf8r39` 与 ASR `card-mu2qkh9tk9sc` 并完成音频连线，
原有卡片/连线未改动，编辑权已释放。ASR 数据流窗口已实际显示
“你好，这是机器人仿真测试，请识别前方的公交车。”，并已截图检查。
此前空白时 WebSocket 明确返回 `Topic ... not registered`。测试应走 Core 的
`/api/mcp/{mcp_id}/call`，且需要核验输出注册，而不能只看直接 ROS 探针。
进一步排查确认：125db64 的 `mcp_call_tool` 在 remote_mic 分支内 `import asyncio`，
使整个函数的 `asyncio` 成为局部变量；外部工具调用后的 `asyncio.create_task`
因此发生未绑定异常并被吞掉。Python `symtable` 检查确认该局部作用域。
公共源码未改；验收使用现有 `/api/mcp/{mcp_id}/ping` 刷新能力，从正在运行的卡片
读取并注册 topic。这条原生刷新路径通过，不代表调用后的即时注册问题已修复。
VOP `card-mu2rdavvd36s`、Face `card-mu2rnx9ka0ii` 已通过页面添加并连到原相机；
原 GPU VOP 卡片保持不变。网页实际显示公交车置信度 0.88 和两张 `low_quality`
未知人脸结果，截图已检查；人脸配置为 CPU，没有登记任何身份。
TTS `card-mu2rzx0a96n0` 已配置 Matcha 并连在 ASR 后；在对应
`/phanthymotus_sim_g1/mic/audio/asr/tts` 上实测整句合成，收到 96202 bytes PCM
及 EOF。网页显示“本句结束”；真实鼠标点击后，原版 renderer 日志确认
AudioContext `running`、16 kHz、1600 samples 的首块实际排入播放。已暂停播放。
这不是扬声器听感验收；普通 DOM `.click()` 曾使上下文停留在 suspended，
自动化播放必须提供浏览器真实用户手势，不能只看按钮变成暂停图标。
该测试没有点击全画布“开启智能控制”，不能替代全画布启停验收。

2026-09-16 经用户授权完成一次临时验收画布的网页全局启停：先确认项目关闭、
独立 GPU VOP 无实例，备份原 8 张卡片，暂移除 decision_core 与独立 GPU VOP，
保留 mic→ASR→TTS、camera→公共 VOP/Face。点击“开启智能控制”后全部就绪，
ASR 页面连续显示完整中文，公共 VOP 页面显示公交车 0.88/行人，Face 页面显示
2 张 low_quality 未知人脸；TTS 自动接收 ASR 文本，日志记录 157314 bytes 合成。
TTS 页面显示音频流状态，真实鼠标点击播放，但截图无可见波形，未作听感验收。
点击“停止智能控制”后 project_running=false，6 张测试卡片均 idle；最近 5 分钟
日志未见 InvalidHandle/Traceback。原画布 JSON 精确恢复、编辑锁释放，GPU VOP
仍 idle 且无实例，未重启独立容器。此单次通过不证明此前启停竞态已修复，
也不覆盖 OCR、已知身份识别或 Decision Core 行为。
本地证据：`/private/tmp/sim-canvas-backup-20260916.json` 与
`/private/tmp/sim-canvas-{asr,vop,face,tts}-20260916.png`；ASR 截图被启动完成
弹窗遮挡，文本确认来自实时 DOM 与对应模型日志，不能把该截图当清晰文本证据。

可重复运行的 ROS 验收脚本为 `scripts/perception_acceptance.py`，已在部署的
Perception 容器内分别实跑 ASR、VOP、Face、TTS；各分支检查真实输出并清理本次
实例。要求仿真输入及卡片空闲，显式指定 MCP ID，防止误选其他同名服务。
将脚本放到远端 checkout 后，从仓库根目录执行（ID 应先在当前 WebUI 核对）：

```bash
docker exec -i phanthymotus-sim-p0-perception bash -c \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 - vop --driver-id mcp-1787543441 --perception-id mcp-1787541754' \
  < simulation/scripts/perception_acceptance.py
```

把 `vop` 换成 `asr`、`face_recognition` 或 `tts` 可验证对应卡片。脚本不证明
浏览器音频播放、人脸身份正样本或完整画布生命周期；这些仍须单独验收。
TTS 页面复测可追加 `--tts-input-topic /phanthymotus_sim_g1/mic/audio/asr --preview-delay 30`，
在 `TTS_PREVIEW_READY` 后打开同一数据流并点击播放；脚本等待完整 EOF，随后停止实例。

完整 Driver 已部署：
`phanthymotus-sim/sim-driver:full-7bdc319875b1-amd64`，镜像 ID
`sha256:45497be8403c403fbc360e35bfe4ea33a57f680692800097d581f5996d1c2f0f`。
使用仓库 `docker/sim-driver-p2.Dockerfile`，远端准备固定版本 G1 模型及哈希锁定
wheel，关闭构建网络；完整源文件 COPY 后通过 MuJoCo 站立/语义挥手/跌倒/重置
测试及 `test_media_fixture.py`。构建上下文在远端
`/mnt/data/hanzebei/tmp/sim-driver-full-20260915-r2`，不是公共应用源码补丁。
旧容器保留在 `runtime/full-driver-switch-20260915-150435` 对应备份中；
切换前确认智能控制关闭、所有仿真传感器停止。新镜像健康检查及四张公共卡片的
跨容器 ROS 回归通过：ASR 输出预期中文句子；VOP 检出公交车（0.88）；
Face 检出两张低质量、未知身份人脸（不代表身份识别通过）；TTS 输出 96194 bytes
PCM 和 EOF。测试后逐项停止实例，Driver 和公共三层均 running、restart_count=0。
这些是换镜像后的 ROS 回归；先前 GUI 证据不能替代本镜像的完整画布启停验收。

不要继承旧容器的 `MUJOCO_GL=egl`：当前镜像不提供 EGL，且 Driver 不调用
MuJoCo 图像渲染器；沿用 `compose.p2.yaml` 的 `MUJOCO_GL=disable`。
退出流程根据后端调用 `stop_gesture` 或 `stop_move`，避免清理时访问不存在的方法。
无网络启动/退出回归（在构建机器运行，8 秒超时码 124 是预期测试终止）：

```bash
image=phanthymotus-sim/sim-driver:full-7bdc319875b1-amd64
log=$(mktemp)
rc=0
docker run --rm --network none --memory 1g --cpus 1 -e MUJOCO_GL=disable \
  --entrypoint /bin/bash "$image" -c \
  'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && timeout 8 python3 /work/main.py' >"$log" 2>&1 || rc=$?
test "$rc" = 124 && grep -q 'backend=mujoco_g1_29dof' "$log" && ! grep -q Traceback "$log"
```

已发现但未修改的上游限制：125db64fb860 的 ASR 在热音频输入下启动时，
可能在 `_vad_proc.start()` 前进入 `_audio_cb`，把 `exitcode=None` 的未启动进程
误判为死亡，导致 `start` 返回 `idle`。本次先启动 ASR、再启动仿真 mic 的对照
通过 ROS 和 GUI；这只是隔离竞态的测试顺序，不代表热启动问题已修复。
后续完整启停回归必须保留这一失败项，不以 HTTP 200 或自动重试视为通过。
`scripts/perception_acceptance.py --producer-first` 可检查传感器先启动的真实输出，
不再仅观察卡片状态。2026-09-15 在当前部署复测 ASR：`start/info` 均为 `running`，
但 40 秒无预期文本，验收退出 1；日志显示 `perception_spin` 在线程的
`_take_subscription` 中因 `InvalidHandle: cannot use Destroyable because destruction was requested`
退出。它与既有 VAD 启动竞态是否同源尚未证实，不能合并归因。
测试 finally 已停止本次 ASR 和 mic；重启服务仅作恢复，不作为修复或验收通过。
恢复后消费者先启动的同脚本对照退出 0，收到完整“你好，这是机器人仿真测试，
请识别前方的公交车。”，最后 mic/asr 均为 idle；这证明服务恢复，不证明竞态消失。

2026-09-16 依赖层隔离复现：已安装 `ros-humble-rclpy 3.3.21`，
不加载应用/模型，仅重复创建销毁订阅，原版第 3 次循环退出 `InvalidHandle`。
官方 [#1150](https://github.com/ros2/rclpy/pull/1150) 处理执行与销毁竞态，
但 [Humble 回移请求 #1610](https://github.com/ros2/rclpy/issues/1610) 尚未关闭。
`docker/rclpy-humble.Dockerfile` 和对应 patch 是**本地诊断候选，不是正式验收镜像**：
构建时验证原 executor SHA-256，公共应用文件不变。v1 候选第一次真实 ROS 测试
100 次循环、990 条消息通过；补测第 50 次失败，带堆栈复测第 47 次失败，
新失败位置是 `_wait_for_ready_callbacks → waitable.add_to_wait_set → qos_event`，
不是已保护的 take 路径。当前安装实现已含 #1590 的 valid_waitables 过滤，
不能简单重复该补丁或吞掉整个 spin 异常。候选 51 个 Perception 应用文件与
`125db64fb860` 逐字节比较通过；业务回调抛出的 InvalidHandle 仍传播。
v1 未替换运行服务，也未跑卡片/GUI 回归。

v2 在 `waitable.add_to_wait_set` 补充限定的 InvalidHandle 保护，并将失败项
从后续 is_ready 检查列表移除。这是本地补丁，不声称官方已回移。
独立无网络容器两轮各 100 次循环通过，分别收到 992/975 条消息，执行线程
存活且 errors 为空；两轮均验证业务回调异常不被吞掉。v2 的 51 个公共应用
文件再次逐字节比对通过。

2026-09-16 v2 卡片回归：确认 project-running=false、无画布编辑锁及四张公共
Perception 卡片和两个仿真传感器 idle 后，仅以 Compose `--no-deps` 临时切换
Perception 到 `phanthymotus-sim/perception:rclpy-guard-candidate2`
（image ID `cbdedb4a6b3f4c2ab8b392d339a536b291aa6af39ed47758e1e7098456ff1047`）。
Core、ActuCore、Driver 和其他项目容器未重建；运行目录 `.env` 未改动，仍保留
原 native4 digest，因此普通 Compose 重部署会恢复原镜像。
同日后续按当前三个容器的实际 Image ID，再次运行
`scripts/verify-platform-sources.py` 对比上游 `125db64fb860`：Core 157 文件、
Perception 51 文件、ActuCore 4 文件全部 PASS。该检查覆盖镜像内业务文件，
不代表配置挂载与上游相同，也不把 rclpy 依赖修补算作“原版依赖”。
复测使用本地已有 Git 对象、远端 Docker 临时容器，无需向远端复制源码；
命令级 `DOCKER_HOST=ssh://<已授权容器入口>` 与 `DOCKER_API_VERSION=1.43`
用于匹配当前远端 Docker API，未修改全局 Git 信任或 Docker 配置。
`perception_acceptance.py` 经 Core 调用并订阅真实 ROS 输出：

- producer-first ASR 两次均输出完整测试句，正常 stop 后 mic/asr 为 idle；
- producer-first VOP 检出 bus（0.88）；
- producer-first face_recognition 检出两张低质量未知人脸，不代表已注册身份识别；
- TTS 产生 96322 bytes 非零 PCM 并收到结束标记，正常 stop；
- 本轮仍未验证候选镜像的 GUI 全画布、TTS 可见波形和长期稳定性，不能判整体完成。

同日 Chrome 单卡查看数据流复测：TTS 使用画布输出 topic，ROS 收到
96508 bytes 及结束标记，WebUI 标签显示「音频流 100ms/帧」及「本句结束」。
截图波形区域仍空白，但实测该后台标签页 `document.visibilityState=hidden`，
服务实际提供的 `audio.js` 在 `document.hidden` 时跳过绘图，因此不能将这张
后台截图判为 Core 绘图故障。原生窗口控制超时，尚未取得前台可见波形证据；
未修改 Core、未更改共享画布，测试 TTS 已停止，任务创建的标签页已关闭。

随后通过系统原生 AppleScript 激活本任务自建 Chrome 标签页（按临时唯一标题
选择，不操作其他页面），读回 `document.visibilityState=visible`。再次运行
`perception_acceptance.py tts --tts-input-topic /phanthymotus_sim_g1/mic/audio/asr
--preview-delay 15`，同次 ROS 收到 96416 bytes 和 EOF，页面截图明确显示非空
橙色波形及“本句结束”。截图 `/private/tmp/sim-tts-visible-20260916.png` 已人工式
视觉检查；TTS stop=idle，临时标签页关闭，画布及公共源码均未修改。
这补齐单卡前台波形证据，不代表已做人工听感或全部卡片联合启动验收。

测试输入仍是 speech.wav / bus.jpg 循环素材，不是三维场景虚拟传感器。
2026-09-16 已构建独立 `sim-driver:render-candidate1`（image ID 前缀
`3436b0ade403`），在原 Driver 上增加 Mesa 23.2.1 的 OSMesa CPU 无窗口渲染库。
`scripts/render_probe.py` 在无网络、无 GPU、1 GiB/2 CPU 临时容器中退出 0：
已知物体中心深度 2.600 m，物体位移后 33.58% 像素变化；官方 G1 模型挂载
躯干相机后仍为 29 个执行器，根朝向旋转 45° 后 51.94% 像素变化。
这是渲染依赖与模型挂载证据，尚未接入常驻 Driver 的 ROS 相机发布或 Perception。
复测命令：

```bash
docker run --rm -i --network none --memory 1g --cpus 2 \
  --cap-drop ALL --security-opt no-new-privileges --entrypoint python3 \
  phanthymotus-sim/sim-driver:render-candidate1 - \
  --g1-model /work/resource/mujoco/g1/scene_29dof.xml \
 < simulation/scripts/render_probe.py
```

同日 `render-camera-candidate2`（image ID 前缀 `547252866dc3`）接入可选
`SIM_CAMERA_MODE=mujoco`，JPEG 沿用 `camera_rgb` topic，320×240、垂直视角 65°。
独立线程持有 GL 上下文，读取同一物理状态，渲染失败返回 error/last_error，
不降级成静态图片；`input_source=mujoco_render` 且 `calibrated=false` 明示近似相机。
`scripts/camera_ros_probe.py` 在无网络、无 GPU、ROS Domain 95 的临时容器中通过：
收到 7 帧，转向后 46.34% 像素变化，时间戳递增，stop 后停止发布，drop_camera
断流及恢复通过，注入渲染故障后 info=error、再次 start 拒绝。
日志中的 `injected render failure` 是该负向测试预期结果。

`compose.world-camera.yaml` 放在 p2/fixtures overlay 之后，指定
`SIM_CAMERA_DRIVER_IMAGE` 为已验收渲染镜像即可显式切换；它清空图片 fixture，
不改变音频配置。

2026-09-16 `render-room-candidate3`（image ID
`01145ad37acc5bcbfababbf1b723dd43ce47608c6e465196441ca5bbc02e3eed`）
增加自建有碰撞几何的桌、椅和背景墙，不使用照片贴板。隔离 ROS 探针收到 7 帧，
转向后 84.48% 像素变化，停止、断流恢复和渲染失败检查均通过。
空闲检查通过后，仅替换常驻 Sim Driver；公共三项服务未重建，原 `.env` 未改。
通过 Core 启动相机后再启动原版 VOP，`perception_acceptance.py vop
--producer-first --expected-object chair` 得到真实 ROS 输出 `chair`、置信度 0.86。
Chrome 中点击既有相机卡片“查看数据流”，实际显示桌椅房间、320×240 JPEG、
约 2 fps；截图为本机 `/private/tmp/sim-room-camera-webui-20260916.png`。
测试结束相机和测试 VOP 均 idle，未改变共享画布，测试标签页已关闭。
这证明场景相机到卡片及网页预览链路，不代表摄影级拟真、识别准确率或完整导航验收。
音频仍为 fixture；深度/雷达与此场景尚未统一，人脸正样本及 OCR 等门禁仍未完成。

随后已构建并仅切换 Driver 到 `render-speech-candidate4`（image ID 前缀
`860923fade4a`），保留三维相机，同时配置 `SIM_AUDIO_SILENCE_MS=5000`。
此参数接受 0–60000 的整数毫秒；0 保留旧的无间隔回放，fixture overlay 默认
5000。每次启动先输出该静音段，再播放 WAV，每轮重复；末块补零会额外增加
不足 32 ms 的间隔，素材自身的静音另计。`input_source.silence_ms` 可读回，
仍明确标为 `fixture_replay`，不是声学传播模拟，也不是模型生成的识别结果。
现有单测覆盖静音、重播、重启、越界值；Mac 默认 Python 缺 Pillow，未在本机
跑通，改用候选镜像的无网络临时容器执行同一测试，1 test / rc=0。
部署后 producer-first ASR 返回完整的“你好，这是机器人仿真测试，请识别前方的
公交车。”，正常 stop。直接订阅 ROS 的独立探针收到初始静音 190 块、非零音频
145 块、随后连续静音 100 块，每块 1024 bytes / audio/pcm-16k；退出 0，麦克风 idle。
上述公交车语句是旧测试素材。2026-09-16 随后通过运行中公共 Perception 的
`POST /tts/test`（`text` 为“你好，这是机器人仿真测试，请识别前方的椅子。”）
在远端生成 `fixtures/speech-room.wav`，未覆盖原 `speech.wav`。
生成 WAV 为 150422 bytes，SHA-256 为
`cfd4fa834adad7e52793d5a0e8e6a441bc0d3fb9ae2f052dac9f01c88d2047c3`；
验证为 16 kHz / mono / PCM16、非零音频后，只重建 Driver 切换素材。
`world-camera.env` 固定新路径；搬到新主机前需在那里生成同格式素材，不能只复制 env。
这是语音内容与桌椅场景的一致性修正，仍是周期回放，不是由场景事件触发的对话。

同日使用当前桌椅场景通过 Core 启动独立 `sim_accept_face_empty_room` 实例，
原版 `face_recognition.recognize_by_stream` 返回 `ok=true`、`count=0`、
`faces=[]`、`frames_examined=1`。这验证有图像但没有人脸时正常返回空结果，
不是断流；未注册或写入任何测试真人身份，不能当作已知身份识别通过。
finally 已停止该实例和相机，均读回 idle。

同日新增并实跑 `scripts/perception_stack_acceptance.py` 联合验收：先确认主控
停止及四张公共卡片、两个仿真传感器 idle，再使用 `sim_stack_*` 独立实例启动
麦克风/三维相机、TTS、ASR、VOP、人脸卡片。TTS 订阅 ASR 输出，测试没有调用
`tts.speak` 或注入假识别文本。真实 ROS 结果为 `asr=true`、`vop=true`（chair）、
`face_empty=true`、`tts_bytes=157360`、非零 PCM、EOF=true；脚本退出 0，
finally 六项 stop 均为 idle，四个应用/Driver 容器 restart_count 均为 0。
它证明并行 ROS 链及 ASR→TTS 自动衔接，不替代全画布按钮启停、长期稳定性或 OCR 验收。
切换椅子语音后，联合脚本新增 `--expected-text`（默认“椅子”，拒绝空值），
不再仅凭 ASR 文本中出现“机器人”放行。实际再次运行退出 0：ASR 命中“椅子”、
VOP 命中 chair、无人脸正常空结果，TTS 非零音频 157754 bytes 且收到 EOF。
finally 六项均停止为 idle；未调用直接 speak 冒充 ASR→TTS 链。

2026-09-16 11:48–11:51（北京时间）另完成 Chrome 真实画布按钮联合验收：
确认 project=false、editor=null、相关工具及其他 owner 的 GPU VOP idle 后，
备份原 8 卡布局，取得编辑锁，仅临时移除 Decision Core 和 GPU VOP。
保留六卡四条连线，点击“开启智能控制”，六卡 ready，`has_error=false`。
逐张点击“查看数据流”并实际检查截图：ASR 有“椅子”语句；公共 VOP 连续检出
chair（约 0.82–0.90）；人脸卡片持续 `count=0, faces=[]`；TTS 显示非零波形。
未点击 `tts.speak`，该音频由 ASR topic 自动触发。ASR 曾把“前方”误识别为
“前田芳”，VOP 也有低置信 bench 输出；本次证明数据流与页面功能，不证明识别准确率。
截图在本机 `/private/tmp/sim-room-{asr,vop,face,tts}-canvas.png`，均已打开检查。
随后点击“停止智能控制”，六项均读回 idle，其他 GPU VOP 仍 idle；恢复后端布局
与备份逐字段一致（8 卡），释放编辑锁并关闭本任务页面。未启用 Decision Core。
此门禁只覆盖上述四张公共感知卡片，人脸正样本、OCR、长期稳定性仍未验收。

复测（在 Perception 容器中加载 ROS 环境后，MCP ID 必须现场核对）：

```bash
python3 perception_stack_acceptance.py \
  --driver-id <仿真Driver的MCP-ID> --perception-id <公共Perception的MCP-ID>
```

此次 Driver 最初使用 stdin overlay 临时部署，原 `.env` 仍指向旧镜像。
为避免复部署丢失覆盖，`config/world-camera.env` 固定了本次已验证的两个本地
镜像 ID、Compose project 和文件顺序。将它以 `world-camera.env` 放在既有运行
目录，并放入 `compose.world-camera.yaml` 及最新 `compose.fixtures.yaml`（含
`SIM_AUDIO_SILENCE_MS` 传递）。它不含凭证，也不替换原 `.env`。
这些 ID 仅适用于镜像已存在的 Docker 主机，不是可从仓库拉取的发布版本。

```bash
# 先验证解析；后一个环境文件覆盖原配置，仅输出服务名而不打印秘密。
docker compose --env-file .env --env-file world-camera.env config --services
# 仅在画布停止、相关卡片 idle 且获得部署授权时执行：
docker compose --env-file .env --env-file world-camera.env \
  up -d --no-deps --pull never perception sim-driver
```

当前 Perception 含已说明的 rclpy 依赖补丁，不应称为完全原版镜像。
2026-09-16 已在既有运行目录保存以上三个文件，实际 Compose 2.29.2 解析验证
`PERSISTENT_COMPOSE_CONFIG_PASS`：四个服务、project 名、两个精确镜像 ID、
MuJoCo 模式、禁用静态图、非空音频路径和 5000 ms 静音间隔均符合预期。
此次只持久化配置，未重启容器；不把配置解析通过等同于重建验收。
仅使用原四个 Compose 文件会恢复原 Driver。回滚命令（在既有运行目录）：

```bash
docker compose --env-file .env -p phanthymotus-sim-public-ipc \
  -f compose.platform.yaml -f compose.p1.yaml -f compose.p2.yaml \
  -f compose.fixtures.yaml up -d --no-deps --pull never sim-driver
```

不要增加 `--remove-orphans`，同机还有其他 owner 的服务。

如需恢复原依赖，在运行目录用原 `.env` 及相同四个 Compose 文件执行
`up -d --no-deps --pull never perception`，且不要设置 PERCEPTION_IMAGE 环境覆盖。

依赖回归命令（在已构建候选镜像的 Docker 主机执行；无网络、无业务卷）：

```bash
docker run --rm -i --network none --memory 512m --cpus 1 \
  --cap-drop ALL --security-opt no-new-privileges --entrypoint /bin/bash \
  phanthymotus-sim/perception:rclpy-guard-candidate2 -c \
  'source /opt/ros/humble/setup.bash && timeout -k 5 120 python3 -' \
  < simulation/scripts/ros_lifecycle_probe.py
```

此外，全画布启动路径 `_resolve_and_register` 会显式注册输出 topic；不能把
单卡调用的 `asyncio` 注册缺陷直接推断为全画布必然失败。
ASR YAML 的 VAD 模式应放在 `plugins.asr.kws.trigger_mode: vad`；顶层同名字段
不会控制 VAD worker，`kws.enabled: false` 也不能替代该字段。错误位置会触发 KWS 模型下载。
公共 VOP 的 CLIP 目录固定为 `/work/weights/clip`，部署文件将同一模型卷同时挂载
到 `/models` 与 `/work/weights`，使其在不修改应用源码的前提下持久化。

基线 `125db64` 的 OCR 校验清单仅含 Jetson TensorRT 引擎及固定哈希；自行编译的
x86 引擎不能直接替换，启动下载器会拒绝/替换不匹配文件。不能通过跳过校验或
运行时 monkey patch 规避“不修改公共源码”的要求。已检查官方 `ocr.py` / `ocr_runtime.py`：
仅支持名为 rapidocr 的 TensorRT 实现，没有可配置的 CPU/ONNX 后端。
2026-09-16 用户已授权在独立 `feat/ocr-onnx-cpu` 分支实现公共 Perception 的显式
CPU 后端并在验证后提 PR。66 项定向测试通过；隔离 CPU 模型已从 MuJoCo
标牌渲染图识别出 `SIM ROOM`（score=0.99798），完整图片入口的有字/空白/损坏输入
检查通过；OCR ROS 与画布数据流验收通过，输出 `SIM ROOM`。
常驻 Perception 已显式切换三个 OCR 业务文件的兼容补丁；Core/ActuCore 未修改。
这不是“所有公共业务源码未修改”的部署，恢复纯基线可使用下面的回滚覆盖。

### OCR CPU 候选覆盖（2026-09-16）

#### 构建候选镜像

`scripts/build-ocr-cpu-image.sh` 固定采用 PR #215 的 `7f87c173cee45354686527d9a684fbcbef67fe2c`，
从该提交生成三个业务文件的补丁，不依赖工作树未提交内容。默认基底是测试机已验证的
rclpy 修补镜像 ID；其他机器须先按 `docker/rclpy-humble.Dockerfile` 构建对应依赖层，
再显式传入 `BASE_IMAGE`。不得把其他版本镜像当作该基底。

在远端准备包含上述提交及父提交的 Git checkout；GitHub 镜像地址须先验证。
两个 wheel 必须在远端下载到 `WHEEL_DIR`，不能在 Mac 下载再上传：
`rapidocr-3.9.1-py3-none-any.whl` 与
`pyvips_binary-8.18.6-cp37-abi3-manylinux_2_28_x86_64.whl`。
Dockerfile 对两者执行固定 SHA-256 校验；其余 Python 依赖仍需镜像源可达。

```bash
# 在 simulation/ 下运行。只构建，不部署，也不推送。
bash scripts/build-ocr-cpu-image.sh "$OCR_CHECKOUT" "$WHEEL_DIR" phanthymotus-sim/ocr:rebuild
# 在隔离容器中验证真实模型；模型只读卷需先准备完整固定哈希权重。
docker run --rm --network none --cpus 2 --memory 2g \
  -v sim-ocr-onnx-candidate-models:/models:ro \
  --entrypoint bash phanthymotus-sim/ocr:rebuild \
  -c 'PYTHONPATH=/work python3 /work/tools/verify_ocr_cpu.py'
```

此入口复现源码与依赖版本，不保证镜像字节级一致；默认基底不是已发布镜像。

`config/ocr-cpu.env`、`config/perception-ocr-cpu.yaml` 和 `compose.ocr-cpu.yaml`
记录当前已验证的本地镜像及独立模型卷。将 `ocr-cpu.env` 放在部署目录，与
原 `.env`、`world-camera.env` 同级；配置 YAML 保留在 `config/`。仅在画布停止、
无其他编辑者且相机/麦克风 idle 时执行：

```bash
docker compose --env-file .env --env-file world-camera.env --env-file ocr-cpu.env config --quiet
docker compose --env-file .env --env-file world-camera.env --env-file ocr-cpu.env up -d --no-deps perception sim-driver
# 回滚：原配置、旧镜像及模型卷均保留；不重建 Core/ActuCore。
docker compose --env-file .env --env-file world-camera.env up -d --no-deps perception sim-driver
```

这些 SHA 是已存在于测试机的本地镜像，不是可从镜像仓拉取的发布版本。上述仓库内
脚本已远端重建成功：`phanthymotus-sim/ocr:rebuild-7f87c17`，image ID
`1f33ef241ae40ef5aec2f181185ad2ac666f7564ef8837beeaa2bf1bb01e33e4`。
断网、2 CPU/2 GiB、只读模型卷的真实推理检查通过，三个 OCR 业务文件与已部署
候选镜像 SHA-256 一致；常驻服务未切换。独立 OCR 源码提交为 `7f87c17`，
见 [PR #215](https://github.com/4paradigm/phanthymotus/pull/215)。
2026-09-16 已通过 OCR producer-first ROS、四卡联合回归
和七卡画布启动/停止；网页看到 OCR 标牌、ASR 文本、椅子检测及空人脸结果。
后续补验：公开 bus.jpg 回放检测出两个人脸（未知身份、低质量），仅证明检测正样本，
不证明身份识别；已恢复 MuJoCo 相机。前台 Chrome 实际显示 TTS 橙色活动波形，
ROS 收到 96408 bytes 音频及 EOF，停止后 idle；未做人工听感评价。
截图：`/private/tmp/sim-ocr-candidate-tts-visible-20260916.png`（本机验收证据，不随仓库发布）。
2026-09-15 另外只读比较已获取的上游 `2f872a342e7e5f82fc747f4222053b387278064b`：
ASR 仍先创建订阅再启动 VAD；Core 的 `mcp_call_tool` 仍包含局部 `import asyncio`；
OCR 相关实现未变化。不能以该版本升级替代这些问题的修复。当前运行基线仍为 `125db64`。

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
