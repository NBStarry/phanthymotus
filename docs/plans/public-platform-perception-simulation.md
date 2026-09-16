# 公共三层与仿真 Driver 感知闭环

## 目标与边界

部署公共 Core、Perception、ActuCore 的锁定主仓版本，在 x86 原生构建；仿真数据与控制逻辑留在 simulation/Driver。通过实际 ROS 输入和 WebUI 验证现有感知卡片，而不是伪造感知结果。

2026-09-16 用户补充授权：公共 OCR 缺少 x86 推理路径，可在独立 `feat/ocr-onnx-cpu` 分支修改 Perception 并在验证后提 PR。该兼容变更不混入仿真代码，不据此扩大 Core/ActuCore 业务修改范围。当前 Perception 的 rclpy 依赖修补与公共业务文件未修改是两个不同事实，必须分别报告。

## 实施与验收

1. 锁定公共应用版本、镜像与资源限额，保留恢复入口；核对实际运行文件，而不只检查镜像标签。
2. 仿真 Driver 持续生成 MuJoCo 场景相机图像、物理状态及明确标注的语音回放；暂停、断流和停止必须可观察。
3. 真实数据链验证：语音→ASR→TTS；场景相机→VOP/人脸检测/OCR。画布拖拽连线、开始/停止按钮和数据流窗口均需观察。空人脸场景只能证明负样本路径。
4. OCR 先在隔离 CPU 进程验证真实模型及错误路径，再加入场景中的可读标牌。标牌是实际场景纹理，由相机渲染，不直接注入 OCR 文本。
5. 验证后更新锁定配置、启动说明、当前证据和缺项；不把仿真单卡输出等同于真机、安全或识别准确率验收。

## 当前证据与缺项

- 已有：公共应用文件基线 `125db64fb860` 核验；MuJoCo 场景相机；ASR/VOP/空人脸/TTS 真实 ROS 联调和四卡 WebUI 验收。
- 交付复核：对当前运行镜像重新逐文件校验 Core 157 文件、ActuCore 4 文件，均与锁定公共源码一致；运行 Driver 的 main.py、mujoco_backend.py、media_fixture.py 与本地交付文件 SHA-256 相同；智能控制 running=false。
- 标牌 Driver 和 OCR 最小补丁候选已部署：相机→OCR ROS 输出 `SIM ROOM`（0.99293）；ASR→TTS/VOP/空人脸联合回归通过；完整图片入口有字/空白/损坏检查通过。
- 七卡五连线临时画布经页面按钮启动，OCR/ASR/VOP/空人脸数据流实际可见，TTS 面板收到句末标记（本轮截图未捕获活动波形）；页面停止并恢复原八卡、editor=null。Core/ActuCore 镜像未变，Perception 43 个业务 Python 文件仅三个 OCR 补丁文件不同。
- 补验通过：公开 bus.jpg 回放检测到两个人脸（低质量、未知身份，仅证明检测正样本）；已恢复 MuJoCo 相机。前台 Chrome 的 TTS 面板实际显示橙色活动波形，ROS 收到 96408 bytes 音频及 EOF，随后停止。
- OCR 已独立提交 `7f87c17`，PR：https://github.com/4paradigm/phanthymotus/pull/215 ，目标 upstream/main；不含仿真 Driver、Core 或 ActuCore 改动。
- 已补齐仓库内 `simulation/docker/perception-ocr-cpu.Dockerfile` 与 `simulation/scripts/build-ocr-cpu-image.sh`：固定 PR 提交、生成最小补丁、校验 wheel，README 同步运行入口。`bash -n`、缺参拒绝、diff 检查通过。
- 远端源码下载和脚本重建成功：`phanthymotus-sim/ocr:rebuild-7f87c17`，image ID `1f33ef241ae40ef5aec2f181185ad2ac666f7564ef8837beeaa2bf1bb01e33e4`。断网、2 CPU/2 GiB、只读模型卷中真实推理通过有字/空白/损坏输入；三个 OCR 业务文件 SHA-256 与已验收常驻镜像一致。未替换常驻服务，不要求不同构建时间的镜像字节级相同。
- 边界：Jetson 未验收；候选兼容镜像未发布到镜像仓，已提供本机构建入口。
- 不在本次扩展范围：统一雷达/深度场景、摄影级渲染、声学传播、双足步态证明。语音回放与当前房间语义相符，但不声称事件驱动对话。

本文与实际范围一致；simulation/README.md 已同步 OCR、人脸正样本与 TTS 可见波形证据和未完成项，不修改其他 owner 状态。

## 完成审计（2026-09-16）

| 要求 | 证据与结论 |
| --- | --- |
| 公共层独立部署、业务源码边界 | 四容器运行、restart_count=0；Core 157/ActuCore 4 文件逐字节一致；Perception 仅经用户授权的三个 OCR 文件例外，rclpy 修补另行明示。 |
| 连续仿真输入 | MuJoCo 场景相机随姿态变化，camera_ros_probe 已验证变化和停止/断流；语音为带间隔的显式回放，不是实时声学仿真。 |
| 现有感知真实运作 | ASR 文本、TTS 非零音频与 EOF、VOP 椅子、空人脸与两脸正样本、OCR SIM ROOM，均来自实际 ROS/模型链；不是填充感知输出。 |
| 人能看到结果 | 已复核 OCR 数据流与 TTS 橙色波形截图；前述七卡联调经页面开始/停止，恢复原画布。 |
| 可复现与恢复 | 公共镜像构建/源码校验、Driver 渲染 Dockerfile、固定 OCR PR 补丁脚本、Compose 配置及回滚命令齐备；OCR 脚本在远端实际构建并断网推理通过。 |
| 测试边界 | 媒体回放边界测试在实际 Driver 镜像中通过（1 项）；Mac 缺 Pillow，未声称本机测试通过。OCR 66 项及真实图片错误路径已通过；不代替 Jetson、准确率或长期稳定性验收。 |

本次按用户补充授权后的范围完成部署与感知闭环交付；不扩展为全部硬件能力或生产可靠性认证。
