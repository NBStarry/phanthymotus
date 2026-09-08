# 二维导航 Driver 接入合同（待 Driver owner 实现、确认）

本文件是双仓交接需求，不表示天轶当前 Driver 已满足。Driver 同事独立实现与
提交；本分支没有修改 Driver。工具/topic 名沿用 Driver 已发布名称，通过
卡片配置与 Canvas 连线接入，不要求重命名硬件端。

## 输入 producer

| 数据 | ROS type / QoS | 源语义与初验目标 |
|---|---|---|
| 二维扫描 | sensor_msgs/msg/LaserScan；Reliable 发布兼容 Best Effort 消费，KeepLast 2 | 原始扫描，建议有效更新≥5 Hz；真实采样开始时间、range/angle、frame；不是轮询5次同一缓存 |
| 里程计 | nav_msgs/msg/Odometry；Reliable，KeepLast 5 | 连续 odom→base_link，建议≥20 Hz；米、弧度、m/s、rad/s，协方差有效 |
| TF | 标准 /tf 与 /tf_static | Driver 只提供 odom→base_link→laser/camera，不导入厂商竞争的 map→odom |
| RGB，可选 | std_msgs/msg/UInt8MultiArray；PSE1，phanthy.sensor.camera_rgb_frame.v1 | JPEG 与 header.stamp_ns/timing.source_stamp_ns 相同、ros_system_time；移动相机必须有源时刻动态 TF |
| 执行反馈 | std_msgs/msg/String，data/json；Reliable，KeepLast 1 | 下述 JSON，每次实际硬件读取成功后生成，建议≥10 Hz |

所有 ROS stamp 使用同一已验证的系统时间域，跨主机须量测时钟偏差。
首次接入建议扫描、里程计源年龄 p95<200 ms、最大<500 ms；这些是门槛目标，
不是对天轶的实测结论。测不到源时间必须明确不可用，不拿接收时刻补写。

连续里程计优先来自轮速/底盘里程计。若只有厂商全局定位，先确认是否存在
连续里程计接口；禁止在全球定位发生重定位跳变时仍宣称 odom 连续。
只有二维地图/激光点列表而无原始扫描时间与角度时，不能假装完成此合同。

雷达在测试姿态下必须水平且与底盘刚性连接。扫描平面看不到的上身碰撞体、
台阶和悬空物要在现场安全范围中排除；真实 footprint 包含测试姿态整机投影。

## 执行 consumer

卡片真实输出为 /<namespace>/nav2/velocity_proposal，schema 沿用
phanthy.navigation.velocity_proposal.v1，frame=base_link：

~~~json
{
  "schema": "phanthy.navigation.velocity_proposal.v1",
  "nav_id": "unique-navigation-id",
  "sequence": 1,
  "issued_at_unix_ms": 1700000000000,
  "ttl_ms": 250,
  "frame": "base_link",
  "nav_status": "navigating",
  "velocity": {"x": 0.2, "y": 0.0, "yaw": 0.0},
  "shadow_only": true,
  "physical_execution": false,
  "reason": null
}
~~~

上面两项布尔值表示卡片只是提案者，不表示 Driver 可以省去当次执行授权。
不得连接卡片内部 proposal_preview topic，它是算法验证流，不是物理执行输入。

Driver 必须：

- 接受新鲜、合法、非零首包后绑定 nav_id；零速首包不武装。检查单调 sequence、
  有效期、时钟、数值、frame、schema、前进/转向及实际速度/加速度上限。
- 同时只能有一个运动控制权，覆盖厂商遥控、原生导航和直接 SDK 等竞争路径，
  不只检查 ROS topic 发布者数量。不允许本卡片在后方绕过厂商安全控制。
- 暂停时停车并保留可恢复的当前任务；终态 arrived/cancelled/aborted/error/stopped
  都只允许零速，完成真实停车确认后退役旧 ID，再接收下一任务。
- 丢失提案、Driver 进程退出、失去反馈、急停时独立执行已验证的停车策略。
  stop_navigation/cancel 不能被长时动作或模型调用阻塞。
- 返回明确拒绝原因，不吞掉 nav_id 冲突或仅返回 HTTP 成功。

## 最小新增反馈

当前卡片消费以下合同。若 Driver 已有等价状态，请由 Driver owner 给出映射，
经双方确认后统一，不创建第二套语义相冲突的状态。

~~~json
{
  "schema": "phanthy.navigation.execution_status.v1",
  "source_stamp_ns": 1700000000000000000,
  "safety_ready": true,
  "nav_id": "unique-navigation-id",
  "stop_confirmed": true,
  "reason": null
}
~~~

- source_stamp_ns 表示一次新的、真实硬件反馈读取；旧缓存重新发布不能刷新它。
- safety_ready 代表实际授权、急停、控制权和硬件健康条件，不只是连接在线。
- stop_confirmed=true 仅在执行停车后，根据新硬件运动反馈确认近零。
  卡片要求该回执的 nav_id 与待结束任务一致，且源时间晚于发起停车的时刻。
- 终态回执需保持同一 nav_id 足够长或可查询重放；不得立刻改成 null 使消费者
  永远无法确认。空闲且从未绑定任务时 nav_id 可为 null。
- 人工急停/取消不得因随后恢复 safety_ready 而自动重新武装旧任务。

## Driver 验证交付

提交真实 tools/list、配置样例、topic 类型/QoS、TF 树与原始采样证据；
分别统计生产次数、不同源帧次数、消费次数，给出时钟偏差与间隔分布。
测试零速首包、过期/乱序包、其他 nav_id 干扰、连续十任务、
暂停/恢复、终态重放、断流、进程退出和真实停车；现场人员负责物理操作。

本卡片已有调用链与测试代码不等于该 Driver 合同完成。联合验收前，
execution_enabled 保持 false，不能设置通过标记替代测试。
