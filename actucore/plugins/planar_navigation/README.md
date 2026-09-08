# 二维语义导航 PlanarSemanticNavigation

独立的 ActuCore 产品：使用二维激光扫描和连续里程计建图、保存地图、定位、
导航，并使用可选 RGB/VLM 记录语义地点。不是 FAST-LIVO2 的模式开关，
不依赖厂商地图、厂商规划、G1 SDK 或三维导航分支。

**当前状态：开发验证中，不是 accepted。** 本地单元/契约测试不能证明真实
ROS 算法、Canvas、Driver 执行或天轶运动通过。真实输入未接入前不可对外宣称
“已支持天轶导航”。不在此分支发布 Resource Center Card/Skill。

## 怎么工作

~~~text
Driver LaserScan + odom/TF
  ├─ 建图：SLAM Toolbox（异步），现场人工移动
  └─ 定位：Map Server + AMCL（需初始位姿）
                ↓
         NavFn + Regulated Pure Pursuit
                ↓
      检查定位/数据/生命周期/Driver 状态
                ↓
     velocity proposal → Driver 独占执行与停车确认
~~~

建图和定位互斥，不在建图时运行自主导航。地图保存使用 Nav2 map_saver_cli，
只有 YAML、图像通过核验并写入目录索引后才停建图；失败保留算法与原始现场。
加载地图不会自动宣称定位成功。relocalize 发出初始位姿，AMCL 的新位姿协方差、
对应时刻 TF、扫描、里程计、代价地图和 Nav2 生命周期均有效后才允许导航。

全局代价地图覆盖已加载地图，局部地图滚动；实时扫描执行 marking + clearing。
控制器不倒车，可停下转向，也可能产生小曲率前行，不承诺“只直线加原地转向”。
目标未知、占据或内切膨胀区域直接拒绝。碰撞检查仍由 Nav2 执行，禁止通过
缩小实测 footprint 绕过安全拒绝。

标准控制默认 20 Hz，非零提案默认 5 Hz，TTL 250 ms；零速不等待周期。
直行速度默认最多 0.2 m/s、角速默认最多 0.3 rad/s，不设置最低运动速度；
实际速度和加速度还必须受 Driver 验证上限约束。位置容差 0.20 m、
终点朝向容差 0.10 rad。配置在停卡后生效。

## 接线与配置

| 输入 | 数据 | 必需 |
|---|---|---|
| scan | sensor_msgs/LaserScan，真实扫描源时间、米/弧度 | 是 |
| odom | nav_msgs/Odometry + odom→base_link，连续非全局位姿 | 是 |
| rgb | UInt8MultiArray，PSE1 RGB frame、JPEG、源时间与 frame | 语义 capture 时 |
| execution_status | Driver 执行/安全/停止确认 JSON | 启用执行提案时 |

输入 topic 可配置，不能为了适配卡片擅自改 Driver 已发布 topic。当前主线
Core 提供 input_topics；卡片按配置检查实际连线。支持 input_bindings 的
Core 也会检查端口和 topic 一致。TF 由标准 ROS 接入，不额外增加 Canvas 端口。

默认 scan_topic=/scan、odom_topic=/odom 只是示例，必须改为 Driver 的真实输出。
base_frame=base_link，odom_frame=odom，map_frame=planar_map。雷达固定安装 TF
与头部相机的动态 TF 由 Driver 提供。二维扫描要求近似水平和平面运动，
不能把头部深度点云/二维地图点或厂商位姿随意改名当作算法原始输入。

footprint 默认 []，故意阻止加载地图后自主导航。配置测量得到的、有序凸多边形
顶点，单位米，包含底盘、上身及测试姿态伸出的结构；不要复制其他本体轮廓。
例如矩形只是格式示意：[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]。

execution_enabled=false 时，只在内部 proposal_preview topic 发布算法预览，
不暴露可连执行器的提案端口。**shadow_only=true 不是物理禁用开关**：
已有 Driver 会执行这种提案。因此严禁把内部预览 topic 手动连到 Driver。
只有现场确认执行接口与控制权后，才停卡并设置 execution_enabled=true；
同时必须配置并连接 execution_status。详细要求见 [Driver 接入合同](DRIVER_CONTRACT.md)。

## 使用流程

1. 在 Canvas 配置真实输入 topic、地图目录和 footprint，连接扫描与里程计。
2. 启动卡片；status 显示输入缺失/过期等原因，不假装定位就绪。
3. start_mapping(map_name)，现场人工移动，map_view 查看累计地图和青色实时扫描。
4. stop_mapping 返回保存地图的 map_id/revision。保存失败可以重试，不会返回空成功。
5. load_map(map_id)，等待 AMCL 激活后 relocalize(x,y,yaw)；info.ready=true
   表示本轮输入与定位门槛通过，不等于物理导航已验收。
6. navigate_to_pose(x,y,yaw)。pause_navigation/resume_navigation 或
   stop_navigation 均可从独立控制通路调用。人工停卡不会被 VLM 请求锁住。
7. 到达、取消、失败先发布终态零速；执行模式下，等待相同 nav_id 的新鲜
   Driver 停车确认后才发 ACP 完成事件、释放任务并允许下一次导航。

地图监控用现有 image/jpeg 渲染器，包含当前位置、路径和实时扫描。
定位时显示全局 costmap：cost≥99 红色、未知灰色；灰色同样不可直接设为目标。
显示分辨率为 640×640，不限制算法地图点数；安全资源上限是 1600 万栅格。
不修改 Core renderer，不增加独立 plan/goal 卡片。

地图目录包含 catalog.sqlite 与每个随机 map_id 子目录的 map.yaml/map.pgm。
版本是地图实际字节摘要；同名地图不会覆盖旧地图。目录索引事务成功前，
产物不出现在 list_maps。失败的未索引目录保留用于人工排查，不自动删除。
磁盘低于 256 MiB 时拒绝新地图或语义数据写入；部署须预留远高于此值的空间。

## 语义地点

配置 rgb_topic、vlm_url、vlm_model、vlm_api_key，再连接 RGB 输入。vlm_url
为支持 OpenAI chat/completions 结构的 HTTPS 服务根路径（例如结尾 /v1）。
密钥不进入回执和日志，schema 标注敏感字段；VLM 会收到拍摄 JPEG，
部署者必须确认上传图片符合团队的数据授权。

capture 只在固定地图完成定位后工作：等待调用后的新 RGB，并按同一源时间
查 map→base_link 和相机 TF，生成地点描述，将 JPEG、描述、位姿、时间和
地图版本事务写入 SQLite。最多 1000 个地点；list_points/delete_point 管理。
本功能没有深度录制、MCAP 或自动距离真值导出。

navigate(query) 仅匹配当前地图的记录地点。精确 ID/描述可直接命中，
否则 VLM 只能从给定 ID 中选；不存在或歧义结果不会构造运动坐标。
支持中文查询。VLM 内容只当数据，不执行其中的指令。

重启后加载同一 map_id/revision 并重新定位，记录地点可复用。
传感器短暂 stale 不删地点；换图只显示对应地图地点；篡改地图文件会明确报
map_revision_mismatch。首版不支持边建图边记录地点，避免回环优化改变坐标。

## 故障与停止

- 扫描/里程计同时检查接收年龄和源时间年龄，重复源时间不刷新接收新鲜度。
- TF、扫描、里程计或 Nav2/Driver 不可信时，提案归零。恢复连续健康一秒后，
  仅在原任务仍有效、未人工暂停/取消时恢复；旧非零候选已清空，需新命令。
- 里程计几何跳变锁存阻断，需要停卡、确认输入后重新启动和定位，不自动复用。
- Nav2 结果不等于停车。Driver 未确认时 stop_unconfirmed，任务不释放，
  页面可重试停止。不能把删除卡片或退出进程当作已物理停止。
- 进程退出后的最终安全依赖 Driver TTL 和硬件急停；卡片不能替代 Driver 安全层。
- 二维扫描无法发现扫描平面之外的台阶、坑洞或悬空障碍。首验需受控平整场地，
  确保雷达盲区不存在危险；这不是全身三维避障产品。

## 构建和离线验证

~~~bash
# 仓库根目录：只构建本地，不登录、不推送或注册。
bash deploy/build_actucore.sh --jp-version 6.1 --mirror tuna --local
bash deploy/build_actucore.sh --base --jp-version 6.1 --mirror tuna --local

# 无 ROS 的本地测试，需 Pillow、PyYAML；测试目录不访问真实配置数据库。
python3 -m unittest discover -s actucore/tests -p test_planar_navigation.py -v

# 镜像构建后替换 <IMAGE>。隔离网络和域，不挂设备、不连 Driver。
docker run --rm --network none \
  -e ROS_DOMAIN_ID=191 -e ROS_LOCALHOST_ONLY=1 \
  <IMAGE> bash -c 'source /ros_ws/install/setup.bash && python3 /work/tests/planar_ros_smoke.py'
~~~

卡片随标准 ActuCore 镜像构建；JP6.1 的锁定 Nav2/SLAM 运行依赖先构建为
`actucore-planar-navigation-base`，再由统一 ActuCore 镜像消费。base 不是可部署
卡片，不复制应用代码，也不注册 Resource Center。它继承框架 `jetson-base:jp${JP_VERSION}-torch`，
保留 CUDA torch、原有卡片、配置和部署入口；不再提供独立 CPU/planar 镜像。
二维算法本身不用 GPU，不代表整个共享镜像没有 GPU 依赖。新增算法只在启动卡片时运行。
APT 只读取临时生成的国内 HTTPS Ubuntu 二进制源并保留签名校验，不读取基础镜像的
ROS APT 索引，也不安装第二套 ROS。测试实际执行 focal/jammy 的源生成命令。
SLAM Toolbox 2.6.10、Nav2 及传递源码提交锁定在 [sources.lock](sources.lock)，
只编译基础镜像缺少的包，产物在 `/opt/actucore_navigation_ws/install`。
源码锁和系统包清单留在 `/opt/actucore_navigation_ws/`；基础镜像仍使用框架 tag，
不承诺逐字节可复现，发布需记录最终 RepoDigest。`BUILD_JOBS` 默认 2；
`GIT_MIRROR_PREFIX` 可指定源码下载镜像前缀，下载有超时和三次重试。
SLAM Toolbox 为 LGPL（上游仓库标识 LGPL-2.1），Nav2 为 Apache-2.0；
发行包及传递依赖保留原许可证。没有复制 FAST-LIVO2/GPL 算法代码。

标准基础镜像可能需要下载；构建前检查 Docker 数据目录磁盘而不只是代码目录。
共享 CUDA/ROS 基础镜像较大，建议至少 30 GiB 可用构建空间，这是操作预留值，不是实测镜像增量。
不能因代码目录位于 NVMe 就假定 /var/lib/docker 也在 NVMe。

真实 ROS smoke 使用合成矩形房间和理想运动模型，实际启动算法子进程，
检查非空地图、保存、AMCL、连续两次到达及扫描断流归零。它不能代替现场测试，
也不证明复杂环境定位精度。缺少 ROS 或算法包时必须失败，不伪造 PASS。

## 验收与回滚

天轶 Driver 同事先按合同交付，现场先不取得执行控制权验证，再低速执行。
完整验收：静止/人工移动数据频率与延迟、建图与扫描重合、存图加载、
重定位、十次连续导航、绕障、暂停/取消后重试、断流恢复、停止距离、三个语义
地点重启后复用。记录输入、处理、控制、提案和执行实际频率，以及 CPU、内存、
端到端延迟 p50/p95/max，不用配置频率代替实测。

雨强五分钟复核：加载已验收地图 → 重定位 → 前往记录地点 → 停止 → 再次导航。
保存源码 SHA、镜像摘要、配置、Driver 版本、地图摘要、日志、视频及验收人。
有前置失败就显示真实失败，不能跳过或以进程存活代替完成。

回滚由现场人员先停导航并确认 Driver 停车，再恢复先前 ActuCore 镜像/配置。
不要覆盖地图目录，不动 Core/Perception/Driver 其他业务。当前 build 不自动部署。
