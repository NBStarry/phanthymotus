# 第三方依赖锁

Nav2 companion 的基础镜像和 apt 包必须按 `source-lock.env` 与 `Dockerfile` 中的精确版本构建。下表记录 card5 当前镜像所用的直接依赖；镜像内同时生成 `/opt/g1-nav2-package-lock.txt` 供部署审计。

| 包 | 锁定版本 | 上游 | 许可证 |
|---|---|---|---|
| `python3-colcon-common-extensions` | `0.3.0-100` | <https://github.com/colcon/colcon-common-extensions> | Apache-2.0 |
| `python3-pytest` | `6.2.5-1ubuntu2` | <https://github.com/pytest-dev/pytest> | MIT / Expat |
| `ros-humble-nav2-bringup` | `1.1.20-1jammy.20260613.010543` | <https://github.com/ros-navigation/navigation2> | Apache-2.0 |
| `ros-humble-navigation2` | `1.1.20-1jammy.20260613.005009` | <https://github.com/ros-navigation/navigation2> | Apache-2.0 |
| `ros-humble-pointcloud-to-laserscan` | `2.0.1-3jammy.20260607.112335` | <https://github.com/ros-perception/pointcloud_to_laserscan> | BSD |
| `ros-humble-rmw-fastrtps-cpp` | `6.2.10-1jammy.20260605.135747` | <https://github.com/ros2/rmw_fastrtps> | Apache-2.0 |
| `ros-humble-slam-toolbox` | `2.6.10-1jammy.20260613.004837` | <https://github.com/SteveMacenski/slam_toolbox> | LGPL |

版本来自本机已构建的 `phanthy-nav2:g1-humble-nav2card4` 镜像中的 `dpkg-query -W`；许可证来自镜像内 Debian copyright 或 ROS `package.xml`。镜像构建若无法解析任一精确版本必须失败，不允许自动升级到仓库最新版本。

本机 Docker Desktop 的 arm64 模拟环境中，apt 的 `_apt` sandbox 会让 detached signature 校验异常退出，而同一响应的 `gpgv` 验签正常；Dockerfile 因此仅在构建阶段设置 `APT::Sandbox::User=root`。仓库签名校验仍然开启，未使用 `trusted=yes`、`AllowUnauthenticated` 或 `--allow-unauthenticated`。
