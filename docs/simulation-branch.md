# `sim` 分支维护约定

`NBStarry/phanthymotus:sim` 是一个长期仿真集成分支。它用于保存 Core 和
Perception 中方便虚拟环境调试的源码修改，不以合并进
`4paradigm/phanthymotus:main` 为交付目标。

## 边界

本分支可以包含：

- Core / Perception 中通用的仿真调试和可观测性改进；
- 默认关闭、由环境变量显式开启的仿真辅助功能；
- 与行为同步的单元、契约和 WebUI 回归测试。
- `simulation/` 下的 Sim Driver、MuJoCo / Gazebo backend、Compose、地图、版本锁和验收脚本。

以下内容不放入本分支：

- bag、模型、wheel、镜像归档和其他大文件；
- WLCB 私网地址、代理地址、token、密钥或设备标识；
- 绕过生命周期、停止、失联或安全检查的调试后门。

仿真代码只在本仓 `simulation/` 维护，不再建立独立 `phanthymotus-sim` Git 仓库。
远端部署从当前干净的 `sim` 分支 HEAD 构建，镜像标签记录其精确 commit；大文件只在
远端下载到 `simulation/artifacts/` 或 Docker volume。

## 同步上游

长期分支使用普通 merge 保留已部署 commit 的可追溯性，不 rebase、不强推：

```bash
git fetch upstream main
git switch sim
git merge --no-ff upstream/main
```

每次同步后都要重新运行定向测试、P0 和当前最高阶段仿真验收，再更新
`simulation/versions.lock.yaml` 中的上游基线。

## 当前本地修改

- Feishu REST 鉴权、健康探测和 Open API 调用遵循标准 HTTP(S) 代理环境变量；
- Canvas 的数据流详情向 renderer 传递真实 MCP id，与监控面板使用同一 Driver 模型；
- 显式设置 `LOCAL_SERVICES_MANIFEST` 时，Core 会将运行时提供的本地仿真镜像纳入
  标准「我的服务」和 Docker 生命周期；正式环境未设置该变量时行为不变。

上述行为都有源码内回归测试，不再由构建时 patch 产生。
