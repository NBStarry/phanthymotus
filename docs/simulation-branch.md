# `sim` 分支维护约定

`NBStarry/phanthymotus:sim` 是一个长期仿真集成分支。它用于保存 Core 和
Perception 中方便虚拟环境调试的源码修改，不以合并进
`4paradigm/phanthymotus:main` 为交付目标。

## 边界

本分支可以包含：

- Core / Perception 中通用的仿真调试和可观测性改进；
- 默认关闭、由环境变量显式开启的仿真辅助功能；
- 与行为同步的单元、契约和 WebUI 回归测试。

以下内容不放入本分支：

- Sim Driver、MuJoCo / Gazebo backend、地图、bag、模型和其他大文件；
- WLCB 私网地址、代理地址、token、密钥或设备标识；
- 绕过生命周期、停止、失联或安全检查的调试后门。

仿真 Driver、Compose、资源限制、版本锁和远端验收脚本继续由独立的
`phanthymotus-sim` 项目管理。该项目必须锁定本分支的精确 commit，不能只锁分支名。

## 同步上游

长期分支使用普通 merge 保留已部署 commit 的可追溯性，不 rebase、不强推：

```bash
git fetch upstream main
git switch sim
git merge --no-ff upstream/main
```

每次同步后都要重新运行定向测试、P0 和当前最高阶段仿真验收，再更新
`phanthymotus-sim/versions.lock.yaml` 中的上游基线和本分支 commit。

## 当前本地修改

- Feishu REST 鉴权、健康探测和 Open API 调用遵循标准 HTTP(S) 代理环境变量；
- Canvas 的数据流详情向 renderer 传递真实 MCP id，与监控面板使用同一 Driver 模型；
- 显式设置 `LOCAL_SERVICES_MANIFEST` 时，Core 会将运行时提供的本地仿真镜像纳入
  标准「我的服务」和 Docker 生命周期；正式环境未设置该变量时行为不变。

上述行为都有源码内回归测试，不再由构建时 patch 产生。
