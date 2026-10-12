# 钩子 effort 标定

从仓库根目录进入 ROS 2 容器后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/pc10/r1pro_web_colcon/install/setup.bash
python3 calibration/hook_effort_calibration.py
```

程序只读取左右臂 `JointState.effort`，不会向机器人发送运动指令。完整参数和操作步骤见
仓库根目录的 `todo.md`。

## 完整钩子搬箱标定向导

`hook_effort_calibration.py` 只解决“试抬升后 effort 是否足够”的一项标定。
完整搬箱还必须固定源/目标位姿、料盆局部挂点、左右钩尖 TCP、钩姿态、
预挂/试抬/完整抬升/放下/脱钩/退出偏移，以及搬运高度、waypoint 和速度。
向导还会从 Web API 读取当前四个 torso 关节作为固定高度候选值；应先把躯干
调整到计划长期使用的搬箱姿态，再开始标定。dry-run 和 live 都会比较当前 torso
反馈与标定值，超过容差时拒绝执行，不会自动升降躯干。
使用下面的向导可以把这些值与 effort 报告合并为可直接提交给
`dual_hook_pick_and_place` 的 YAML：

```bash
source /opt/ros/humble/setup.bash
source /home/pc10/r1pro_web_colcon/install/setup.bash
python3 calibration/hook_transport_calibration.py \
  --effort-report runtime_data/r1pro_web_control/hook_effort_calibration.yaml \
  --output runtime_data/r1pro_web_control/hook_transport_calibration.yaml
```

向导只收集数据并离线检查，不发送机器人动作。`workflow_params` 是完整参数，
`derived_plan_summary` 给出计算后的挂点、钩尖和全路径 waypoint 数；
其中 `source_hook_point_span_m` 是箱体挂点宽度，
`source_commanded_tip_span_m` 是考虑左右 TCP 后实际命令的末端跨度，
`source_engagement_tip_z_m` 是左右挂入绝对高度；
`calibration_complete=false` 或 `remaining_checks` 非空时不得真实搬运。

报告默认写入：

```text
runtime_data/r1pro_web_control/hook_transport_calibration.yaml
```

代码工作流的“完整钩子标定报告路径”默认就是这个文件。报告存在时，workflow
会自动载入其中的 `workflow_params`；Web 表单里显示的默认值不会覆盖报告中的
几何、TCP、轨迹、effort、躯干和导航动作名。若报告路径被改成其他位置，只需在
Web 表单填该路径并保存代码工作流原子动作，不需要修改 Python。

钩子搬运现在支持“挂住料盆后由底盘执行保存的导航原子动作”：到达目标点后
才执行双臂放置、脱钩，再执行返回初始点导航原子动作。向导会从 Web 动作库列出
`navigate_to_pose` 动作，要求填写：

```yaml
hook_nav_to_place_action_name: <目标放置点导航动作名>
hook_nav_to_home_action_name: <返回初始点导航动作名>
```

这两个动作名必须对应 Web 中保存的导航原子动作；workflow 不会使用硬编码的
`(0,0,0)` 替代它们。底盘导航期间双钩保持承重，导航完成并确认底盘稳定后才开始
放置和脱钩。

在 Web“原子动作工作流”页选择钩子 workflow 后，确认报告路径并点击“另存为原子动作”。
保存的动作会携带当前参数；将它加入总工作流后仍会使用同一份标定报告，而不是恢复
类默认值。真实执行前仍必须先 dry-run，并满足报告 `calibration_complete=true`、
`remaining_checks=[]`。如果直接从 Web 点击真实执行，还要勾选“已确认现场安全并允许
真实运动”（对应 `motion_confirmed=true`）；默认关闭。

### 先 dry-run，再真实搬运

确认报告已经完整、Web 服务和控制 token 都是当前实例后，可让程序调用 API 做
在线 IK dry-run；只有在线 dry-run 成功并输入精确确认短语，才会提交真实工作流：

```bash
python3 calibration/hook_transport_calibration.py \
  --load-report runtime_data/r1pro_web_control/hook_transport_calibration.yaml \
  --run live --api-url http://127.0.0.1:8080 \
  --token-file ~/.ros/r1pro_web_control/token
```

控制 token 也可以通过 `R1PRO_CONTROL_TOKEN` 环境变量提供；不会写入标定报告。
程序会在超时或异常时发送停止请求。中止、急停和 Web UI 的停止按钮仍然优先。
