# pc10 Web 控制代码副本

复制日期：2026-10-09。来源：pc10 (192.168.3.23) 的 ros2_humble_gpu 容器所使用的工作空间。

## 编辑入口

- 前端：src/r1pro_web_control/web/index.html、app.js、styles.css
- 后端：src/r1pro_web_control/r1pro_web_control/server_node.py
- 离散微调：src/r1pro_web_control/r1pro_web_control/code_workflows/aruco_id1_chassis_fine_adjust.py
- ArUco 识别：src/r1pro_aruco_box_pose/
- 启动脚本：scripts/
- 运行快照：runtime_snapshot/（当前网页、启动参数、actions.json、sequences.json）

本副本用于编辑。脚本和配置中的 /home/pc10 路径保留了原始值，复制后不能直接当作本机部署运行。
后端需要 Linux ROS 2 Humble、机器人 ROS 网络和相关依赖；FoundationPose、YOLO 模型权重、数据集和 Docker 镜像不在本包中。控制令牌没有复制。
Mac 上运行前端时，需将 /api 请求和图像/事件流代理到 http://192.168.3.23:8080；直接用静态服务器打开原版网页不足以连接后端。

## 下载到 Mac 桌面

在 Mac 终端执行（连接当前机器人）：

```bash
scp nvidia@192.168.3.4:/home/nvidia/galaxea/r1pro-web-control-stack-pc10-20261009.tar.gz ~/Desktop/
cd ~/Desktop
tar -xzf r1pro-web-control-stack-pc10-20261009.tar.gz
```

修改的是独立副本，不会自动同步到 pc10 正在运行的服务。
