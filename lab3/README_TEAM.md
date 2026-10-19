# Lab 3 — Quadrotor geometric control

依据 [MIT VNAV 2023 Lab 3](https://vnav.mit.edu/labs_2023/lab3/exercises.html) 完成。
已在本机 ARM64 Docker / ROS Noetic 中，通过 **Box64 0.3.6 运行未修改的官方
Lab 3 Unity/TESSE 模拟器**，完成悬停、调参和圆轨迹飞行。

## 本地交付

| 交付项 | 文件 |
| --- | --- |
| Deliverable 1：ROS 四元数转换 | [deliverable_1.txt](deliverable_1.txt) |
| Deliverable 2：旋翼布局秩分析、阻力补偿 | [PDF](report/deliverable_2.pdf)、[LaTeX](report/deliverable_2.tex) |
| Deliverable 3：完整控制器源码 | [controller_pkg](controller_pkg/) |
| 官方 TESSE 的 RViz 飞行录像 | [rviz_official_tesse.mp4](report/video/rviz_official_tesse.mp4) |
| 完整原始录像，含起飞过渡段 | [rviz_official_tesse_full.mp4](report/video/rviz_official_tesse_full.mp4) |
| 官方飞行轨迹、指标和说明 | [图表 PDF](report/unity_tracking.pdf)、[指标 JSON](report/unity_metrics.json)、[验证说明](report/validation.md) |
| 官方原始飞行数据 | [CSV](report/unity_flight.csv)、`report/unity_flight.bag` |
| 题目要求的录像说明文件 | [rviz_drone_flight.txt](rviz_drone_flight.txt) |

**按用户要求，所有文件只保留在本地，未上传 Google Drive / Dropbox。**
题目原本要求的公开链接已在说明文件中替换成本地录像路径；未执行远程提交或推送。

## 官方飞行结果

原定轨迹保持为 `x=5*sin(t/2), y=5*cos(t/2), z=2, yaw=-t/2`，
半径 5 米，周期 `4*pi ≈ 12.566` 秒。

| 项目 | 实测结果 |
| --- | ---: |
| 总轨迹时间 | 55.02 s |
| 稳定统计窗口 | 25.133–55.02 s，排除前两个名义周期 |
| 稳定段实际绕行 | 2.376 圈，由实际位置相位累计计算 |
| 稳定段三维位置 RMS 误差 | 0.3063 m |
| 稳定段最大三维位置误差 | 0.5845 m |
| 稳定段半径 RMS 误差 | 0.0829 m |
| 稳定段高度范围 | 1.560–1.864 m，目标 2 m |

最终参数为 `kx=2, kv=2.4, kr=1.8, komega=0.7`，控制定时器配置为 200 Hz。
由于模拟器状态发布和容器调度，实际消息频率会低于定时器配置。
本控制器采用题目允许的简化，忽略期望角速度前馈；仍存在高度偏差，未声称精确零误差。

正式录像为 1280×800、8 fps，长 27.125 秒，从完整录像的第 30 秒开始截取，
保持原始播放速度，覆盖超过一整圈稳定飞行。完整录像长 57.125 秒。
画面内的来源标记说明状态来自官方 MIT Lab 3 TESSE。

## 本机直接复用

已有容器 `ros_noetic_lab3`，Lab 3 挂载到 `/workspace/lab3`。
自包含工作区 `/tmp/lab3_unity_ws` 已编译通过，见 [构建日志](report/build_unity_noetic.txt)。
Box64 和模拟器属于容器内运行时文件，重建容器后需重新准备。

```bash
docker start ros_noetic_lab3
docker exec -it ros_noetic_lab3 bash
cd /workspace/lab3
bash scripts/build_workspace.sh
bash scripts/prepare_unity_arm64.sh

# 终端 1：保持运行，包含 roscore、Unity 和 ROS 桥接
bash scripts/start_unity_arm64.sh

# 终端 2：同一容器内，启动控制器、RViz、录屏及飞行数据记录
bash /workspace/lab3/scripts/run_unity_flight.sh
```

脚本使用 ROS master `11317`、Unity 显示 `:97`、RViz 显示 `:96`。
同一网络中一次仅运行一个 TESSE 实例，避免其固定 UDP 端口冲突。
若已存在旧的 Lab 3 运行会话，先结束其进程再运行启动脚本。
本次任务结束时已停止本次实验进程，保留共用容器供其他实验使用。

## 从 Dockerfile 重新搭建

在 ARM64 主机上，从本目录运行：

```bash
docker build -t vnav-lab3-noetic .
docker run --name vnav_lab3 --shm-size=1g -it -v "$PWD:/workspace/lab3" vnav-lab3-noetic
```

然后在容器内依次执行上述 build、prepare、start 脚本；另开宿主机终端，
执行 `docker exec -it vnav_lab3 bash`，再运行 flight 脚本。
新容器的所有包来自本目录 `dependencies/`，无需 Lab 4 工作区。
Dockerfile 已包含编译、RViz、Xvfb、FFmpeg 和 Python 依赖。
Box64 固定使用 0.3.6，下载并构建；Unity 来自
[官方 lab3.zip](https://vnav.mit.edu/material/lab3.zip)。

已有 x86_64 Noetic 环境时，也可构建同一工作区，直接启动官方 `lab3.x86_64`，
再启动 `tesse_quadrotor_bridge.launch` 和 `traj_tracking.launch`，无需 Box64。

## 控制器实现

- `src/controller_node.cpp`：ROS 订阅、参数、消息校验、超时检查和电机命令发布。
- `include/controller_pkg/geometric_controller.h`：位置反馈、SO(3) 姿态误差、力矩和旋翼分配。
- `src/traj_publisher.cpp`：官方圆轨迹及精确解析速度、加速度；`static_pose:=true` 切换悬停。
- `config/gains_unity.yaml`：官方 TESSE 最终参数，也是标准启动文件默认值。
- `config/gains.yaml`：早期独立模型的参数，仅供独立模型实验使用。

采用 ROS 向上 z 轴，`R` 从机体转到世界坐标：

```text
A = -kx*(x-xd) - kv*(v-vd) + m*g*e3 + m*ad
b3d = A / ||A||
b2d = normalize(b3d cross [cos(yawd), sin(yawd), 0])
Rd = [b2d cross b3d, b2d, b3d]
eR = 0.5 * vee(Rd^T*R - R^T*Rd)
f = A dot (R*e3)
M = -kr*eR - komega*omega_body + omega_body cross (J*omega_body)
```

TESSE 的状态角速度按题目约定为世界坐标，因此先计算
`omega_body = R.transpose()*omega_world`。
保留骨架参数 `m=1, d=0.3, cf=0.001, cd=0.00001, J=I`。
官方 Unity 内部的惯量并非单位阵；最终增益是在真实模拟器中调出的。

令 `a=d/sqrt(2)`、`s_i=omega_i*abs(omega_i)`，模拟器旋翼分配为：

```text
              [ cf     cf     cf     cf   ]
[f Mx My Mz] = [ a*cf   a*cf  -a*cf  -a*cf] s
              [-a*cf   a*cf   a*cf  -a*cf]
              [ cd    -cd     cd    -cd   ]
```

左边为列向量；求逆后取带符号平方根，输出 rad/s。
个人题图与模拟器图的旋翼编号不同，报告各自按对应图定义。
阻力补偿可通过 `compensate_drag` 开启，官方飞行默认关闭。

## 数据回放和报告再生成

```bash
source /tmp/lab3_unity_ws/devel/setup.bash
python3 scripts/analyze_unity.py

# 独立终端启动 roscore，停止实时控制和模拟器后回放：
rosparam set /use_sim_time true
rosrun rviz rviz -d controller_pkg/rviz/circle.rviz
# 另一个同 ROS master 的终端：
rosbag play --clock report/unity_flight.bag

# 数学报告
cd report
pdflatex -interaction=nonstopmode -halt-on-error deliverable_2.tex
```

`report/unity_metrics.json` 和 `unity_tracking.pdf` 只读取官方飞行 CSV。
原始 bag 保留在本地，因体积原因由 `.gitignore` 排除；CSV、图表和录像可正常提交。

## 早期独立模型记录

`scripts/validate_offline.sh`、`controller_pkg/scripts/rigid_body_demo.py`、
`report/numerical_validation.txt`、`report/ros_metrics.json` 及
`report/video/rviz_rigid_body_demo.mp4` 来自独立刚体模型，用于检查数学实现。
它们不包含 Unity 接触、电机动态，指标不能与官方飞行混用。
QEMU 4.2 / 8.2 的失败日志也保留；本次改用 Lab 4 已验证的 Box64 0.3.6
解决了 x86_64 Unity 在 ARM64 上运行的问题。
