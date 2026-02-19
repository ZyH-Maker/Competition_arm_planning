# 机器人搬运系统（ROS 2 + MoveIt2 + BehaviorTree）

**项目概述**
本仓库是一个 ROS 2 工作区，面向“二维码 + 转盘 + 粗加工 + 暂存区”的自动搬运流程。核心由 `main_bt` 行为树驱动，配合 MoveIt2 规划、串口控制机械臂/转盘，以及外部视觉程序（通过 Socket 接入）完成识别和抓取。

**目录结构**
- `src/main_bt` 行为树主包（BT Runner + 自定义 BT 节点 + 轨迹插值）
- `src/robot_config` MoveIt2 配置与启动文件
- `src/robot_description` 机器人 URDF/Xacro 与模型
- `src/robot_serial` 串口通信节点（控制机械臂、转盘、夹爪）
- `src/vision_node` 视觉桥接节点（Socket -> ROS 2 服务）
- `src/robot_interfaces` ROS 2 服务接口定义
- `src/bt_test_nodes` 测试用话题/服务节点
- `src/Config` 预置轨迹和任务配置（home/target/二维码）
- `start_all.sh` 一键启动脚本（打开多个终端）

**系统运行流程（主行为树）**
主流程定义在 `src/main_bt/trees/main.xml`：
- 启动与初始化
- 二维码区：启动二维码视觉、读取二维码任务序列
- 转盘区：根据任务序列抓取/放置
- 粗加工区：按任务序列放置并取回
- 暂存区：按任务序列放置
- 返回出发点并结束

各子流程在以下文件中定义：
- `src/main_bt/trees/subtrees/turntable.xml` 转盘区取放流程
- `src/main_bt/trees/subtrees/place_and_pick.xml` 粗加工区放置与取回
- `src/main_bt/trees/subtrees/place.xml` 暂存区放置

**核心包说明**
- `main_bt`
  - 可执行程序：`bt_runner`，启动后加载行为树并执行
  - 关键文件
    - `src/main_bt/src/bt_runner.cpp` 注册 BT 节点、加载 XML、构建 MoveIt 接口
    - `src/main_bt/src/bt_nodes_main.cpp` 进程管理、二维码读取、到位等待等
    - `src/main_bt/src/bt_nodes_motion.cpp` MoveIt 规划、夹爪控制、碰撞体管理
    - `src/main_bt/src/bt_nodes_comm.cpp` 视觉服务请求、任务序列下发
    - `src/main_bt/src/grasp.cpp` 轨迹规划、插值、夹爪命令
    - `src/main_bt/src/trajectory_interpolator.cpp` 关节轨迹插值
- `robot_serial`
  - 可执行程序：`serial_service_node`
  - 提供服务：`/send_command`，用于转盘/夹爪/移动指令
  - 订阅：`/display_dense_path`，将 MoveIt 轨迹点流式发送到串口
  - 发布：`/start`、`/success`（Bool），作为“开始/到位”信号
- `robot_interfaces`
  - `SendCommand.srv` 下发控制命令（移动/转盘/夹爪）
  - `GetCircles.srv` 获取视觉检测结果字符串
- `vision_node`
  - `circle_server`：监听 9091 端口，解析圆柱数据并提供 `GetCircles` 服务
  - `qr_server`：监听 9090 端口，提供二维码服务（使用 `vision_interfaces/QrCode`）
  - 说明：`vision_node` 的 `qr_server`/`qr_client` 引用了 `vision_interfaces` 包，仓库内未提供，需要外部安装或替换为本仓库接口。
- `bt_test_nodes`
  - 发布 `/start` 与 `/success` 模拟到位信号
  - 提供 `GetCircles` 测试服务（固定三圆数据）

**视觉数据格式**
`GetCircles.srv` 返回字符串格式：
- `1:red,True,0.40,-0.15,-0.0565;2:green,True,0.30,0.0,-0.0565;3:blue,True,0.30,0.15,-0.0565`
- 字段顺序：`id:color,exist,x,y,z`
- 解析逻辑在 `src/main_bt/src/communication.cpp`

**构建与运行**
以下命令在工作区根目录（本仓库）执行：

1. 构建
```bash
colcon build --symlink-install
```

2. 载入环境
```bash
source install/setup.bash
```

3. 启动 MoveIt2 Demo（可选）
```bash
ros2 launch robot_config demo.launch.py
```

4. 启动行为树主程序
```bash
ros2 launch main_bt main.launch.py
```

5. 启动串口服务（控制硬件）
```bash
ros2 run robot_serial serial_service_node --ros-args -p serial_port_1:=/dev/ttyUSB0
```

6. 启动视觉服务（Socket -> ROS 2）
```bash
ros2 run vision_node circle_server
```

7. 一键启动
```bash
./start_all.sh
```
`start_all.sh` 会自动打开多个终端窗口，依赖系统存在 `gnome-terminal`/`konsole`/`xterm`。

**重要参数与路径**
- 视觉外部程序路径（硬编码默认值）
  - `src/main_bt/src/bt_runner.cpp`
  - `qrcode_vision_cmd=/home/fins/Vision/build/Bin/Qr`
  - `turntable_vision_cmd=/home/fins/Vision/build/Bin/Detect_P`
  - `place_and_pick_vision_cmd=/home/fins/Vision/build/Bin/Detect2`
  - `buffer_vision_cmd=/home/fins/Vision/build/Bin/Detect`
- 二维码文件
  - `src/Config/Qr.yaml`
  - `ReadQRCodeFileAction` 会读取此文件的第一行并提取数字序列
- 预置轨迹文件
  - `src/Config/home.yaml` 回零位轨迹
  - `src/Config/target.yaml` 目标位轨迹
- 控制器配置
  - `src/robot_config/config/ros2_controllers.yaml`

**常见修改点**
- 调整流程
  - 修改 `src/main_bt/trees/main.xml` 与 `src/main_bt/trees/subtrees/*.xml`
- 修改夹爪/转盘命令
  - 串口指令定义在 `src/robot_serial/include/robot_serial/fin_serial.hpp`
  - 逻辑入口在 `src/robot_serial/src/serial_service_node.cpp`
- 修改视觉通信格式
  - 解析逻辑：`src/main_bt/src/communication.cpp` 中 `parseCylinderString`
- 修改预置轨迹
  - 编辑 `src/Config/home.yaml`、`src/Config/target.yaml`
- 修改机器人模型/配置
  - URDF/Xacro：`src/robot_description/urdf/`
  - MoveIt 配置：`src/robot_config/config/`

**测试与调试**
- 模拟圆检测服务
  - `ros2 run bt_test_nodes circles_service`
- 模拟到位信号
  - `ros2 run bt_test_nodes start_publisher`
  - `ros2 run bt_test_nodes success_publisher`

**已知注意事项**
- `vision_node` 中的 `qr_server`/`qr_client` 依赖 `vision_interfaces` 包，仓库内未提供。
- `ReadQRCodeFileAction` 使用固定路径 `/home/fins/robot/src/Config/Qr.yaml`，如果工作区位置变化需要同步修改。

