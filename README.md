### 任务整合（到建图）
在linux中解压文件custom_car.tar.gz得到custom_car文件夹,把里面的custom_bot_ws文件夹单独拿出来

```
#前置：下载cartographer
sudo apt update
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

#打开终端
cd custom_bot_ws
#编译(对里面文件修改均需重新编译)
rm -rf build/install/log/
colcon build --symlink-install
#初始化
source install/setup.bash
#执行launch启动
ros2 launch custom_nav_pkg slam_launch.py
#在 RViz2 中，添加几个组件观察：
#添加 Map 组件，将 Topic 设为 /map。
#添加 LaserScan 组件，将 Topic 设为 /scan。
#添加 RobotModel 组件，以显示你的小车模型。
确保全局的 Fixed Frame 设置为 odom或者map。

#新开终端控制小车
source ~/custom_bot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

#建完保存地图(新开终端)
cd custom_bot_ws/src/custom_nav_pkg/outputs
ros2 run nav2_map_server map_saver_cli -f map_1
```





***

# 前三周任务：Gazebo+Rviz2:加载场景及机器人

## custom_bot.urdf

AI写的机器人配置文件  
可从仓库下载至本地

## 命令行启动Gazebo+Rviz2

### 终端1

``` bash
# 1. 告诉 Gazebo 去哪儿找 TB3 的模型
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# 2. 尝试启动世界 (放在后台)
source /opt/ros/humble/setup.bash #激活ros2环境,如果不想每个终端内都敲一遍,可写进.bashrc文件
killall -9 gzserver gzclient #  清除先前开启的gazebo线程
gazebo /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &

# 3.加载机器人(确保当前目录下有custom_bot.urdf文件)
# 两条命令需一起执行
ros2 run gazebo_ros spawn_entity.py -entity my_custom_bot -file custom_bot.urdf -x -2.0 -y -0.5 -z 0.2 &
ros2 run robot_state_publisher robot_state_publisher custom_bot.urdf
```

### 终端2

``` bash
# 终端 2：启动 RViz2
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

### 终端3

``` bash
# 终端 3：启动键盘控制
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# 第四周任务：输出机器人状态信息

## RobotStateMonitor.py

机器人状态信息监控及广播脚本  
加载场景及机器人后终端输入

``` bash
python3 RobotStateMonitor.py
#示例输出:
[INFO] [1774603425.211133688] [robot_state_monitor]: State Update:
{
  "pose": {
    "x": -2.0118343513338512,
    "y": -0.7696763376050626
  },
  "battery": 100.0
}
```

# 第五周任务: slam建图

## 1.安装依赖包

``` bash
# 安装核心依赖包
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot4-simulator

# 安装辅助工具 (用于 TF 调试和键盘控制)
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-teleop-twist-keyboard
```

## 2.创建slam包，配置核心文件

可参考仓库下ros2_slam_ws文件夹

## 3.启动建图

先启动仿真世界，加载小车(见上文)  
激活SLAM算法，新开终端：

``` bash
cd ros2_slam_ws
source install/setup.bash
ros2 launch custom_slam slam_launch.py
#如果要重新建图,在当前终端下按 Ctrl + C 终止进程，然后重新运行启动命令
```

参考手册配置rviz2窗口  
打开遥控终端控制小车移动(见上文)  
注意速度要很低，保证建图效果  

***

# 终版项目路线：Embodied-SimLite + ROS2 + SLAM + Nav2

参照 `课件PPT/课件_11综合项目设计与实现.pdf` 和
`课件PPT/课件_12项目测试与展示.pdf`，终版路线以 Web 端
Embodied-SimLite 仿真为物理环境，ROS2 侧完成 SLAM、前沿探索、Nav2
导航、地图评估和状态机恢复。

## 已补齐的 ROS2 工作区能力

核心代码位于：

```bash
ros2_slam_ws/src/custom_slam
```

新增内容：

- `launch/system_launch.py`：一键启动 Web 路线的 SLAM + Nav2 + 综合项目节点。
- `launch/slam_web_launch.py`：启动 Web 路线 SLAM Toolbox。
- `launch/nav2_web_launch.py`：启动 Nav2、前沿探索、地图评估、状态机。
- `scripts/exploration_node.py`：前沿检测、聚类过滤、目标评分。
- `scripts/map_evaluator.py`：地图覆盖率、前沿数量、验收状态评估。
- `scripts/state_manager.py`：IDLE / EXPLORATION / NAVIGATION / MAP_EVALUATION / ERROR / RECOVERY / FINISH 状态机。
- `config/*_web_params.yaml`：Web 路线专用参数，避免 `/clock` 缺失导致的时间同步问题。

详细对照分析见：

```bash
ros2_slam_ws/src/custom_slam/docs/project_gap_analysis.md
```

3 分钟连续演示视频脚本见：

```bash
ros2_slam_ws/src/custom_slam/docs/demo_video_3min_script.md
```

## 运行顺序

终端 1：启动 Web 仿真。

```bash
cd embodied-sim-lite
python3 ProductV1.0.py
```

浏览器打开：

```text
http://localhost:8000
```

终端 2：启动 ROS2 桥接器。

```bash
cd embodied-sim-lite
source /opt/ros/humble/setup.bash
python3 sim_ros2_bridgeV1.0.py
```

终端 3：启动终版综合系统。

```bash
cd ros2_slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select custom_slam
source install/setup.bash
ros2 launch custom_slam system_launch.py
```

演示恢复状态机：

```bash
ros2 topic pub --once /system_command std_msgs/msg/String "{data: INJECT_ERROR}"
```
