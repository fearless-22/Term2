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
ros2 launch custom_slam slam_launch.py
```

参考手册配置rviz2窗口
打开遥控终端控制小车移动(见上文)
注意速度要很低，保证建图效果
