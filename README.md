# 1.custom_bot.urdf
AI写的机器人配置文件
## 加载Turtlebot3场景及机器人(前三周任务)
### 终端1
``` bash
# 1. 告诉 Gazebo 去哪儿找 TB3 的模型
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# 2. 尝试启动世界 (放在后台)
source /opt/ros/humble/setup.bash #激活ros2环境,如果不想每个终端内都敲一遍,可写进.bashrc文件
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


