import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformListener, TransformException
import json

class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('robot_state_monitor') # [cite: 240]
        
        # 1. TF Buffer & Listener Setup
        self.tf_buffer = Buffer() # 缓存变换数据 [cite: 242, 243]
        self.tf_listener = TransformListener(self.tf_buffer, self) # [cite: 244]
        
        # 2. 初始化共享字典 (Data Storage) [cite: 245, 246]
        self.robot_state = {
            'pose': {'x': 0.0, 'y': 0.0},
            'battery': 100.0
        } # [cite: 263, 264, 266, 751]

        # 3. 传感器数据订阅
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # [cite: 256]
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10) # [cite: 257]

        # 创建一个定时器，周期性处理和发布状态 (例如 1Hz)
        self.timer = self.create_timer(1.0, self.update_loop)

    def odom_callback(self, msg): # [cite: 260]
        # 更新里程计的 xy 坐标
        self.robot_state['pose']['x'] = msg.pose.pose.position.x # [cite: 264]
        self.robot_state['pose']['y'] = msg.pose.pose.position.y # [cite: 266]

    def battery_callback(self, msg):
        # 更新电池状态 (假设使用 percentage 字段)
        self.robot_state['battery'] = msg.percentage * 100

    def update_loop(self):
        # 4. TF 一致性检查与查询 [cite: 272]
        try:
            # 关键点：获取最新可用变换 (Time=0) [cite: 281, 284]
            t = self.tf_buffer.lookup_transform(
                'odom',          # Target Frame [cite: 639]
                'base_link',    # Source Frame [cite: 640]
                rclpy.time.Time() # Time=0 获取最新 [cite: 288, 641]
            )
            # 可选：将全局 TF 坐标也存入 JSON
            # global_x = t.transform.translation.x
            # global_y = t.transform.translation.y
            
        except TransformException as ex: # [cite: 289]
            self.get_logger().info(f'Could not transform: {ex}') # [cite: 290]
            return # 如果没有 TF，可能不想发布不完整的数据

        # 5. Web 接口数据序列化 (JSON) [cite: 294]
        state_json = json.dumps(self.robot_state, indent=2) # [cite: 298]
        
        # 打印日志验证输出 [cite: 298]
        self.get_logger().info(f'State Update:\n{state_json}') # [cite: 298]

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()