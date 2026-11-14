import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
from rclpy.duration import Duration
import math



def euler_from_quaternion(x, y, z, w):
    """四元数 (x, y, z, w) → 欧拉角 (roll, pitch, yaw, 弧度)
    与 quaternion_from_euler 互逆，旋转顺序 ZYX
    """
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)  # 限幅 +90°/-90°
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


class TfListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
       
        self.create_timer(1, self.listen_tf)

    def listen_tf(self):
        try:
            result = self.buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Duration(seconds=0)
,rclpy.time.Duration(seconds=1.0) )  
            transform = result.transform
            self.get_logger().info(f"平移: {transform.translation}")
            self.get_logger().info(f"旋转: {transform.rotation}")
            rotation_euler = euler_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w)
            self.get_logger().info(f"欧拉角: {rotation_euler}")
        except Exception as e:
            self.get_logger().warn(f"获取失败: {e}")
        
        
def main():
    rclpy.init()
    node = TfListener()
    rclpy.spin(node)
    rclpy.shutdown()