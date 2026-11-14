from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
from rclpy.duration import Duration
import math
from autopatrol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


def euler_from_quaternion(x, y, z, w):
    """四元数 (x, y, z, w) → 欧拉角 (roll, pitch, yaw, 弧度)，旋转顺序 ZYX"""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]

def quaternion_from_euler(roll, pitch, yaw):
    """欧拉角 (roll, pitch, yaw, 弧度) → 四元数 [x, y, z, w]，旋转顺序 ZYX"""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy

    return [x, y, z, w]

class PatrolNode(BasicNavigator):
    def __init__(self,node_name='patrol_node'):
        super().__init__(node_name)
        #声明参数
        self.declare_parameter('initial_point',[0.0,0.0,0.0])
        self.declare_parameter('target_point',[0.0,0.0,0.0,1.0,1.0,1.57])#参考下面的pose接口数据结构
        self.initial_point=self.get_parameter('initial_point').value
        self.target_point=self.get_parameter('target_point').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.speech_client=self.create_client(SpeechText,'speech_text')
        self.declare_parameter("img_save_path","")
        self.img_save_path=self.get_parameter("img_save_path").value
        self.cv_bridge=CvBridge()
        self.latest_img_=None
        self.img_sub_=self.create_subscription(Image,'/camera_sensor/image_raw',self.img_callback,1)
        
    def img_callback(self,msg):
         self.latest_img_=msg
    def record_img(self):
        if self.latest_img_ is not None:
            pose = self.get_current_pose()
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img_)
            path=f'{self.img_save_path}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png'
            cv2.imwrite(path,cv_image)
            
        
        
        
        
        
        
    def get_pose_by_xyyaw(self,x,y,yaw):
        pose=PoseStamped()
        pose.header.frame_id="map"
        pose.pose.position.x=x
        pose.pose.position.y=y
        quat=quaternion_from_euler(0,0,yaw)
        #返回顺序为xyzw
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        return pose
    def init_robot_pose(self):
        # 初始化机器人位置
        self.initial_point=self.get_parameter('initial_point').value
        init_pose=self.get_pose_by_xyyaw(self.initial_point[0],self.initial_point[1],self.initial_point[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()#wait for nav2 to be active
    # 初始化机器人位置
    def get_target_points(self):
        points=[]
        self.target_point=self.get_parameter('target_point').value
        for index in range(int(len(self.target_point)/3)):
            x=self.target_point[index*3]
            y=self.target_point[index*3+1]
            yaw=self.target_point[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'目标点{index}->{x},{y},{yaw}')
        return points
    # 获取目标点
    def nav_to_pose(self,target_point):
        self.goToPose(target_point)
        while not self.isTaskComplete():
                feedback=self.getFeedback()
                self.get_logger().info(f'剩余距离:{feedback.distance_remaining}')
                #nav.cancelTask()
        result=self.getResult()
        self.get_logger().info(f'导航结果:{result}')
        
    # 导航到目标点
    def get_current_pose(self):
        while rclpy.ok():
            try:
                result = self.buffer.lookup_transform(
                            "map", "base_footprint", rclpy.time.Duration(seconds=0)
            ,rclpy.time.Duration(seconds=1.0) )  
                transform = result.transform
                self.get_logger().info(f"平移: {transform.translation}")
                        # self.get_logger().info(f"旋转: {transform.rotation}")
                        # rotation_euler = euler_from_quaternion(
                        #     transform.rotation.x,
                        #     transform.rotation.y,
                        #     transform.rotation.z,
                        #     transform.rotation.w)
                        # self.get_logger().info(f"欧拉角: {rotation_euler}")
                return transform
            except Exception as e:
                self.get_logger().warn(f"获取失败: {e}")
        
    # 获取当前机器人位置
    
    
    
    
    def speech_text(self,text):
        
        #调用服务合成语音
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("服务未启动，等待中...")
        
        request=SpeechText.Request()
        request.text=text
        future=self.speech_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            response=future.result()
            if response.result==True:
                self.get_logger().info(f"语音合成成功{text}")
            else:
                self.get_logger().warn(f"响应失败:{text}")
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

def main():
    
    rclpy.init()
    patrol=PatrolNode()
    patrol.speech_text("正在初始化位置")
                       
    patrol.init_robot_pose()
    patrol.speech_text("机器人位置初始化完成")
    
    while rclpy.ok():
        points =patrol.get_target_points()
        for point in points:
            x,y,yaw=point[0],point[1],point[2]
            target_pose=patrol.get_pose_by_xyyaw(x,y,yaw)
            patrol.speech_text(f"正在导航到目标点({x},{y},{yaw})")
            patrol.nav_to_pose(target_pose)
            patrol.speech_text(f"已到达目标点({x},{y}),正在准备记录图像")
            patrol.record_img()
            patrol.speech_text("图像记录完成")
    rclpy.shutdown()
  
