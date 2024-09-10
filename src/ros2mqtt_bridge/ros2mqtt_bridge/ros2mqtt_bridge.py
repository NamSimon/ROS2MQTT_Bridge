import rclpy
from rclpy.node import Node
import importlib
import time
import os
from mqtt.mqtt import MQTTClient  # MQTTClient 클래스를 불러옴

class ROS2MQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2mqtt_bridge')

        # ROS 관련 설정
        self.platform = os.getenv('PLATFORM', '')  # 플랫폼 이름
        self.ros2mqtt_ros_topic = os.getenv('ROS2MQTT_ROS_TOPIC', '')
        self.ros2mqtt_ros_type = os.getenv('ROS2MQTT_ROS_TYPE', '')  # ROS 메시지 타입
        self.mode = os.getenv('MODE', '')  # 모드 ('pub' 또는 'sub')

        # ROS 메시지 타입을 동적으로 로드
        self.ros_msg_type = self.get_ros_msg_type(self.ros2mqtt_ros_type)

        # MQTT 클라이언트를 설정하고 메시지 콜백을 등록
        self.mqtt_client = MQTTClient(self.on_mqtt_message_received)
        if self.platform == 'edge':
        # ROS 퍼블리셔 및 구독자 설정
            if self.mode == 'pub':
                self.mqtt_client.subscribe()  # MQTT 구독 시작
                # ROS -> MQTT
                self.get_logger().info("모드: pub - MQTT 구독, ROS 퍼블리시")
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    10
                )

            elif self.mode == 'sub':
                # MQTT -> ROS
                self.get_logger().info("모드: sub - MQTT 퍼블리시, ROS 구독")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    self.ros_to_mqtt_callback,
                    10
                )
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
                return
            
        elif self.platform =='user':
            # ROS 퍼블리셔 및 구독자 설정


            if self.mode == 'pub': 
                # MQTT -> ROS
                self.get_logger().info("모드: sub - MQTT 퍼블리시, ROS 구독")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    self.ros_to_mqtt_callback,
                    10
                )
                
            elif self.mode == 'sub':
                self.mqtt_client.subscribe()  # MQTT 구독 시작
                # ROS -> MQTT
                self.get_logger().info("모드: pub - MQTT 구독, ROS 퍼블리시")
                self.ros_publisher = self.create_publisher(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    10
                )

                
            else:
                self.get_logger().error("올바르지 않은 모드 설정. 'pub' 또는 'sub'만 허용됩니다.")
                return
            

    def get_ros_msg_type(self, ros_type_name):
        """ROS 메시지 타입을 동적으로 로드하는 함수."""
        if '/' not in ros_type_name:
            self.get_logger().error(f"Invalid ROS message type: {ros_type_name}")
            return None
        package_name, msg_name = ros_type_name.split('/')
        module = importlib.import_module(f"{package_name}.msg")
        return getattr(module, msg_name)

    def ros_to_mqtt_callback(self, msg):
        """ROS에서 수신된 데이터를 MQTT로 퍼블리시하는 콜백 함수."""
        self.get_logger().info(f"ROS에서 데이터 수신: {msg}")
        self.mqtt_client.publish(msg.data)

    def on_mqtt_message_received(self, data):
        """MQTT에서 수신된 메시지를 ROS로 퍼블리시."""
        self.get_logger().info("MQTT 메시지를 ROS로 퍼블리시합니다.")
        ros_msg = self.ros_msg_type()
        ros_msg.data = data['data']
        self.ros_publisher.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    ros2mqtt_bridge = ROS2MQTTBridge()

    try:
        rclpy.spin(ros2mqtt_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ros2mqtt_bridge.mqtt_client.stop()
        ros2mqtt_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()