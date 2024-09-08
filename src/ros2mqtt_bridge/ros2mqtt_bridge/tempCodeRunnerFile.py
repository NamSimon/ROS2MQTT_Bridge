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
        self.ros2mqtt_ros_topic = os.getenv('ROS2MQTT_ROS_TOPIC', '')
        self.ros2mqtt_ros_type = os.getenv('ROS2MQTT_ROS_TYPE', '')  # ROS 메시지 타입

        # ROS 메시지 타입을 동적으로 로드
        self.ros_msg_type = self.get_ros_msg_type(self.ros2mqtt_ros_type)

        # MQTT 클라이언트를 설정하고 메시지 콜백을 등록
        self.mqtt_client = MQTTClient(self.on_mqtt_message_received)

        # ROS 데이터 플래그와 타임아웃 설정
        self.ros_data_received = False
        self.ros_timeout = 5  # ROS 데이터를 기다리는 시간 (초)
        self.start_time = time.time()

        # ROS 구독 설정 (첫 번째 메시지 확인용)
        self.ros_subscription = self.create_subscription(
            self.ros_msg_type,
            self.ros2mqtt_ros_topic,
            self.first_ros_callback,
            10
        )

        # MQTT -> ROS 전송을 위한 퍼블리셔 설정
        self.ros_publisher = self.create_publisher(
            self.ros_msg_type,
            self.ros2mqtt_ros_topic,
            10
        )

        # 1초마다 ROS 데이터를 기다리기 위한 타이머
        self.timer = self.create_timer(1.0, self.check_for_ros_data)

    def get_ros_msg_type(self, ros_type_name):
        """ROS 메시지 타입을 동적으로 로드하는 함수."""
        package_name, msg_name = ros_type_name.split('/')
        module = importlib.import_module(f"{package_name}.msg")
        return getattr(module, msg_name)

    def first_ros_callback(self, msg):
        """첫 번째로 수신된 ROS 메시지의 콜백 함수."""
        self.get_logger().info(f"첫 번째 ROS 데이터 수신: {msg}")
        self.ros_data_received = True

        # 데이터가 수신되면 지속적으로 구독 및 MQTT 퍼블리시 설정
        self.ros_subscription = self.create_subscription(
            self.ros_msg_type,
            self.ros2mqtt_ros_topic,
            self.ros_to_mqtt_callback,
            10
        )

    def check_for_ros_data(self):
        """ROS 데이터를 기다리고, 타임아웃 시 MQTT 구독을 시작하는 함수."""
        if not self.ros_data_received and (time.time() - self.start_time) > self.ros_timeout:
            self.get_logger().info("ROS 토픽에 데이터가 없습니다. MQTT 구독을 시작합니다.")
            self.mqtt_client.subscribe()
            self.timer.cancel()

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