import rclpy
from rclpy.node import Node
import yaml
import json
import paho.mqtt.client as mqtt
import importlib
import time

class ROS2MQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2mqtt_bridge')

        # 1. params.ros2.yaml 파일 읽기
        config_file = '/home/edge/ROS2MQTT_Bridge/src/ros2mqtt_bridge/config/params.ros2.yaml'
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # 파라미터 가져오기
        broker_host = self.config['/**/*']['ros__parameters']['broker']['host']
        broker_port = self.config['/**/*']['ros__parameters']['broker']['port']

        # ros2mqtt 관련 설정
        ros2mqtt = self.config['/**/*']['ros__parameters']['bridge']['ros2mqtt']
        self.ros2mqtt_ros_topic = ros2mqtt['ros_topics'][0]
        self.ros2mqtt_mqtt_topic = ros2mqtt[self.ros2mqtt_ros_topic]['mqtt_topic']
        self.ros2mqtt_ros_type = ros2mqtt[self.ros2mqtt_ros_topic]['ros_type']  # ROS 메시지 타입

        # mqtt2ros 관련 설정
        mqtt2ros = self.config['/**/*']['ros__parameters']['bridge']['mqtt2ros']
        self.mqtt2ros_mqtt_topic = mqtt2ros['mqtt_topics'][0]
        self.mqtt2ros_ros_topic = mqtt2ros[self.mqtt2ros_mqtt_topic]['ros_topic']
        self.mqtt2ros_ros_type = mqtt2ros[self.mqtt2ros_mqtt_topic]['ros_type']  # ROS 메시지 타입

        # MQTT 클라이언트 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(broker_host, broker_port, 60)
        self.mqtt_client.loop_start()

        # ROS 타입을 동적으로 불러오기
        self.ros_msg_type = self.get_ros_msg_type(self.ros2mqtt_ros_type)
        self.mqtt2ros_msg_type = self.get_ros_msg_type(self.mqtt2ros_ros_type)

        # ROS 데이터 플래그
        self.ros_data_received = False

        # ROS 데이터를 기다리기 위한 타임아웃 변수
        self.start_time = time.time()
        self.ros_timeout = 10  # 10초 동안 ROS 데이터를 기다림

        # 첫 번째 구독, 데이터를 확인
        self.ros_subscription = self.create_subscription(
            self.ros_msg_type,
            self.ros2mqtt_ros_topic,
            self.first_ros_callback,
            10
        )

        # ROS 퍼블리셔 (MQTT -> ROS 전송)
        self.ros_publisher = self.create_publisher(
            self.mqtt2ros_msg_type,
            self.mqtt2ros_ros_topic,
            10
        )

        # 타이머로 1초마다 체크하여 ROS 데이터를 기다림
        self.timer = self.create_timer(1.0, self.check_for_ros_data)

    # ROS 메시지 타입을 동적으로 가져오기 위한 함수
    def get_ros_msg_type(self, ros_type_name):
        package_name, msg_name = ros_type_name.split('/')
        module = importlib.import_module(f"{package_name}.msg")
        return getattr(module, msg_name)

    # 첫 번째로 ROS에서 데이터를 구독할 때 호출되는 콜백 함수
    def first_ros_callback(self, msg):
        self.get_logger().info(f"첫 번째 ROS 데이터 수신: {msg}")
        
        # ROS 데이터를 받으면, 플래그를 설정하고 계속해서 구독
        self.ros_data_received = True

        # 첫 데이터가 있다면 계속해서 ROS 구독 후 MQTT 퍼블리시
        self.ros_subscription = self.create_subscription(
            self.ros_msg_type,
            self.ros2mqtt_ros_topic,
            self.ros_to_mqtt_callback,
            10
        )

    # 타이머로 ROS 데이터를 기다림. 타임아웃이 발생하면 MQTT 구독으로 전환
    def check_for_ros_data(self):
        current_time = time.time()
        if not self.ros_data_received and (current_time - self.start_time) > self.ros_timeout:
            self.get_logger().info("ROS 토픽에 데이터가 없습니다. MQTT 구독을 시작합니다.")
            self.mqtt_client.subscribe(self.mqtt2ros_mqtt_topic)
            # 타이머 해제
            self.timer.cancel()

    # ROS 메시지를 계속해서 확인하고, MQTT로 퍼블리시
    def ros_to_mqtt_callback(self, msg):
        self.get_logger().info(f"ROS에서 데이터 수신: {msg}")

        # ROS 메시지를 JSON으로 변환하여 MQTT로 퍼블리시
        data = {'data': msg.data}
        self.mqtt_client.publish(self.ros2mqtt_mqtt_topic, json.dumps(data))

    # MQTT 브로커 연결 시 콜백
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT 브로커에 연결되었습니다.")
        else:
            self.get_logger().error(f"MQTT 브로커에 연결 실패: {rc}")

    # MQTT로부터 데이터를 수신할 때의 콜백
    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"MQTT에서 데이터 수신: {msg.topic}")
        data = json.loads(msg.payload.decode('utf-8'))

        # MQTT 메시지를 ROS로 퍼블리시
        ros_msg = self.mqtt2ros_msg_type()
        ros_msg.data = data['data']  # 메시지 타입에 맞게 데이터를 변환
        self.ros_publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    ros2mqtt_bridge = ROS2MQTTBridge()

    try:
        rclpy.spin(ros2mqtt_bridge)  # 프로그램이 계속 실행됨
    except KeyboardInterrupt:
        pass
    finally:
        ros2mqtt_bridge.mqtt_client.loop_stop()
        ros2mqtt_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()