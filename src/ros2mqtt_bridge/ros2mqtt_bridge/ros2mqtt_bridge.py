import rclpy
from rclpy.node import Node
import importlib
import os
import json
from mqtt.mqtt import MQTTClient

class ROS2MQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2mqtt_bridge')

        # ROS 관련 설정
        self.platform = os.getenv('PLATFORM', '')  # 플랫폼 이름
        self.ros2mqtt_ros_topic = os.getenv('ROS2MQTT_ROS_TOPIC', '')
        self.ros2mqtt_ros_type = os.getenv('ROS2MQTT_ROS_TYPE', '')  # ROS 메시지 타입
        self.mode = os.getenv('MODE')  # 모드 ('pub' 또는 'sub')

        # ROS 메시지 타입을 동적으로 로드
        self.ros_msg_type = self.get_ros_msg_type(self.ros2mqtt_ros_type)

        # MQTT 클라이언트를 설정하고 메시지 콜백을 등록
        self.mqtt_client = MQTTClient(self.on_mqtt_message_received)

        if self.platform == 'edge':
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
            
        elif self.platform == 'user':
            if self.mode == 'pub': 
                # MQTT -> ROS
                self.get_logger().info("모드: pub - MQTT 구독, ROS 퍼블리시")
                self.ros_subscription = self.create_subscription(
                    self.ros_msg_type,
                    self.ros2mqtt_ros_topic,
                    self.ros_to_mqtt_callback,
                    10
                )
                
            elif self.mode == 'sub':
                self.mqtt_client.subscribe()  # MQTT 구독 시작
                # ROS -> MQTT
                self.get_logger().info("모드: sub - MQTT 퍼블리시, ROS 구독")
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
        if '/msg/' in ros_type_name:
            ros_type_name = ros_type_name.replace('/msg/', '/')
        
        package_name, msg_name = ros_type_name.split('/')
        
        try:
            module = importlib.import_module(f"{package_name}.msg")
            msg_type = getattr(module, msg_name, None)

            if msg_type is None:
                self.get_logger().error(f"Message type {ros_type_name} not found in package {package_name}")
                return None
            
            if not hasattr(msg_type, '_TYPE_SUPPORT'):
                self.get_logger().error(f"Message type {ros_type_name} is missing _TYPE_SUPPORT. Ensure it's a valid ROS 2 message.")
                return None

            return msg_type
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import ROS message type: {ros_type_name}. Error: {e}")
            return None

    def ros_to_mqtt_callback(self, msg):
        """ROS에서 수신된 데이터를 MQTT로 퍼블리시하는 콜백 함수."""
        self.get_logger().info(f"ROS에서 데이터 수신: {msg}")

        # ROS 메시지를 JSON으로 변환하여 MQTT로 전송
        json_data = self.ros_msg_to_json(msg)
        self.mqtt_client.publish(json_data)

    def ros_msg_to_json(self, msg):
        """ROS 메시지를 JSON으로 변환하는 함수."""
        msg_dict = self.ros_msg_to_dict(msg)
        return json.dumps(msg_dict)  # JSON 문자열로 변환

    def on_mqtt_message_received(self, mqtt_message):
        """MQTT에서 수신된 메시지를 ROS로 퍼블리시."""
        self.get_logger().info("MQTT 메시지를 ROS로 퍼블리시합니다.")
        
        # MQTT 메시지에서 페이로드 가져오기
        json_data = mqtt_message.payload.decode('utf-8')

        # 이미 딕셔너리일 경우 변환하지 않고, 문자열이면 JSON으로 파싱
        try:
            data = json.loads(json_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 디코딩 오류: {e}")
            return

        # 딕셔너리를 ROS 메시지로 변환
        ros_msg = self.ros_msg_type()
        self.dict_to_ros_msg(ros_msg, data)
        
        # ROS 퍼블리시
        self.ros_publisher.publish(ros_msg)

    def ros_msg_to_dict(self, msg):
        """ROS 메시지를 딕셔너리로 변환하는 함수."""
        data = {}
        for field_name in msg.__slots__:
            value = getattr(msg, field_name)
            if hasattr(value, '__slots__'):  # 서브 메시지 (예: Vector3)의 경우
                # 서브 메시지일 경우 재귀적으로 딕셔너리로 변환
                data[field_name] = self.ros_msg_to_dict(value)
            elif isinstance(value, list):  # 리스트(배열) 처리
                # 배열의 경우, 각각의 요소를 재귀적으로 처리
                data[field_name] = [self.ros_msg_to_dict(v) if hasattr(v, '__slots__') else v for v in value]
            else:
                # 기본 타입의 경우 그대로 할당
                data[field_name] = value
        return data

    def dict_to_ros_msg(self, ros_msg, data):
        """딕셔너리를 ROS 메시지로 변환하는 함수."""
        for field_name, value in data.items():
            if hasattr(ros_msg, field_name):
                field = getattr(ros_msg, field_name)
                
                # 서브 메시지일 경우 재귀적으로 처리
                if hasattr(field, '__slots__'):
                    self.dict_to_ros_msg(field, value)
                elif isinstance(value, list):  # 배열 타입 처리
                    # 배열의 경우, 각각의 요소를 재귀적으로 처리
                    for i, val in enumerate(value):
                        if hasattr(field[i], '__slots__'):
                            self.dict_to_ros_msg(field[i], val)
                        else:
                            field[i] = val
                else:
                    # 필드 타입에 맞춰 값을 설정
                    if isinstance(field, str):
                        setattr(ros_msg, field_name, str(value))
                    elif isinstance(field, float):
                        setattr(ros_msg, field_name, float(value))
                    elif isinstance(field, int):
                        setattr(ros_msg, field_name, int(value))
                    else:
                        setattr(ros_msg, field_name, value)

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