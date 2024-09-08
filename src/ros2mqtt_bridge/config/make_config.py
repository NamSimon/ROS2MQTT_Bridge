import os
import yaml

def create_yaml_content():
    broker_host = os.getenv('BROKER_HOST')
    broker_port = os.getenv('BROKER_PORT')
    
    # 환경 변수로부터 토픽 및 타입 정보를 가져옵니다.
    ros2mqtt_ros_topic = os.getenv('ROS2MQTT_ROS_TOPIC', '')
    ros2mqtt_mqtt_topic = os.getenv('ROS2MQTT_MQTT_TOPIC', '')
    ros2mqtt_ros_type = os.getenv('ROS2MQTT_ROS_TYPE', '')
    
    mqtt2ros_mqtt_topic = os.getenv('MQTT2ROS_MQTT_TOPIC', '')
    mqtt2ros_ros_topic = os.getenv('MQTT2ROS_ROS_TOPIC', '')
    mqtt2ros_ros_type = os.getenv('MQTT2ROS_ROS_TYPE', '')

    # YAML 구조 생성
    yaml_content = {
        "/**/*": {
            "ros__parameters": {
                "broker": {
                    "host": broker_host,
                    "port": broker_port
                },
                "bridge": {
                    "ros2mqtt": {
                        "ros_topics": [ros2mqtt_ros_topic],
                        ros2mqtt_ros_topic: {
                            "mqtt_topic": ros2mqtt_mqtt_topic,
                            "ros_type": ros2mqtt_ros_type
                        }
                    },
                    "mqtt2ros": {
                        "mqtt_topics": [mqtt2ros_mqtt_topic],
                        mqtt2ros_mqtt_topic: {
                            "ros_topic": mqtt2ros_ros_topic,
                            "ros_type": mqtt2ros_ros_type
                        }
                    }
                }
            }
        }
    }

    return yaml_content


def write_yaml_file():
    try:
        # 현재 스크립트의 위치를 기준으로 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, 'params.ros2.yaml')
        
        # YAML 파일 생성
        content = create_yaml_content()
        with open(file_path, 'w') as file:
            yaml.dump(content, file, default_flow_style=False, sort_keys=False, Dumper=yaml.SafeDumper)
        
        print(f"YAML file created at {file_path}")
        print("성공")
    
    except Exception as e:
        print(f"Failed to create YAML file at {file_path}: {e}")

if __name__ == "__main__":
    write_yaml_file()