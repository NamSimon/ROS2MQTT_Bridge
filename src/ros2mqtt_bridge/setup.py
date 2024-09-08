from setuptools import find_packages, setup

package_name = 'ros2mqtt_bridge'

setup(
    name=package_name,
    version='0.0.1',  # 버전을 0.0.1로 변경
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edge',
    maintainer_email='cknam0708@sju.ac.kr',
    description='A ROS2 package for bridging ROS2 and MQTT communication',
    license='Apache License 2.0',  # 적절한 라이선스 추가
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2mqtt_bridge = ros2mqtt_bridge.ros2mqtt_bridge:main',  # 스크립트 실행을 위한 엔트리 포인트 추가
        ],
    },
)