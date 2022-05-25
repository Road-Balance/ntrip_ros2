from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ntrip_ros',
            executable='socket_ntrip',
            parameters=[
                {'rtcm_topic': '/rtcm'},
                {'target_host': 'www.gnssdata.or.kr'},
                {'target_port': 2101},
                {'mountpoint': 'SOUL-RTCM23'},
                {'useragent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36'},
                {'user': 'tge1375@naver.com:gnss'},
                {'verbose': True},
            ],
            output='screen',
            emulate_tty=True
        )
    ])