from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vicon_computer_ip = "192.168.2.136" # ip address of vicon computer
    port_number = "801" # default port number
    hostname = f"{vicon_computer_ip}:{port_number}"

    return LaunchDescription([
        Node(
            package='vicon_bridge',
            executable='vicon_bridge',
            name='vicon_bridge',
            # parameters = [
            #     {"hostname": hostname},
            #     {"stream_mode": "ServerPush"}, # or "ClientPull"
            #     {"update_rate_hz": 250.0},
            #     {"publish_specific_segment": False},
            #     {"world_frame_id": "world"},
            #     {"tf_namespace": "vicon"}
            #     ]
        ),
    ])
