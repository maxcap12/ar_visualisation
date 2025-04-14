from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",  # Change to string
        description="port"
    )

    server_host_arg = DeclareLaunchArgument(
        "server-host",
        default_value="0.0.0.0",
        description="ip address of the client"           
    )

    server_node = Node(
        package="ar_visualisation",
        executable="server_node.py",
        name="server_node",
        arguments=[
            LaunchConfiguration("server-host"),
            LaunchConfiguration("port")
        ]
    )

    map_creator_node = Node(
        package="ar_visualisation",
        executable="map_creator",
        name="map_creator_node"
    )

    return LaunchDescription([
        port_arg,
        server_host_arg,
        port_arg,
        server_node,
        map_creator_node
    ])