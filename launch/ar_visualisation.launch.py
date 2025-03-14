from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    client_host_arg = DeclareLaunchArgument(
        "client-host",
        default_value="localhost",
        description="glasses ip address"
    )

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

    client_node = Node(
        package="ar_visualisation",
        executable="client_node.py",
        name="client_node",
        arguments=[
            LaunchConfiguration("client-host"),
            LaunchConfiguration("port")
        ]
    )

    mesh_creator_node = Node(
        package="ar_visualisation",
        executable="mesh_creator",
        name="mesh_creator_node"
    )

    return LaunchDescription([
        client_host_arg,
        port_arg,
        server_host_arg,
        port_arg,
        server_node,
        client_node,
        mesh_creator_node
    ])