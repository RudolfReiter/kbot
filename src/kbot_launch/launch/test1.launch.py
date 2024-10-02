from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    package_share_directory = get_package_share_directory('kbot_launch')

    param_value_arg = DeclareLaunchArgument(
        'data_path', default_value='/home/rudolf/ros2/kbot/data', description='Path to data files'
    )
    
    order_optimizer_node = Node(
            package='planner_cpp',
            executable='order_optimizer',
            name='order_optimizer',
            output='screen',
            parameters=[{'data_path': LaunchConfiguration('data_path')}],
        )

    test_pose_publisher_node = Node(
            package='planner_cpp',
            executable='position_publisher',
            name='position_publisher',
            output='screen',
        )

    test_order_publisher_node = Node(
            package='planner_cpp',
            executable='order_publisher',
            name='order_publisher',
            output='screen',
        )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', package_share_directory + '/rviz_conf.rviz'],
            output='screen'
        )

    ld.add_action(param_value_arg)
    ld.add_action(order_optimizer_node)
    ld.add_action(test_pose_publisher_node)
    ld.add_action(test_order_publisher_node)
    ld.add_action(rviz)

    return ld