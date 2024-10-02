from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    order_optimizer_node = Node(
            package='planner_cpp',
            executable='order_optimizer',
            name='order_optimizer',
            output='screen',
            parameters=[{'data_path': '/home/rudolf/ros2/kbot/data'}],
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
            arguments=['-d', '/home/rudolf/ros2/kbot/src/kbot_launch/launch/rviz_conf.rviz'],
            output='screen'
        )

    
    ld.add_action(order_optimizer_node)
    ld.add_action(test_pose_publisher_node)
    ld.add_action(test_order_publisher_node)
    ld.add_action(rviz)

    return ld