from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к URDF файлу
    urdf_path = PathJoinSubstitution([
        FindPackageShare('g1_robot_description'),
        'urdf',
        'g1_29dof.urdf'  # Ваш URDF файл
    ])
    
    return LaunchDescription([
        # Публикатор состояния робота
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),
        
        # Публикатор состояний сочленений (можно заменить на реальный хардвер)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz для визуализации
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('<package_name>'),
                'rviz',
                'view_robot.rviz'  # Конфиг RViz (опционально)
            ])]
        ),
    ])
