import os
import subprocess
from string import Template

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_models() -> str:
    models_pkg = 'hex_description'
    pkg_path = get_package_share_directory(models_pkg)

    xacro_path = os.path.join(pkg_path, 'models', 'hex_model.xacro')
    model_path = os.path.join(pkg_path, 'models', 'hex_model.urdf')

    subprocess.run(['xacro', xacro_path, f'-o{model_path}'], check=True)

    return model_path


def generate_rviz_config(description_path: str) -> str:
    models_pkg = 'hex_description'
    pkg_path = get_package_share_directory(models_pkg)

    template_path = os.path.join(pkg_path, 'rviz', 'show.rviz.template')
    config_path = os.path.join(pkg_path, 'rviz', 'show.rviz')

    with open(template_path, 'r') as file:
        template = Template(file.read())

    result = template.substitute({'description_path': description_path})
    with open(config_path, 'w') as file:
        file.write(result)

    return config_path


def generate_launch_description():
    description_path = generate_models()

    return LaunchDescription([
        Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            output='own_log',
        ),
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[description_path],
        ),
        Node(
            name='dummy_joint_state_publisher',
            package='hex_control',
            executable='dummy_joint_state_publisher',
            output='screen'
        )
    ])
