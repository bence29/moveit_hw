# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch_ros.actions import Node
# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='moveit_hw',
#             executable='moveit_hw',
#             name='moveit_hw',
#             parameters=[os.path.join(get_package_share_directory('moveit_hw'), 'config/hw_params.yaml')]
#         )
#     ])


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_hw',
            executable='moveit_hw',
            name='moveit_hw',
            parameters=[os.path.join(get_package_share_directory('moveit_hw'), 'config/hw_params.yaml'),
                {
                    "robot_description_kinematics": {
                        "manipulator": {
                            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"
                        }
                    }
                }
            ]
        )
    ])