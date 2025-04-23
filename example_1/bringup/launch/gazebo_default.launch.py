# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Whether to use SLAM')
    )

    # Add ground plane parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            'ground_mu',
            default_value='0.01',
            description='Ground plane primary friction coefficient')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ground_mu2',
            default_value='0.01',
            description='Ground plane secondary friction coefficient')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ground_slip1',
            default_value='1.0',
            description='Ground plane slip1 value')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ground_slip2',
            default_value='1.0',
            description='Ground plane slip2 value')
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ground_size',
            default_value='200',
            description='Ground plane size')
    )

    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format', default='sdf').perform(context)

        # Create a launch argument for the xacro file   
        xacro_file = LaunchConfiguration('xacro_file', default='test_ackermann_drive').perform(context)

        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('gz_ros2_control_demos'),
                    performed_description_format,
                    f'{xacro_file}.xacro.{performed_description_format}'
                ]),
                ' ',
                'ground_mu:=',
                LaunchConfiguration('ground_mu', default='0.01'),
                ' ',
                'ground_mu2:=',
                LaunchConfiguration('ground_mu2', default='0.01'),
                ' ',
                'ground_slip1:=',
                LaunchConfiguration('ground_slip1', default='1.0'),
                ' ',
                'ground_slip2:=',
                LaunchConfiguration('ground_slip2', default='1.0'),
                ' ',
                'ground_size:=',
                LaunchConfiguration('ground_size', default='200'),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('gz_ros2_control_demos'),
            'config',
            'ackermann_drive_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ackermann', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /ackermann_steering_controller/tf_odometry:=/tf',
                   ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ld = LaunchDescription([
        bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ] + declared_arguments)
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
