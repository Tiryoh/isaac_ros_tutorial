# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import xacro

def generate_launch_description():
    """Launch file which brings up visual odometry node configured for RealSense."""

    realsense_type = LaunchConfiguration('realsense_type')

    declare_realsense_type_cmd = DeclareLaunchArgument(
        name='realsense_type',
        default_value='d435i',
        description='RealSense type: d415, d435, d435i')

    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': 2, #https://github.com/IntelRealSense/realsense-ros/issues/817
                'infra_fps': 90.0,
                'unite_imu_method': 'linear_interpolation' # copy | linear_interpolation
        }],
        )

    # Use OpaqueFunction to parse argument
    # ref: https://github.com/rt-net/raspimouse_ros2_examples/blob/foxy-devel/launch/teleop_joy.launch.py
    def func_launch_visual_odometry_container(context):
        visual_odometry_node = ComposableNode(
            name='visual_odometry_node',
            package='isaac_ros_visual_odometry',
            namespace='camera',
            plugin='isaac_ros::visual_odometry::VisualOdometryNode',
            parameters=[{
                'enable_rectified_pose': False,
                'denoise_input_images': False,
                'rectified_images': True,
                'enable_debug_mode': False,
                'enable_imu': True if context.launch_configurations['realsense_type'] == "d435i" else False,
                'debug_dump_path': '/tmp/elbrus',
                'left_camera_frame': 'camera_infra1_frame',
                'right_camera_frame': 'camera_infra2_frame',
                'fixed_frame': 'odom',
                'imu_frame': 'camera_imu_optical_frame',
                'current_smooth_frame': 'base_link',
                'current_rectified_frame': 'base_link_rect'
            }],
            remappings=[('stereo_camera/left/image', 'infra1/image_rect_raw'),
                        ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                        ('stereo_camera/right/image', 'infra2/image_rect_raw'),
                        ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                        ('visual_odometry/imu', 'imu')]
        )
        return [
            ComposableNodeContainer(
                name='visual_odometry_launch_container',
                namespace='camera',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    visual_odometry_node
                ],
                output='screen'
            )
        ]

    visual_odometry_launch_container = OpaqueFunction(function=func_launch_visual_odometry_container)

    def func_launch_model_node(context):
        xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_{}_camera.urdf.xacro'.format(context.launch_configurations['realsense_type']))
        parameters = {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'}
        robot_desc = xacro.process_file(xacro_path, mappings=parameters).toprettyxml(indent='  ')
        return [Node(
            name='model_node',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )]

    model_node = OpaqueFunction(function=func_launch_model_node)

    ld = LaunchDescription()
    
    ld.add_action(declare_realsense_type_cmd)
    ld.add_action(visual_odometry_launch_container)
    ld.add_action(realsense_camera_node)
    ld.add_action(model_node)

    return ld
