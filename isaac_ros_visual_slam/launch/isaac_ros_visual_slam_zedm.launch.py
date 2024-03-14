# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for RealSense."""
    # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
    camera_model = 'zedm'

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    # 'input_left_camera_frame': camera_model+'_left_camera_frame',
                    # 'input_right_camera_frame': camera_model+'_right_camera_frame',
                    'input_imu_frame': camera_model+'_imu_link',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.00031087587083255885 ,
                    'gyro_random_walk': 0.00001055941968559931,
                    'accel_noise_density': 0.003907644774014534,
                    'accel_random_walk': 0.00043090053285131967 ,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 35.00
                    }],
        remappings=[('stereo_camera/left/image', 'zed_node/left/image_rect_color_rgb'),
                    ('stereo_camera/left/camera_info', 'zed_node/left/camera_info'),
                    ('stereo_camera/right/image',
                     'zed_node/right/image_rect_color_rgb'),
                    ('stereo_camera/right/camera_info',
                     'zed_node/right/camera_info'),
                    ('visual_slam/imu', 'zed_node/imu/data')]
    )

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', 'zed_node/left/image_rect_color'),
            ('image', 'zed_node/left/image_rect_color_rgb')]
    )

    image_format_converter_node_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_right',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', 'zed_node/right/image_rect_color'),
            ('image', 'zed_node/right/image_rect_color_rgb')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            image_format_converter_node_left,
            image_format_converter_node_right,
            visual_slam_node
        ],
        output='screen'
    )

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('isaac_ros_visual_slam'),
        'config',
        'zedm.yaml'
    )

    # ZED node using manual composition
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            {'general.camera_model': camera_model,
             'general.camera_name': camera_model}
        ]
    )

    # Adding delay because isaac_ros_visual_slam requires
    # tf from rsp_node at start
    return launch.LaunchDescription([
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[visual_slam_launch_container]
        ),
        zed_node])