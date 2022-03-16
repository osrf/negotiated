# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='negotiated_examples',
                    plugin='negotiated_examples::NegotiatedSubExample1',
                    name='negotiated_sub_example1',
                    remappings=[('example', 'downstream/example')],
                    parameters=[{
                        'string_a_weight': 1.0,
                        'int32_weight': 0.5,
                    }],
                ),
                ComposableNode(
                    package='negotiated_examples',
                    plugin='negotiated_examples::NegotiatedPubSubPairExample1',
                    name='negotiated_pub_sub_pair_example1',
                    parameters=[{
                        'pub_string_a_weight': 0.1,
                        'pub_int32_weight': -1.0,
                        'sub_string_a_weight': 1.0,
                        'sub_int32_weight': 0.5,
                    }],
                ),
                ComposableNode(
                    package='negotiated_examples',
                    plugin='negotiated_examples::NegotiatedPubExample1',
                    name='negotiated_pub_example1',
                    parameters=[{
                        'string_a_weight': 1.0,
                        'int32_weight': 0.5,
                        'string_b_weight': 0.1,
                    }],
                )
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
