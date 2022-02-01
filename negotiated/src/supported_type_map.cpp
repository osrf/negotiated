// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/supported_type_map.hpp"

namespace negotiated
{

negotiated_interfaces::msg::SupportedTypes SupportedTypeMap::get_types() const
{
  auto ret = negotiated_interfaces::msg::SupportedTypes();
  for (const std::pair<const std::string, SupportedTypeInfo> & pair : name_to_supported_types_) {
    ret.supported_types.push_back(pair.second.supported_type);
  }

  return ret;
}

std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)>
SupportedTypeMap::get_sub_factory(
  const std::string & ros_type_name,
  const std::string & format_match) const
{
  std::string key_name = ros_type_name + "+" + format_match;
  if (name_to_supported_types_.count(key_name) == 0) {
    return nullptr;
  }
  return name_to_supported_types_.at(key_name).sub_factory;
}

std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)>
SupportedTypeMap::get_pub_factory(
  const std::string & ros_type_name,
  const std::string & format_match) const
{
  std::string key_name = ros_type_name + "+" + format_match;
  if (name_to_supported_types_.count(key_name) == 0) {
    return nullptr;
  }
  return name_to_supported_types_.at(key_name).pub_factory;
}

}  // namespace negotiated
