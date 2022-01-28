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

#include <memory>
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

void SupportedTypeMap::dispatch_msg(
  const std::string & ros_type_name,
  const std::string & name,
  std::shared_ptr<rclcpp::SerializedMessage> msg) const
{
  std::string key_name = ros_type_name + "+" + name;
  if (name_to_supported_types_.count(key_name) == 0) {
    // We were asked to dispatch for a type that we don't have, so skip
    return;
  }

  // TODO(clalancette): This is bogus; what should we fill in?
  rclcpp::MessageInfo msg_info;

  SupportedTypeInfo type_info = name_to_supported_types_.at(key_name);

  std::shared_ptr<MessageContainerBase> msg_container = type_info.message_container;
  std::shared_ptr<void> msg_ptr = msg_container->get_msg_ptr();

  std::shared_ptr<rclcpp::SerializationBase> serializer = type_info.serializer;
  serializer->deserialize_message(msg.get(), msg_ptr.get());

  std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> asc = type_info.asc;
  asc->dispatch(msg_ptr, msg_info);
}

std::shared_ptr<rclcpp::SerializationBase> SupportedTypeMap::get_serializer(
  const std::string & ros_type_name,
  const std::string & name) const
{
  std::string key_name = ros_type_name + "+" + name;
  if (name_to_supported_types_.count(key_name) == 0) {
    return nullptr;
  }
  return name_to_supported_types_.at(key_name).serializer;
}

}  // namespace negotiated
