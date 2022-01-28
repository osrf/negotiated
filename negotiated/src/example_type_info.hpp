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

#ifndef NEGOTIATED__EXAMPLE_TYPE_INFO_HPP_
#define NEGOTIATED__EXAMPLE_TYPE_INFO_HPP_

#include <string>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace negotiated_examples
{

struct StringT
{
  using MsgT = std_msgs::msg::String;
  const static inline std::string ros_type = "std_msgs/msg/String";
  const static inline std::string name = "a";
};

struct Int32T
{
  using MsgT = std_msgs::msg::Int32;
  const static inline std::string ros_type = "std_msgs/msg/Int32";
  const static inline std::string name = "b";
};

}  // namespace negotiated_examples

#endif  // NEGOTIATED__EXAMPLE_TYPE_INFO_HPP_
