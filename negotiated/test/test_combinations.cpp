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

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "negotiated/combinations.hpp"

TEST(Combinations, BoundRange)
{
  auto func =
    [](std::vector<std::string>::iterator first, std::vector<std::string>::iterator last) -> bool
    {
      std::vector<std::string>::iterator it = first;
      if (*it != "a") {
        return false;
      }
      it++;
      if (it != last) {
        return false;
      }
      if (*it != "b") {
        return false;
      }
      return true;
    };

  std::vector<std::string> string_list{"a", "b"};

  BoundRange br(func, string_list.begin(), string_list.begin() + 1);
  ASSERT_TRUE(br());
}

TEST(Combinations, rotate_discontinuous_mid_distance_less)
{
  std::vector<std::string> string_list{"a", "b", "c"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 1;
  std::vector<std::string>::iterator last = string_list.end();

  rotate_discontinuous(
    first, mid, std::distance(first, mid),
    mid, last, std::distance(mid, last));

  ASSERT_EQ(string_list.size(), 3u);
  ASSERT_EQ(string_list[0], "b");
  ASSERT_EQ(string_list[1], "c");
  ASSERT_EQ(string_list[2], "a");
}

TEST(Combinations, rotate_discontinuous_mid_distance_more)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 3;
  std::vector<std::string>::iterator last = string_list.end();

  rotate_discontinuous(
    first, mid, std::distance(first, mid),
    mid, last, std::distance(mid, last));

  ASSERT_EQ(string_list.size(), 4u);
  ASSERT_EQ(string_list[0], "d");
  ASSERT_EQ(string_list[1], "a");
  ASSERT_EQ(string_list[2], "b");
  ASSERT_EQ(string_list[3], "c");
}

TEST(Combinations, combine_discontinuous_zero_distance_d1)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin();
  std::vector<std::string>::iterator last = string_list.end();

  auto func =
    [](std::vector<std::string>::iterator first, std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 0u);
  ASSERT_NE(std::distance(mid, last), 0u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "a");
  ASSERT_EQ(string_list[1], "b");
  ASSERT_EQ(string_list[2], "c");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_zero_distance_d2)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.end();
  std::vector<std::string>::iterator last = string_list.end();

  auto func =
    [](std::vector<std::string>::iterator first, std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_NE(std::distance(first, mid), 0u);
  ASSERT_EQ(std::distance(mid, last), 0u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "a");
  ASSERT_EQ(string_list[1], "b");
  ASSERT_EQ(string_list[2], "c");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_one_distance_d1_f_return_true)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 1;
  std::vector<std::string>::iterator last = string_list.end();

  auto func =
    [](std::vector<std::string>::iterator first, std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 1u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "a");
  ASSERT_EQ(string_list[1], "b");
  ASSERT_EQ(string_list[2], "c");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_one_distance_d1_f_return_false)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 1;
  std::vector<std::string>::iterator last = string_list.end();

  int count = 0;
  auto func =
    [&count](std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      if (count++ == 0) {
        return false;
      }
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 1u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "b");
  ASSERT_EQ(string_list[1], "a");
  ASSERT_EQ(string_list[2], "c");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_large_distance_no_swap)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 2;
  std::vector<std::string>::iterator last = string_list.end();

  int count = 0;
  auto func =
    [&count](std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      if (count++ == 0) {
        return false;
      }
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 2u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "a");
  ASSERT_EQ(string_list[1], "c");
  ASSERT_EQ(string_list[2], "b");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_large_distance_swap)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 2;
  std::vector<std::string>::iterator last = string_list.end();

  int count = 0;
  auto func =
    [&count](std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      if (count++ < 3) {
        return false;
      }
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 2u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "b");
  ASSERT_EQ(string_list[1], "c");
  ASSERT_EQ(string_list[2], "a");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, combine_discontinuous_large_distance_swap_last_element)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 2;
  std::vector<std::string>::iterator last = string_list.end();

  int count = 0;
  auto func =
    [&count](std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      if (count++ < 4) {
        return false;
      }
      return true;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 2u);
  ASSERT_TRUE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "b");
  ASSERT_EQ(string_list[1], "d");
  ASSERT_EQ(string_list[2], "a");
  ASSERT_EQ(string_list[3], "c");
}

TEST(Combinations, combine_discontinuous_large_distance_rotate_zero)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator mid = string_list.begin() + 2;
  std::vector<std::string>::iterator last = string_list.end();

  auto func =
    [](std::vector<std::string>::iterator first, std::vector<std::string>::iterator last) -> bool
    {
      (void)first;
      (void)last;
      return false;
    };

  BoundRange br(func, first, mid);

  ASSERT_EQ(std::distance(first, mid), 2u);
  ASSERT_FALSE(
    combine_discontinuous(
      first, mid, std::distance(first, mid),
      mid, last, std::distance(mid, last),
      br));
  ASSERT_EQ(string_list[0], "a");
  ASSERT_EQ(string_list[1], "b");
  ASSERT_EQ(string_list[2], "c");
  ASSERT_EQ(string_list[3], "d");
}

TEST(Combinations, for_each_combination)
{
  std::vector<std::string> string_list{"a", "b", "c", "d"};

  std::vector<std::string>::iterator first = string_list.begin();
  std::vector<std::string>::iterator last = string_list.end();

  std::vector<std::vector<std::string>> pairs;

  auto func =
    [&pairs](std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
    {
      std::vector<std::string> set(first, last);
      std::sort(set.begin(), set.end());
      pairs.push_back(set);
      return false;
    };

  {
    // Test the one-long combinations
    pairs.clear();
    for_each_combination(first, string_list.begin() + 1, last, func);
    ASSERT_EQ(pairs.size(), 4u);
    std::vector<std::string> expected{"a", "b", "c", "d"};
    for (size_t i = 0; i < pairs.size(); ++i) {
      ASSERT_EQ(pairs[i].size(), 1u);
      std::string actual_output;
      for (const std::string & j : pairs[i]) {
        actual_output += j;
      }
      ASSERT_EQ(actual_output, expected[i]);
    }
    ASSERT_EQ(string_list[0], "a");
    ASSERT_EQ(string_list[1], "b");
    ASSERT_EQ(string_list[2], "c");
    ASSERT_EQ(string_list[3], "d");
  }

  {
    // Test the two-long combinations
    pairs.clear();
    for_each_combination(first, string_list.begin() + 2, last, func);
    ASSERT_EQ(pairs.size(), 6u);
    std::vector<std::string> expected{"ab", "ac", "ad", "bc", "bd", "cd"};
    for (size_t i = 0; i < pairs.size(); ++i) {
      ASSERT_EQ(pairs[i].size(), 2u);
      std::string actual_output;
      for (const std::string & j : pairs[i]) {
        actual_output += j;
      }
      ASSERT_EQ(actual_output, expected[i]);
    }
    ASSERT_EQ(string_list[0], "a");
    ASSERT_EQ(string_list[1], "b");
    ASSERT_EQ(string_list[2], "c");
    ASSERT_EQ(string_list[3], "d");
  }

  {
    // Test the three-long combinations
    pairs.clear();
    for_each_combination(first, string_list.begin() + 3, last, func);
    ASSERT_EQ(pairs.size(), 4u);
    std::vector<std::string> expected{"abc", "abd", "acd", "bcd"};
    for (size_t i = 0; i < pairs.size(); ++i) {
      ASSERT_EQ(pairs[i].size(), 3u);
      std::string actual_output;
      for (const std::string & j : pairs[i]) {
        actual_output += j;
      }
      ASSERT_EQ(actual_output, expected[i]);
    }
    ASSERT_EQ(string_list[0], "a");
    ASSERT_EQ(string_list[1], "b");
    ASSERT_EQ(string_list[2], "c");
    ASSERT_EQ(string_list[3], "d");
  }

  {
    // Test the four-long combinations
    pairs.clear();
    for_each_combination(first, string_list.begin() + 4, last, func);
    ASSERT_EQ(pairs.size(), 1u);
    std::vector<std::string> expected{"abcd"};
    for (size_t i = 0; i < pairs.size(); ++i) {
      ASSERT_EQ(pairs[i].size(), 4u);
      std::string actual_output;
      for (const std::string & j : pairs[i]) {
        actual_output += j;
      }
      ASSERT_EQ(actual_output, expected[i]);
    }
    ASSERT_EQ(string_list[0], "a");
    ASSERT_EQ(string_list[1], "b");
    ASSERT_EQ(string_list[2], "c");
    ASSERT_EQ(string_list[3], "d");
  }
}
