// Copyright (c) 2019 Kinova inc. All rights reserved.
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

#ifndef KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
#define KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_

#include <cmath>

class KortexMathUtil
{
public:
  KortexMathUtil() {}
  ~KortexMathUtil() {}

  static double toRad(double degree);
  static double toDeg(double rad);
  static int getNumberOfTurns(double rad_not_wrapped);
  static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);
  static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int & number_of_turns);
  static double wrapDegreesFromZeroTo360(double deg_not_wrapped);
  static double wrapDegreesFromZeroTo360(double deg_not_wrapped, int & number_of_turns);
  static double relative_position_from_absolute(
    double absolute_position, double min_value, double max_value);
  static double absolute_position_from_relative(
    double relative_position, double min_value, double max_value);
};

#endif  // KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
