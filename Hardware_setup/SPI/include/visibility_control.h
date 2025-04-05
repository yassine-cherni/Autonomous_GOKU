// Copyright 2021 ros2_control Development Team
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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef DIFFDRIVE_STM32F4__VISIBILITY_CONTROL_H_ // Updated
#define DIFFDRIVE_STM32F4__VISIBILITY_CONTROL_H_ // Updated

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_STM32F4_EXPORT __attribute__((dllexport)) // Updated
#define DIFFDRIVE_STM32F4_IMPORT __attribute__((dllimport)) // Updated
#else
#define DIFFDRIVE_STM32F4_EXPORT __declspec(dllexport) // Updated
#define DIFFDRIVE_STM32F4_IMPORT __declspec(dllimport) // Updated
#endif
#ifdef DIFFDRIVE_STM32F4_BUILDING_DLL // Updated
#define DIFFDRIVE_STM32F4_PUBLIC DIFFDRIVE_STM32F4_EXPORT // Updated
#else
#define DIFFDRIVE_STM32F4_PUBLIC DIFFDRIVE_STM32F4_IMPORT // Updated
#endif
#define DIFFDRIVE_STM32F4_PUBLIC_TYPE DIFFDRIVE_STM32F4_PUBLIC // Updated
#define DIFFDRIVE_STM32F4_LOCAL // Updated
#else
#define DIFFDRIVE_STM32F4_EXPORT __attribute__((visibility("default"))) // Updated
#define DIFFDRIVE_STM32F4_IMPORT // Updated
#if __GNUC__ >= 4
#define DIFFDRIVE_STM32F4_PUBLIC __attribute__((visibility("default"))) // Updated
#define DIFFDRIVE_STM32F4_LOCAL __attribute__((visibility("hidden"))) // Updated
#else
#define DIFFDRIVE_STM32F4_PUBLIC // Updated
#define DIFFDRIVE_STM32F4_LOCAL // Updated
#endif
#define DIFFDRIVE_STM32F4_PUBLIC_TYPE // Updated
#endif

#endif  // DIFFDRIVE_STM32F4__VISIBILITY_CONTROL_H_ // Updated
