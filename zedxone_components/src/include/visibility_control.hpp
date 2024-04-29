// Copyright 2024 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VISIBILITY_CONTROL_HPP_
#define VISIBILITY_CONTROL_HPP_

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ZEDXONE_COMPONENTS_EXPORT __attribute__((dllexport))
#define ZEDXONE_COMPONENTS_IMPORT __attribute__((dllimport))
#else
#define ZEDXONE_COMPONENTS_EXPORT __declspec(dllexport)
#define ZEDXONE_COMPONENTS_IMPORT __declspec(dllimport)
#endif
#ifdef ZEDXONE_COMPONENTS_BUILDING_DLL
#define ZEDXONE_COMPONENTS_PUBLIC ZEDXONE_COMPONENTS_EXPORT
#else
#define ZEDXONE_COMPONENTS_PUBLIC ZEDXONE_COMPONENTS_IMPORT
#endif
#define ZEDXONE_COMPONENTS_PUBLIC_TYPE ZEDXONE_COMPONENTS_PUBLIC
#define ZEDXONE_COMPONENTS_LOCAL
#else
#define ZEDXONE_COMPONENTS_EXPORT __attribute__((visibility("default")))
#define ZEDXONE_COMPONENTS_IMPORT
#if __GNUC__ >= 4
#define ZEDXONE_COMPONENTS_PUBLIC __attribute__((visibility("default")))
#define ZEDXONE_COMPONENTS_LOCAL __attribute__((visibility("hidden")))
#else
#define ZEDXONE_COMPONENTS_PUBLIC
#define ZEDXONE_COMPONENTS_LOCAL
#endif
#define ZEDXONE_COMPONENTS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */

#endif  // VISIBILITY_CONTROL_HPP_
