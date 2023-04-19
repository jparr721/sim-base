#pragma once

#include <cstdio>
#include <filesystem>

#if defined(_MSC_VER)
#define EXPORT __declspec(dllexport)
#define IMPORT __declspec(dllimport)
#define NOINLINE __declspec(noinline)
#define INLINE __forceinline
#else
#define EXPORT __attribute__((visibility("default")))
#define IMPORT
#define NOINLINE __attribute__((noinline))
#define INLINE __attribute__((always_inline)) inline
#endif

#define GLOBAL static

// Reduce namespace pollution from windows.h
#if defined(_WIN32)
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#if !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif
#endif

// Processor architecture
#if defined(_MSC_VER) && defined(_M_X86)
#error 32-bit builds are not supported. Please run cmake-gui.exe, delete the cache, and \
         regenerate a new version of the build system that uses a 64 bit version of the compiler
#endif

#if defined(_MSC_VER) // warning C4127: conditional expression is constant
#pragma warning(disable : 4127)
#endif

// Namespace declarations to make life easier
namespace fs = std::filesystem;

inline const fs::path Meshes = fs::path(__FILE__).parent_path().parent_path() / "Meshes";

#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 0

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/// Current release
#define VERSION                                                                \
  TOSTRING(VERSION_MAJOR)                                                      \
  "." TOSTRING(VERSION_MINOR) "." TOSTRING(VERSION_PATCH)

/// Year of the current release
#define YEAR "2023"

/// Authors list
#define AUTHORS "Jarred Parr"

namespace detail {
INLINE void Assert(bool condition, const char *file, int line,
                   const std::string &message) {
  if (!condition) {
    fprintf(stderr, "Assertion Failed in [%s:%d] %s", file, line,
            message.c_str());
    assert(condition);
  }
}
} // namespace detail

#define ASSERT(cond, explanation)                                              \
  do {                                                                         \
    if (!(cond))                                                               \
      ::detail::Assert(cond, __FILE__, __LINE__, explanation);                 \
  } while (0)
#define ASSERT2(cond)                                                          \
  do {                                                                         \
    if (!(cond))                                                               \
      ::detail::Assert(cond, __FILE__, __LINE__, "");                          \
  } while (0)

// Assertions for impossible cases
inline constexpr bool ASSERT_ALWAYS_FALSE_V = false;
