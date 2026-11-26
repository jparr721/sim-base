#pragma once

#include <cassert>
#include <cstdio>
#include <filesystem>
#include <string>

#if defined(_MSC_VER)
#define SIM_INLINE __forceinline
#else
#define SIM_INLINE __attribute__((always_inline)) inline
#endif

// Processor architecture
#if defined(_MSC_VER) && defined(_M_X86)
#error 32-bit builds are not supported. Please run cmake-gui.exe, delete the cache, and \
         regenerate a new version of the build system that uses a 64 bit version of the compiler
#endif

#if defined(_MSC_VER) // warning C4127: conditional expression is constant
#pragma warning(disable : 4127)
#endif

namespace sim {

namespace fs = std::filesystem;

inline const fs::path Meshes = fs::path(__FILE__).parent_path().parent_path() / "Meshes";

namespace detail {
SIM_INLINE void Assert(bool condition, const char *file, int line,
                       const std::string &message) {
  if (!condition) {
    fprintf(stderr, "Assertion Failed in [%s:%d] %s", file, line,
            message.c_str());
    assert(condition);
  }
}
} // namespace detail

} // namespace sim

#define SIM_ASSERT(cond, explanation)                                          \
  do {                                                                         \
    if (!(cond))                                                               \
      ::sim::detail::Assert(cond, __FILE__, __LINE__, explanation);            \
  } while (0)
