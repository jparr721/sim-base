#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace viewer {

using Rgb8 = std::array<std::uint8_t, 3>;

// The two ends of the albedo palette, sRGB encoded. Every generated pixel
// lies between them channel by channel.
inline constexpr Rgb8 kPorcelain{238, 229, 216};
inline constexpr Rgb8 kClay{196, 158, 136};

// A square sRGB8 image, row-major, with no padding between rows.
struct Albedo {
  int size = 0;
  std::vector<Rgb8> pixels; // size * size

  [[nodiscard]] auto At(int x, int y) const -> const Rgb8 & {
    return pixels[static_cast<std::size_t>(y) * size + x];
  }
};

// Tileable fBm value noise mapped through the porcelain-to-clay palette.
// Pure: the same seed and size give the same bytes.
auto GenerateAlbedo(std::uint32_t seed, int size) -> Albedo;

} // namespace viewer
