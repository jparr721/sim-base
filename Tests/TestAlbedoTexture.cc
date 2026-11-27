#include <AlbedoTexture.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>

using viewer::Albedo;
using viewer::GenerateAlbedo;
using viewer::Rgb8;

namespace {

constexpr int kSize = 512;
constexpr int kEdgeEpsilon = 8; // sRGB levels

auto MaxChannelDifference(const Rgb8 &a, const Rgb8 &b) -> int {
  int largest = 0;
  for (int c = 0; c < 3; ++c) {
    largest = std::max(largest, std::abs(int(a[c]) - int(b[c])));
  }
  return largest;
}

} // namespace

TEST(AlbedoTexture, SameSeedGivesSameBytes) {
  const Albedo first = GenerateAlbedo(7, kSize);
  const Albedo second = GenerateAlbedo(7, kSize);
  ASSERT_EQ(first.pixels.size(), std::size_t{kSize} * kSize);
  EXPECT_EQ(first.pixels, second.pixels);
  EXPECT_NE(first.pixels, GenerateAlbedo(8, kSize).pixels);
}

TEST(AlbedoTexture, EdgesMatchTheirWrappedNeighbors) {
  const Albedo albedo = GenerateAlbedo(7, kSize);
  for (int ii = 0; ii < kSize; ++ii) {
    EXPECT_LE(MaxChannelDifference(albedo.At(kSize - 1, ii), albedo.At(0, ii)),
              kEdgeEpsilon)
        << "row " << ii;
    EXPECT_LE(MaxChannelDifference(albedo.At(ii, kSize - 1), albedo.At(ii, 0)),
              kEdgeEpsilon)
        << "column " << ii;
  }
}

TEST(AlbedoTexture, StaysInsideThePalette) {
  const Albedo albedo = GenerateAlbedo(7, kSize);
  for (int c = 0; c < 3; ++c) {
    const auto lo = std::min(viewer::kPorcelain[c], viewer::kClay[c]);
    const auto hi = std::max(viewer::kPorcelain[c], viewer::kClay[c]);
    const auto [darkest, brightest] = std::ranges::minmax(
        albedo.pixels, {}, [c](const Rgb8 &p) { return p[c]; });
    EXPECT_GE(darkest[c], lo) << "channel " << c;
    EXPECT_LE(brightest[c], hi) << "channel " << c;
    // Visible variation, not a flat fill.
    EXPECT_GT(brightest[c] - darkest[c], (hi - lo) / 2) << "channel " << c;
  }
}
