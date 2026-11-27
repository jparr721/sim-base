#include <AlbedoTexture.h>

#include <cmath>

namespace viewer {
namespace {

constexpr int kOctaves = 5;
constexpr int kBaseCells = 4; // lattice cells across the image at octave 0
constexpr float kGain = 0.5f;

auto Hash(std::uint32_t x, std::uint32_t y, std::uint32_t seed) -> float {
  std::uint32_t h = seed * 0x9E3779B9u ^ x * 0x85EBCA6Bu ^ y * 0xC2B2AE35u;
  h ^= h >> 16;
  h *= 0x7FEB352Du;
  h ^= h >> 15;
  h *= 0x846CA68Bu;
  h ^= h >> 16;
  return static_cast<float>(h) / 4294967296.0f;
}

auto Quintic(float t) -> float { return t * t * t * (t * (t * 6 - 15) + 10); }

auto Lerp(float a, float b, float t) -> float { return a + (b - a) * t; }

// Value noise on a lattice of `period` cells that wraps at the image edge,
// sampled at (u, v) in [0, 1).
auto ValueNoise(float u, float v, int period, std::uint32_t seed) -> float {
  const float x = u * static_cast<float>(period);
  const float y = v * static_cast<float>(period);
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const auto wrap = [period](int i) {
    return static_cast<std::uint32_t>(((i % period) + period) % period);
  };
  const std::uint32_t ix0 = wrap(x0), ix1 = wrap(x0 + 1);
  const std::uint32_t iy0 = wrap(y0), iy1 = wrap(y0 + 1);
  const float tx = Quintic(x - static_cast<float>(x0));
  const float ty = Quintic(y - static_cast<float>(y0));
  return Lerp(Lerp(Hash(ix0, iy0, seed), Hash(ix1, iy0, seed), tx),
              Lerp(Hash(ix0, iy1, seed), Hash(ix1, iy1, seed), tx), ty);
}

// Normalized to [0, 1].
auto Fbm(float u, float v, std::uint32_t seed) -> float {
  float sum = 0, amplitude = 1, total = 0;
  for (int octave = 0; octave < kOctaves; ++octave) {
    const auto octaveSeed = seed + static_cast<std::uint32_t>(octave) * 0x68E31DA4u;
    sum += amplitude * ValueNoise(u, v, kBaseCells << octave, octaveSeed);
    total += amplitude;
    amplitude *= kGain;
  }
  return sum / total;
}

auto Smoothstep(float lo, float hi, float x) -> float {
  const float t = std::fmin(std::fmax((x - lo) / (hi - lo), 0.0f), 1.0f);
  return t * t * (3 - 2 * t);
}

auto Palette(float t) -> Rgb8 {
  Rgb8 out{};
  for (int c = 0; c < 3; ++c) {
    const float value = Lerp(kPorcelain[c], kClay[c], t);
    out[c] = static_cast<std::uint8_t>(std::lround(value));
  }
  return out;
}

} // namespace

auto GenerateAlbedo(std::uint32_t seed, int size) -> Albedo {
  Albedo albedo{size, std::vector<Rgb8>(static_cast<std::size_t>(size) * size)};
  const float inverse = 1.0f / static_cast<float>(size);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      // fBm clusters around 0.5; widen it so both palette ends show.
      const float t = Smoothstep(0.2f, 0.8f, Fbm(x * inverse, y * inverse, seed));
      albedo.pixels[static_cast<std::size_t>(y) * size + x] = Palette(t);
    }
  }
  return albedo;
}

} // namespace viewer
