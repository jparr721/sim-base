#pragma once

#include "VolumeIntegrator.h"

namespace sim {

template <typename T> class BackwardEulerVolume : public VolumeIntegrator {
public:
  BackwardEulerVolume(std::shared_ptr<TetMesh> tetMesh,
                Material material,
                T dt = 1.0 / 60.0, T rayleighAlpha = 0.0, T rayleighBeta = 0.0)
      : VolumeIntegrator(std::move(tetMesh), std::move(material), dt,
                       rayleighAlpha, rayleighBeta) {}

  SIM_INLINE void Step() override {

  }

private:
};

} // namespace sim
