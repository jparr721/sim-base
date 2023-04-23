#pragma once

#include <Energy/Strand/DiscreteElasticRod.h>
#include <StrandMesh.h>
#include <TimestepperStrand.h>

class ForwardEulerStrand : public TimestepperStrand {
public:
  ForwardEulerStrand(std::shared_ptr<StrandMesh> strandMesh,
                     std::shared_ptr<DiscreteElasticRod> material,
                     Real dt = 1.0 / 3000.0)
      : TimestepperStrand(std::move(strandMesh), std::move(material), dt) {}

  INLINE void Step() override {

  }
};