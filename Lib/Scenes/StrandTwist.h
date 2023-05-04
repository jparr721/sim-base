#pragma once

#include <Camera.h>
#include <memory>

class StrandTwist {
public:
  std::shared_ptr<Camera<float>> camera;
  explicit StrandTwist(std::shared_ptr<Camera<float>> &camera);

private:
};
