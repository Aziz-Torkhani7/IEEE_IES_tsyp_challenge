#pragma once

#include "device.h"
#include <cstdint>
#include <vector>

class Actuator : public Device {
protected:
  virtual bool write(std::vector<uint8_t> bytes);
};
