#pragma once
#include "actuator.h"
#include "sensor.h"
#include <vector>

class Controller {
public:
  Controller();
  Controller(std::vector<Actuator> actuators, std::vector<Sensor> sensors)
      : actuators(std::move(actuators)), sensors(std::move(sensors)) {}
  Sensor &getSensor(uint32_t id);
  Actuator &getActuator(uint32_t id);

private:
  const std::vector<Actuator> actuators;
  const std::vector<Sensor> sensors;
};
