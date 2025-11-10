#include "manager.h"
#include <cstddef>

void Manager::poll() {
  // Sleep call should be added to lessen power consumption.
  for (std::size_t i = 0; i < this->modules.size(); i++) {
    if (modules[i].condition(controller)) {
      modules[i].poll(controller);
    }
  }
}