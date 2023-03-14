#include "../inc/postprocessor.h"

namespace ffea::app {

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Temperature", 1, mesh);
}

}  // namespace ffea::app
