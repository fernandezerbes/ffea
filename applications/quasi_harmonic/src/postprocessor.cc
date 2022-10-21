#include "../inc/postprocessor.h"

namespace ffea {

namespace utilities {

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh) {
  return PrimaryVariablePostProcessor("Temperature", 1, mesh);
}

}  // namespace utilities

}  // namespace ffea
