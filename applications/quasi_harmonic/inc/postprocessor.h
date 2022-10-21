#ifndef FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_
#define FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_

#include "../../../framework/inc/postprocessor/postprocessor.h"

namespace ffea {

namespace utilities {

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh);

}  // namespace utilities

}  // namespace ffea

#endif  // FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_
