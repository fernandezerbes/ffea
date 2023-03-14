#ifndef FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_
#define FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_

#include "../../../framework/inc/postprocessor/postprocessor.h"

namespace ffea::app {

PrimaryVariablePostProcessor MakeTemperatureProcessor(const Mesh &mesh);

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_QUASIHARMONIC_INC_POSTPROCESSOR_H_
