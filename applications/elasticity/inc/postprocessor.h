#ifndef FFEA_APPLICATIONS_ELASTICITY_INC_POSTPROCESSOR_H_
#define FFEA_APPLICATIONS_ELASTICITY_INC_POSTPROCESSOR_H_

#include "../../../framework/inc/postprocessor/postprocessor.h"

namespace ffea::app {

PrimaryVariablePostProcessor MakeDisplacementProcessor2D(const Mesh &mesh);

PrimaryVariablePostProcessor MakeDisplacementProcessor3D(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStrainProcessor(size_t values_per_node, const Mesh &mesh,
                                                    const DifferentialOperator &B_operator);

DerivedVariableProcessor MakeElasticStrainProcessor2D(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStrainProcessor3D(const Mesh &mesh);

DerivedVariableProcessor MakeElasticStressProcessor(size_t values_per_node, const Mesh &mesh,
                                                    const ConstitutiveModel &constitutive_model,
                                                    const DifferentialOperator &B_operator);

DerivedVariableProcessor MakeElasticStressProcessor2D(const Mesh &mesh,
                                                      const ConstitutiveModel &constitutive_model);

DerivedVariableProcessor MakeElasticStressProcessor3D(const Mesh &mesh,
                                                      const ConstitutiveModel &constitutive_model);

}  // namespace ffea::app

#endif  // FFEA_APPLICATIONS_ELASTICITY_INC_POSTPROCESSOR_H_
