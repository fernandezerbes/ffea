#ifndef FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
#define FFEA_FRAMEWORK_INC_MODEL_MODEL_H_

#include <list>
#include <memory>

#include "../alias.h"
#include "../mesh/element.h"
#include "../mesh/mesh.h"
#include "./boundary_condition.h"
#include "./constitutive_model.h"
#include "./equation.h"
#include "./physical_region.h"

namespace ffea {

class Model {
 public:
  explicit Model(Mesh &mesh);

  size_t number_of_dofs() const;

  void AddPhysicalRegion(PhysicalRegion &region);
  void AddEssentialBoundaryCondition(EssentialBoundaryCondition &boundary_condition);
  Equation GetEquations(double t);
  void ProjectSolutionOnMesh(const Vector<double> &solution);

 private:
  void EnforceBoundaryConditions(Equation &equation, double t);
  void SetSparsity(Equation &equation) const;
  Mesh &mesh_;
  std::vector<PhysicalRegion *> regions_;
  std::vector<EssentialBoundaryCondition *> bcs_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MODEL_MODEL_H_
