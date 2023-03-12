#include "../../inc/model/computational_domain.h"

#include <stdexcept>

namespace ffea {

ComputationalDomain::ComputationalDomain(const std::vector<Element>& elements,
                                         const ConstitutiveModel& constitutive_model,
                                         Integrand integrand, VectorialFunction source)
    : PhysicsProcessor(elements),
      constitutive_model_(constitutive_model),
      source_(source),
      integrand_(integrand) {}

void ComputationalDomain::Process(CSRMatrix<double>& system_stiffness,
                                  Vector<double>& system_rhs) const {
  for (auto& element : elements_) {
    auto number_of_dofs = element.number_of_dofs();
    Matrix<double> element_stiffness = Matrix<double>::Zero(number_of_dofs, number_of_dofs);

    Vector<double> element_rhs;
    if (source_) {
      element_rhs = Vector<double>::Zero(number_of_dofs);
    }

    for (const auto& ip : element.integration_points()) {
      const auto& local_coords = ip.local_coords();
      const auto& N = element.EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kZeroth);
      const auto& global_coords = element.MapLocalToGlobal(N);
      const auto& dN_local =
          element.EvaluateShapeFunctions(local_coords, ffea::DerivativeOrder::kFirst);
      const auto& jacobian = element.EvaluateJacobian(local_coords, dN_local);
      const auto& dN_global = jacobian.inverse() * dN_local;
      const auto& C = constitutive_model_.Evaluate(global_coords);
      const auto& weight = ip.weight();
      const auto& differential = element.EvaluateDifferential(local_coords);

      element_stiffness.triangularView<Eigen::Upper>() +=
          integrand_(dN_global, C) * weight * differential;

      if (source_) {
        const auto& load_vector = source_(global_coords);
        AddLoadContribution(load_vector, N, weight, differential, element_rhs);
      }
    }

    const auto& dof_tags = element.dof_tags();
    if (source_) {
      Scatter(dof_tags, element_stiffness, element_rhs, system_stiffness, system_rhs);
    } else {
      Scatter(dof_tags, element_rhs, system_rhs);
    }
  }
}

void ComputationalDomain::Process(CSRMatrix<double>& system_stiffness,
                                  CSRMatrix<double>& system_mass,
                                  Vector<double>& system_rhs) const {
  throw std::runtime_error("ComputationalDomain::Process not implemented for transient problems");
}

void ComputationalDomain::Process(CSRMatrix<double>& system_stiffness,
                                  CSRMatrix<double>& system_mass, CSRMatrix<double>& system_damping,
                                  Vector<double>& system_rhs) const {
  throw std::runtime_error(
      "ComputationalDomain::Process not implemented for damped transient problems");
}

}  // namespace ffea
