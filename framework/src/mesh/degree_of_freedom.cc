#include "../../inc/mesh/degree_of_freedom.h"

#include <stdexcept>

namespace ffea {

DegreeOfFreedom::DegreeOfFreedom() {}

DegreeOfFreedom::DegreeOfFreedom(int local_id, short number_of_auxiliary_values)
    : value_(),
      auxiliary_values_(number_of_auxiliary_values),
      local_id_(local_id),
      global_id_(local_id)  // TODO: To be set in a parallel environment
{}

DegreeOfFreedom::~DegreeOfFreedom() {}

int DegreeOfFreedom::local_id() const { return local_id_; }

int DegreeOfFreedom::global_id() const { return global_id_; }

double DegreeOfFreedom::value() const { return value_; }

void DegreeOfFreedom::set_value(double value) { value_ = value; }

void DegreeOfFreedom::set_value(const Eigen::VectorXd &solution) {
  set_value(solution(local_id()));
}

double DegreeOfFreedom::auxiliary_value(short index) const {
  check_auxiliary_value(index);

  return auxiliary_values_[index];
}

void DegreeOfFreedom::set_auxiliary_value(short index, double value) {
  check_auxiliary_value(index);

  auxiliary_values_[index] = value;
}

void DegreeOfFreedom::check_auxiliary_value(short index) const {
  if (auxiliary_values_.empty()) {
    throw std::out_of_range("There are no auxiliary values.");
  }

  if (index > (auxiliary_values_.size() - 1)) {
    throw std::out_of_range("Tried to access auxiliary value at index " +
                            std::to_string(index) + " but maximum index is " +
                            std::to_string(auxiliary_values_.size()) + ".");
  }
}

}  // namespace ffea