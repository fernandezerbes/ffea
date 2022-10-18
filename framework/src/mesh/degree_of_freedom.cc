#include "../../inc/mesh/degree_of_freedom.h"

#include <stdexcept>

namespace ffea {

DegreeOfFreedom::DegreeOfFreedom(size_t tag, size_t number_of_auxiliary_values)
    : value_(),
      auxiliary_values_(number_of_auxiliary_values),
      tag_(tag),
      parallel_tag_(tag)  // TODO: To be set in a parallel environment
{}

size_t DegreeOfFreedom::tag() const { return tag_; }

size_t DegreeOfFreedom::parallel_tag() const { return parallel_tag_; }

double DegreeOfFreedom::value() const { return value_; }

void DegreeOfFreedom::set_value(double value) { value_ = value; }

void DegreeOfFreedom::set_value(const Eigen::VectorXd &solution) {
  set_value(solution(tag()));
}

double DegreeOfFreedom::auxiliary_value(size_t index) const {
  check_auxiliary_value(index);

  return auxiliary_values_[index];
}

void DegreeOfFreedom::set_auxiliary_value(size_t index, double value) {
  check_auxiliary_value(index);

  auxiliary_values_[index] = value;
}

void DegreeOfFreedom::check_auxiliary_value(size_t index) const {
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
