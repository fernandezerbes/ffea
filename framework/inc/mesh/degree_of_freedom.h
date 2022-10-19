#ifndef FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H
#define FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace ffea {

class DegreeOfFreedom {
 public:
  explicit DegreeOfFreedom(size_t tag, size_t number_of_axiliary_values = 0);

  size_t tag() const;
  size_t parallel_tag() const;
  double value() const;
  void set_value(double value);
  void set_value(const Eigen::VectorXd &solution);
  double auxiliary_value(size_t idx) const;
  void set_auxiliary_value(size_t idx, double value);

 private:
  void CheckAuxiliaryValueInsideRange(size_t idx) const;

  double value_;
  std::vector<double> auxiliary_values_;
  size_t tag_;
  size_t parallel_tag_;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H