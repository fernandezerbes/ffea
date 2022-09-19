#ifndef FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H
#define FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace ffea {

class DegreeOfFreedom {
 public:
  DegreeOfFreedom(size_t local_id, size_t number_of_axiliary_values = 0);

  size_t local_id() const;
  size_t global_id() const;
  double value() const;
  void set_value(double value);
  void set_value(const Eigen::VectorXd &solution);
  double auxiliary_value(size_t index) const;
  void set_auxiliary_value(size_t index, double value);

 private:
  double value_;
  std::vector<double> auxiliary_values_;
  size_t local_id_;
  size_t global_id_;

  void check_auxiliary_value(size_t index) const;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H