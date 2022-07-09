#ifndef FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H
#define FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H

#include <vector>

namespace ffea {

class DegreeOfFreedom
{
 public:
  DegreeOfFreedom();
  DegreeOfFreedom(int local_id, short number_of_axiliary_values = 0);
  ~DegreeOfFreedom();

  int local_id() const;
  int global_id() const;
  double value() const;
  void set_value(double value);
  double auxiliary_value(short index) const;
  void set_auxiliary_value(short index, double value);
  
 private:
  double value_;
  std::vector<double> auxiliary_values_;
  int local_id_;
  int global_id_;

  void check_auxiliary_value(short index) const;

};

} // namespace ffea

#endif // FFEA_FRAMEWORK_INC_MESH_DEGREEOFFREEDOM_H