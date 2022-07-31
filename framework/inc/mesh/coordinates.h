#ifndef FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_
#define FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_

#include <iostream>
#include <vector>

namespace ffea {

class Coordinates {
 public:
  Coordinates();
  Coordinates(const std::vector<double> &xyz);
  ~Coordinates();

  const std::vector<double> &get() const;
  double get(short index) const;
  void set(double x, double y, double z);
  void set(const std::vector<double> &xyz);
  void set(short index, double value);

 private:
  std::vector<double> xyz_;
};

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz);

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_