#ifndef FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_
#define FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_

#include <iostream>
#include <array>

namespace ffea {

class Coordinates {
 public:
  Coordinates();
  Coordinates(const std::array<double, 3> &xyz);
  Coordinates(double x, double y, double z);
  ~Coordinates();

  double get(size_t index) const;
  void set(double x, double y, double z);
  void set(const std::array<double, 3> &xyz);
  void set(size_t index, double value);

 private:
  std::array<double, 3> xyz_;
};

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz);

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_MESH_COORDINATES_H_