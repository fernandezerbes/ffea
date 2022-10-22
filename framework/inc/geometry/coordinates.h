#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_COORDINATES_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_COORDINATES_H_

#include <array>
#include <iostream>

namespace ffea {

class Coordinates {
 public:
  Coordinates(const std::array<double, 3> &xyz);
  Coordinates(double x, double y, double z);

  double get(size_t idx) const;
  void set(double x, double y, double z);
  void set(const std::array<double, 3> &xyz);
  void set(size_t idx, double value);

 private:
  std::array<double, 3> xyz_;
};

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz);

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_COORDINATES_H_
