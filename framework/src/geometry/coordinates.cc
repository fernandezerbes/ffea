#include "../../inc/geometry/coordinates.h"

#include <stdexcept>
#include <string>

namespace ffea {

Coordinates::Coordinates(const std::array<double, 3> &xyz) : xyz_(xyz) {}

Coordinates::Coordinates(double x, double y, double z) : xyz_({x, y, z}) {}

double Coordinates::get(size_t index) const {
  if (index > 2) {
    throw std::runtime_error("Tried to get coordinate at index " +
                             std::to_string(index) +
                             ", but maximum possible index is 2.");
  }

  return xyz_[index];
}

void Coordinates::set(double x, double y, double z) {
  set(0, x);
  set(1, y);
  set(2, z);
}

void Coordinates::set(const std::array<double, 3> &xyz) { xyz_ = xyz; }

void Coordinates::set(size_t index, double value) {
  if (index > 2) {
    throw std::runtime_error("Tried to set coordinate at index " +
                             std::to_string(index) +
                             ", but maximum possible index is 2.");
  }

  xyz_[index] = value;
}

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz) {
  os << "(x, y, z) = (" << xyz.get(0) << ", " << xyz.get(1) << ", "
     << xyz.get(2) << ")" << std::endl;
  return os;
}

}  // namespace ffea
