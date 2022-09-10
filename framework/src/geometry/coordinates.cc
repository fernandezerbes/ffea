#include "../../inc/geometry/coordinates.h"

namespace ffea {

Coordinates::Coordinates() : xyz_{} {}

Coordinates::Coordinates(const std::array<double, 3> &xyz) : xyz_(xyz) {}

Coordinates::Coordinates(double x, double y, double z) : xyz_({x, y, z}) {}

Coordinates::~Coordinates() {}

double Coordinates::get(size_t index) const { return xyz_[index]; }

void Coordinates::set(double x, double y, double z) {
  set(0, x);
  set(1, y);
  set(2, z);
}

void Coordinates::set(const std::array<double, 3> &xyz) { xyz_ = xyz; }

void Coordinates::set(size_t index, double value) { xyz_[index] = value; }

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz) {
  os << "(x, y, z) = (" << xyz.get(0) << ", " << xyz.get(1) << ", "
     << xyz.get(2) << ")" << std::endl;
  return os;
}

}  // namespace ffea
