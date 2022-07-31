#include "../../inc/mesh/coordinates.h"

namespace ffea {

Coordinates::Coordinates() {}

Coordinates::Coordinates(const std::vector<double> &xyz) : xyz_(xyz) {}

Coordinates::~Coordinates() {}

const std::vector<double> &Coordinates::get() const { return xyz_; }

double Coordinates::get(short index) const { return xyz_[index]; }

void Coordinates::set(double x, double y, double z) {
  set(0, x);
  set(1, y);
  set(2, z);
}

void Coordinates::set(const std::vector<double> &xyz) { xyz_ = xyz; }

void Coordinates::set(short index, double value) { xyz_[index] = value; }

std::ostream &operator<<(std::ostream &os, const Coordinates &xyz) {
  os << "(x, y, z) = (" << xyz.get(0) << ", " << xyz.get(1) << ", "
     << xyz.get(2) << ")" << std::endl;
  return os;
}

}  // namespace ffea
