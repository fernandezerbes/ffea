#ifndef FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
#define FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_

#include "./Coordinates.h"

namespace ffea {

class IntegrationPoint
{
 public:
  IntegrationPoint(const Coordinates &local_coordinates, double weight);
  ~IntegrationPoint();

  const Coordinates &local_coordinates() const;
  double weight() const;

 private:
  Coordinates local_coordinates_;
  double weight_;
};

}

#endif // FFEA_FRAMEWORK_MESH_INTEGRATIONPOINT_H_
