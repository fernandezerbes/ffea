#include <iostream>
#include <memory>

#include "../framework/inc/math/CustomDenseMatrix.h"
#include "../framework/inc/mesh/Coordinates.h"
#include "../framework/inc/mesh/DegreeOfFreedom.h"
#include "../framework/inc/mesh/Node.h"

int main() {

  ffea::Coordinates my_coordinates(1.2, -3.0, 1.5);
  std::cout << my_coordinates << std::endl;

  my_coordinates.set(0, 9.9);
  my_coordinates.set(1, 9.9);
  my_coordinates.set(2, 9.9);

  std::cout << my_coordinates << std::endl;

  ffea::DegreeOfFreedom my_dof(123, 2);

  std::cout << my_dof.value() << std::endl;

  my_dof.auxiliary_value(1);

  ffea::Node my_node(my_coordinates);
  my_node.set_number_of_dofs(10);
  std::cout << "my_node has " << my_node.number_of_dofs() << " dofs" << std::endl;
  auto &dofs = my_node.dofs();

  double sum = 0.0;
  for (auto &dof: my_node.dofs()) {
    dof.set_value(sum);
    sum += 1.0;
  }

  for (const auto &dof: my_node.dofs()) {
    std::cout << "Dof id " << dof.global_id() << " with value " << dof.value() << std::endl;
  }
  
  return 0;
}