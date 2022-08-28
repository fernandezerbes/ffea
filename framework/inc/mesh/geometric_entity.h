#ifndef FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

#include "../math/shape_functions.h"
#include "./coordinates.h"
#include "./node.h"

namespace ffea {

enum class GeometricEntityType {
  kTwoNodeLine,
  kThreeNodeTria,
  kFourNodeQuad,
  kFourNodeTetra,
  kEightNodeHex,
  kSixNodePrism,
  kFiveNodePiramid,
  kThreeNodeLine,
  kSixNodeTria,
  kNineNodeQuad,
  kTenNodeTetra,
  kTwentySevenNodeHex,
  kEighteenNodePrism,
  kFourteenNodePiramid,
  kOneNodePoint,
  kEightNodeQuad,
  kTwentyNodeHex,
  kFifteenNodePrism,
  kThirteenNodePiramid
};

class GeometricEntity {
 public:
  GeometricEntity(size_t dimension, const std::vector<Node *> &nodes,
                  std::unique_ptr<ShapeFunctions> shape_functions);
  virtual ~GeometricEntity();

  size_t dimension() const;
  size_t GetNumberOfNodes() const;
  std::vector<size_t> GetNodesIds() const;
  Coordinates &GetCoordinatesOfNode(size_t node_index) const;
  virtual Eigen::VectorXd EvaluateNormal(
      const Coordinates &local_coordinates) const = 0;
  Eigen::MatrixXd EvaluateJacobian(const Coordinates &local_coordinates) const;
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  Coordinates MapLocalToGlobal(const Coordinates &local_coordinates,
                               const Eigen::MatrixXd &shape_functions) const;

 private:
  size_t dimension_;
  std::vector<Node *> nodes_;
  std::unique_ptr<ShapeFunctions> shape_functions_;
  Eigen::MatrixXd GetNodesCoordinatesValues() const;
};

class TwoNodeLine2D : public GeometricEntity {
 public:
  TwoNodeLine2D(const std::vector<Node *> &nodes);
  virtual ~TwoNodeLine2D();

  virtual Eigen::VectorXd EvaluateNormal(
      const Coordinates &local_coordinates) const override;
};

class TwoNodeLine3D : public GeometricEntity {
 public:
  TwoNodeLine3D(const std::vector<Node *> &nodes);
  virtual ~TwoNodeLine3D();

  virtual Eigen::VectorXd EvaluateNormal(
      const Coordinates &local_coordinates) const override;
};

class FourNodeQuad2D : public GeometricEntity {
 public:
  FourNodeQuad2D(const std::vector<Node *> &nodes);
  virtual ~FourNodeQuad2D();

  virtual Eigen::VectorXd EvaluateNormal(
      const Coordinates &local_coordinates) const override;
};

class FourNodeQuad3D : public GeometricEntity {
 public:
  FourNodeQuad3D(const std::vector<Node *> &nodes);
  virtual ~FourNodeQuad3D();

  virtual Eigen::VectorXd EvaluateNormal(
      const Coordinates &local_coordinates) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRICENTITY_H_
