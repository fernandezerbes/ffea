#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_

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
  GeometricEntity(GeometricEntityType type, size_t dimension,
                  const std::vector<Node *> &nodes,
                  std::unique_ptr<ShapeFunctions> shape_functions);
  virtual ~GeometricEntity() = default;
  GeometricEntity(const GeometricEntity &other) = default;
  GeometricEntity &operator=(const GeometricEntity &other) = default;
  GeometricEntity(GeometricEntity &&other) = default;
  GeometricEntity &operator=(GeometricEntity &&other) = default;

  GeometricEntityType type() const;
  size_t dimension() const;
  size_t GetNumberOfNodes() const;
  std::vector<size_t> GetNodesIds() const;
  Coordinates &GetCoordinatesOfNode(size_t node_index) const;
  Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coordinates,
      const Eigen::MatrixXd &shape_functions_derivatives) const;
  Eigen::MatrixXd EvaluateJacobian(const Coordinates &local_coordinates) const;
  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coordinates,
      DerivativeOrder derivative_order = DerivativeOrder::kZeroth) const;
  virtual Eigen::VectorXd EvaluateNormalVector(
      const Eigen::MatrixXd &jacobian) const;
  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const = 0;
  Coordinates MapLocalToGlobal(const Coordinates &local_coordinates) const;
  Coordinates MapLocalToGlobal(
      const Eigen::MatrixXd &shape_functions_at_point) const;
  const std::vector<Node *> &nodes() const;

 private:
  GeometricEntityType type_;
  size_t dimension_;
  std::vector<Node *> nodes_;
  std::unique_ptr<ShapeFunctions> shape_functions_;
  Eigen::MatrixXd GetNodesCoordinatesValues() const;
};

class Line2D : public GeometricEntity {
 public:
  Line2D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Eigen::MatrixXd &jacobian) const override;
  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Line3D : public GeometricEntity {
 public:
  Line3D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Eigen::MatrixXd &jacobian) const override;
  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Quad2D : public GeometricEntity {
 public:
  Quad2D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Quad3D : public GeometricEntity {
 public:
  Quad3D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Eigen::MatrixXd &jacobian) const override;
  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Hex3D : public GeometricEntity {
 public:
  Hex3D(GeometricEntityType type, const std::vector<Node *> &nodes,
        std::unique_ptr<ShapeFunctions> shape_functions);

  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Tria2D : public GeometricEntity {
 public:
  Tria2D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Tria3D : public GeometricEntity {
 public:
  Tria3D(GeometricEntityType type, const std::vector<Node *> &nodes,
         std::unique_ptr<ShapeFunctions> shape_functions);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Eigen::MatrixXd &jacobian) const override;
  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

class Tetra3D : public GeometricEntity {
 public:
  Tetra3D(GeometricEntityType type, const std::vector<Node *> &nodes,
          std::unique_ptr<ShapeFunctions> shape_functions);

  virtual double EvaluateDifferential(
      const Eigen::MatrixXd &jacobian) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_
