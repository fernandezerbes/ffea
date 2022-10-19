#ifndef FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_
#define FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_

#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

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

enum class DerivativeOrder { kZeroth, kFirst, kSecond };

class GeometricEntity {
 public:
  GeometricEntity(GeometricEntityType type, size_t dim,
                  const std::vector<Node *> &nodes);

  GeometricEntityType type() const;
  size_t dim() const;
  size_t number_of_nodes() const;
  std::vector<size_t> node_tags() const;
  size_t node_tag(size_t node_idx) const;
  Coordinates &node_coords(size_t node_idx) const;
  virtual std::vector<Coordinates> nodal_local_coords() const = 0;

  Eigen::MatrixXd EvaluateShapeFunctions(
      const Coordinates &local_coords,
      DerivativeOrder order = DerivativeOrder::kZeroth) const;
  Eigen::MatrixXd EvaluateJacobian(
      const Coordinates &local_coords,
      const Eigen::MatrixXd &shape_functions_derivatives) const;
  Eigen::MatrixXd EvaluateJacobian(const Coordinates &local_coords) const;
  virtual Eigen::VectorXd EvaluateNormalVector(
      const Coordinates &local_coords) const;
  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const = 0;
  Coordinates MapLocalToGlobal(const Coordinates &local_coords) const;
  Coordinates MapLocalToGlobal(const Eigen::MatrixXd &N_at_point) const;

 private:
  Eigen::MatrixXd nodal_coords() const;
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const = 0;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const = 0;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const = 0;

  GeometricEntityType type_;
  size_t dim_;
  std::vector<Node *> nodes_;
};

// Line entities

class Line : public GeometricEntity {
 public:
  Line(GeometricEntityType type, size_t dim, const std::vector<Node *> &nodes);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Coordinates &local_coords) const override;
  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const override;
};

class TwoNodeLine : public Line {
 public:
  TwoNodeLine(size_t dim, const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

// Quad entities

class Quad : public GeometricEntity {
 public:
  Quad(GeometricEntityType type, size_t dim, const std::vector<Node *> &nodes);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Coordinates &local_coords) const override;
  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const override;
};

class FourNodeQuad : public Quad {
 public:
  FourNodeQuad(size_t dim, const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

// Hex entities

class Hex : public GeometricEntity {
 public:
  Hex(GeometricEntityType type, size_t dim, const std::vector<Node *> &nodes);

  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const override;
};

class EightNodeHex : public Hex {
 public:
  EightNodeHex(const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

// Tria entities
class Tria : public GeometricEntity {
 public:
  Tria(GeometricEntityType type, size_t dim, const std::vector<Node *> &nodes);

  virtual Eigen::VectorXd EvaluateNormalVector(
      const Coordinates &local_coords) const override;
  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const override;
};

class ThreeNodeTria : public Tria {
 public:
  ThreeNodeTria(size_t dim, const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

class SixNodeTria : public Tria {
 public:
  SixNodeTria(size_t dim, const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

// Tetra entities

class Tetra : public GeometricEntity {
 public:
  Tetra(GeometricEntityType type, size_t dim, const std::vector<Node *> &nodes);

  virtual double EvaluateDifferential(
      const Coordinates &local_coords) const override;
};

class FourNodeTetra : public Tetra {
 public:
  FourNodeTetra(const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

class TenNodeTetra : public Tetra {
 public:
  TenNodeTetra(const std::vector<Node *> &nodes);

  virtual std::vector<Coordinates> nodal_local_coords() const override;

 private:
  virtual Eigen::MatrixXd EvaluateShapeFunctions0thDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions1stDerivative(
      const Coordinates &local_coords) const override;
  virtual Eigen::MatrixXd EvaluateShapeFunctions2ndDerivative(
      const Coordinates &local_coords) const override;
};

}  // namespace ffea

#endif  // FFEA_FRAMEWORK_INC_GEOMETRY_GEOMETRICENTITY_H_
