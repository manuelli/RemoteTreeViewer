//
// Created by manuelli on 8/29/17.
//

#ifndef SPARTAN_GRASP_GEOMETRY_H
#define SPARTAN_GRASP_GEOMETRY_H

#include "json.hpp"
#include <Eigen/Dense>

namespace RemoteTreeViewer{
namespace geometry{

using json = nlohmann::json;

json transformToJson(Eigen::Affine3f& transform);

class Geometry{
 public:
  std::vector<double> color_;
  double alpha_;
  virtual json getJsonString() = 0;
  void setColor(std::vector<double> color){
    this->color_ = color;
  }

  void setAlpha(double alpha){
    this->alpha_ = alpha;
  }
};

class Sphere : public Geometry{
 public:
  Eigen::Vector3f origin_;
  float radius_;


  Sphere(Eigen::Vector3f origin, float radius);
  json getJsonString();
};

class GeometryContainer{
 public:
  std::vector<std::shared_ptr<Geometry>> geometries_;
  Eigen::Affine3f transform_;
  GeometryContainer();
  void setTransform(const Eigen::Affine3f& transform);
  Eigen::Affine3f getTransform() const;
  void addGeometry(std::shared_ptr<Geometry> geometry);
};

}// geometry
}// RemoteTreeViewer
#endif //SPARTAN_GRASP_GEOMETRY_H
