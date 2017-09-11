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
  std::string name_ = "";
  Eigen::Affine3f transform_;

  Geometry();

  json getBasicJsonString();

  virtual json getJsonString() = 0;

  void setColor(std::vector<double> color){
    this->color_ = color;
  }

  std::vector<double> getColor(){
    return this->color_;
  }

  void setAlpha(double alpha){
    this->alpha_ = alpha;
  }

  double getAlpha(){
    return this->alpha_;
  }

  void setName(std::string name){
    this->name_ = name;
  }

  std::string getName(){
    return this->name_;
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
  GeometryContainer(const Eigen::Affine3f& transform);

  /**
   * Set the transform for the whole geometry
   * @param transform
   */
  void setTransform(const Eigen::Affine3f& transform);
  Eigen::Affine3f getTransform() const;

  /**
   * Add a geometry to be draw
   * @param geometry
   */
  void addGeometry(std::shared_ptr<Geometry> geometry);

  /**
   * Set the color of all geometries in this container
   * @param color
   */
  void setColor(std::vector<double> color);


};

}// geometry
}// RemoteTreeViewer
#endif //SPARTAN_GRASP_GEOMETRY_H
