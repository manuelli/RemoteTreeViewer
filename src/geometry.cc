//
// Created by manuelli on 8/29/17.
//

#include "RemoteTreeViewer/geometry.h"


namespace RemoteTreeViewer{
namespace geometry{

using json = nlohmann::json;

json transformToJson(Eigen::Affine3f& transform){
  json j;
  const Eigen::Vector3f& trans = transform.translation();
  j["translation"] = std::vector<float>({trans[0], trans[1], trans[2]});

  Eigen::Quaternionf quat(transform.linear());

  j["rotation"] = std::vector<float>({quat.w(), quat.x(), quat.y(), quat.z()});

  return j;
}

// default constructor for parent class sets color and alpha
Geometry::Geometry(){
  this->transform_ = Eigen::Affine3f::Identity();
  this->color_ = std::vector<double>({1,1,1});
  this->alpha_ = 1.0;
}

json Geometry::getBasicJsonString(){
  json j;
  std::vector<double> rgba;
  rgba.insert(rgba.begin(), this->color_.begin(), this->color_.end());
  rgba.push_back(this->alpha_);
  j["color"] = rgba;
  j["transform"] = transformToJson(this->transform_);
  return j;
}

Sphere::Sphere(Eigen::Vector3f origin, float radius){
  this->origin_ = origin;
  this->transform_.translation() = origin;
  this->radius_ = radius;
  this->color_ = std::vector<double>({1,1,1});
  this->alpha_ = 1.0;
}

json Sphere::getJsonString(){
  json j = this->getBasicJsonString();
  j["type"] = "sphere";
  j["radius"] = this->radius_;

//  std::vector<double> rgba;
//  rgba.insert(rgba.begin(), this->color_.begin(), this->color_.end());
//  rgba.push_back(this->alpha_);
//  j["color"] = rgba;
//  j["transform"]["translation"] = std::vector<float>({origin_[0], origin_[1], origin_[2]});
//  j["path"] = json({});

  return j;
}

GeometryContainer::GeometryContainer(){
  this->transform_ = Eigen::Affine3f::Identity();
}

void GeometryContainer::setTransform(const Eigen::Affine3f& transform){
  this->transform_ = transform;
}

void GeometryContainer::addGeometry(std::shared_ptr<Geometry> geometry){
  geometries_.push_back(geometry);
}

Eigen::Affine3f GeometryContainer::getTransform() const{
  return this->transform_;
}

}// geometry
}// RemoteTreeViewer