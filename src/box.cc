//
// Created by manuelli on 8/29/17.
//

#include "RemoteTreeViewer/box.h"


namespace RemoteTreeViewer{
namespace geometry{

Box::Box(const Eigen::Vector3f& dimensions, const Eigen::Affine3f& transform){
  this->dimensions_ = dimensions;
  this->transform_ = transform;
}

json Box::getJsonString(){
  json j = this->getBasicJsonString();
  j["type"] = "box";
  j["lengths"] = {this->dimensions_[0], this->dimensions_[1], this->dimensions_[2]};
  return j;
}


}// geometry
}// RemoteTreeViewer