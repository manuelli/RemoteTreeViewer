//
// Created by manuelli on 8/29/17.
//

#include "RemoteTreeViewer/box.h"


namespace RemoteTreeViewer{
namespace geometry{

Box::Box(const Eigen::Vector3f& dimensions, const Eigen::Affine3f& transform_){
  this->dimensions_ = dimensions;
  this->transform_ = transform;
}

json Box::getJsonString(){
  json j = this->getBasicJsonString();
  j["lengths"] = {this->dimensions_[0], this->dimensions_[1], this->dimensions_[2]};
}


}// geometry
}// RemoteTreeViewer