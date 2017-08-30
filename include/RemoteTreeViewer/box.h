//
// Created by manuelli on 8/30/17.
//

#ifndef SPARTAN_GRASP_BOX_H
#define SPARTAN_GRASP_BOX_H

#include "RemoteTreeViewer/box.h"


namespace RemoteTreeViewer{
namespace geometry{

using json = nlohmann::json;


class Box : public Geometry{
  Eigen::Vector3f dimensions_;
  Eigen::Affine3f transform_;

  Box(const Eigen::Vector3f& dimensions, const Eigen::Affine3f& transform_);
};

}// geometry
}// RemoteTreeViewer

#endif //SPARTAN_GRASP_BOX_H
