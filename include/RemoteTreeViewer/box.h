//
// Created by manuelli on 8/30/17.
//

#ifndef SPARTAN_GRASP_BOX_H
#define SPARTAN_GRASP_BOX_H

#include "RemoteTreeViewer/geometry.h"


namespace RemoteTreeViewer{
namespace geometry{

using json = nlohmann::json;


class Box : public Geometry{
 public:
  Eigen::Vector3f dimensions_;
  Box(const Eigen::Vector3f& dimensions, const Eigen::Affine3f& transform);

  json getJsonString();
};

}// geometry
}// RemoteTreeViewer

#endif //SPARTAN_GRASP_BOX_H
