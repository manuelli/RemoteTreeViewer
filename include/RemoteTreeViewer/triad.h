//
// Created by manuelli on 9/5/17.
//

#ifndef SPARTAN_GRASP_TRIAD_H
#define SPARTAN_GRASP_TRIAD_H

//RemoteTreeViewer
#include "RemoteTreeViewer/geometry.h"


namespace RemoteTreeViewer{
namespace geometry{

using json = nlohmann::json;


class Triad : public Geometry{
 public:
  float scale_;

  Triad(const Eigen::Affine3f& transform, float scale = 0.15);

  json getJsonString();
};

}// geometry
}// RemoteTreeViewer

#endif //SPARTAN_GRASP_TRIAD_H
