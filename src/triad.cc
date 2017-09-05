//
// Created by manuelli on 9/5/17.
//

//RemoteTreeViewer
#include "RemoteTreeViewer/triad.h"

namespace RemoteTreeViewer {
namespace geometry {

using json = nlohmann::json;

Triad::Triad(const Eigen::Affine3f &transform, float scale){
  this->scale_ = scale;
}

json Triad::getJsonString() {
  json j = this->getBasicJsonString();
  j["type"] = "triad";
  j["scale"] = this->scale_;
  return j;
}

}//geometry
}//RemoteTreeViewer