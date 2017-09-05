// system
#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>

// RemoteTreeViewer
#include "viewer2_comms_t.hpp"
#include "RemoteTreeViewer/RemoteTreeViewerWrapper.hpp"
#include "RemoteTreeViewer/json.hpp"

using namespace std;
using namespace Eigen;
//using namespace drake::lcm;
//using namespace DrakeShapes;
using json = nlohmann::json;
using namespace robotlocomotion;

namespace RemoteTreeViewer {

static double getUnixTime(void) {
  struct timespec tv;

  if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

  return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

json makeDefaultJsonDictionary(){
  long long int now = getUnixTime() * 1000 * 1000;
  // default json dict
  json j = {
      {"timestamp", now},
      {"setgeometry",json({})},
      {"settransform", json({})},
      {"delete", json({})}
  };
  return j;
}

RemoteTreeViewerWrapper::RemoteTreeViewerWrapper() {
  this->publish_channel_ = "DIRECTOR_TREE_VIEWER_REQUEST_<0>";
}

void RemoteTreeViewerWrapper::publishPointCloud(const Matrix3Xd &pts,
                                                const vector <string> &path,
                                                const vector <vector<double>> &color) {
  long long int now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {
      {"timestamp", now},
      {"setgeometry",
       {{
            {"path", path},
            {"geometry",
             {
                 {"type", "pointcloud"},
                 {"points", vector < vector < double >> ()},
                 {"channels",
                  {
                      {"rgb", vector < vector < double >> ()}
                  }
                 }
             }
            }
        }},
      },
      {"settransform", json({})},
      {"delete", json({})}
  };

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back({pts(0, i), pts(1, i), pts(2, i)});
    if (color.size() == 1)
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[0]);
    else if (color.size() == pts.cols())
      j["setgeometry"][0]["geometry"]["channels"]["rgb"].push_back(color[i]);
    else if (color.size() != 0)
      printf("Color does not have right number of entries: %ld vs %ld\n", color.size(), pts.cols());
  }

  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto &c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::publishJson(json j){
  long long int now = getUnixTime() * 1000 * 1000;
  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto &c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.publish(this->publish_channel_, &msg);
}

void RemoteTreeViewerWrapper::publishLine(const Matrix3Xd &pts, const vector <string> &path) {
  long long int now = getUnixTime() * 1000 * 1000;
  // Format a JSON string for this pointcloud
  json j = {
      {"timestamp", now},
      {"setgeometry",
       {{
            {"path", path},
            {"geometry",
             {
                 {"type", "line"},
                 {"points", vector < vector < double >> ()},
             }
            }
        }},
      },
      {"settransform", json({})},
      {"delete", json({})}
  };

  // Push in the points and colors.
  for (int i = 0; i < pts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["points"].push_back({pts(0, i), pts(1, i), pts(2, i)});
  }

  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto &c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::publishRawMesh(const Matrix3Xd &verts,
                                             const std::vector<Vector3i> &tris,
                                             const vector <string> &path) {
  long long int now = getUnixTime() * 1000 * 1000;
  json j = {
      {"timestamp", now},
      {"setgeometry",
       {{
            {"path", path},
            {"geometry",
             {
                 {"type", "mesh_data"},
                 {"vertices", vector < vector < double >> ()},
                 {"faces", vector < vector < int >> ()}
             }
            }
        }},
      },
      {"settransform", json({})},
      {"delete", json({})}
  };

  // Push in the points.
  for (int i = 0; i < verts.cols(); i++) {
    j["setgeometry"][0]["geometry"]["vertices"].push_back({verts(0, i), verts(1, i), verts(2, i)});
  }
  // Push in the triangle indices.
  for (int i = 0; i < tris.size(); i++) {
    j["setgeometry"][0]["geometry"]["faces"].push_back({tris[i][0], tris[i][1], tris[i][2]});
  }

  auto msg = viewer2_comms_t();
  msg.utime = now;
  msg.format = "treeviewer_json";
  msg.format_version_major = 1;
  msg.format_version_minor = 0;
  msg.data.clear();
  for (auto &c : j.dump())
    msg.data.push_back(c);
  msg.num_bytes = j.dump().size();
  // Use channel 0 for remote viewer communications.
  lcm_.publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
}

void RemoteTreeViewerWrapper::deletePath(const std::vector <std::string> &path){
  long long int now = getUnixTime() * 1000 * 1000;

  json j = {
      {"timestamp", now},
      {"setgeometry",json({})},
      {"settransform", json({})},
      {"delete", json({})}
      // list of delete commands
  };

  j["delete"] = { {{"path", path}} };
  j["list"] = {0,1,2};

  this->publishJson(j);

}

void RemoteTreeViewerWrapper::publishGeometryContainer(RemoteTreeViewer::geometry::GeometryContainer& container, const std::vector <std::string> &path){

  // default json dict
  json j = makeDefaultJsonDictionary();

  Eigen::Affine3f transform = container.getTransform();
  json transform_json = RemoteTreeViewer::geometry::transformToJson(transform);
  json transform_path = path;
  json single_set_transform_command =  {{"path", path}, {"transform", transform_json} };
  j["settransform"] ={ single_set_transform_command } ;


  // add all the geometries
  auto basic_json = nlohmann::basic_json<>();
  auto json_array = basic_json.array();
  for(int i = 0; i < container.geometries_.size(); i++){
    std::shared_ptr<RemoteTreeViewer::geometry::Geometry> geom = container.geometries_[i];
    json geom_json = geom->getJsonString();

    json_array.push_back(geom_json);
  }

  json single_geometry_message = {{"path", path}, {"geometries", json_array}};
  j["setgeometry"] = {single_geometry_message};

  this->publishJson(j);

//  std::cout << j << "\n\n\n";
}

//void RemoteTreeViewerWrapper::publishRigidBodyTree(const RigidBodyTree<double>& tree, const VectorXd& q, const Vector4d& color, const vector<string>& path, bool visual){
//  auto kinematics_cache = tree.doKinematics(q);
//  for (const auto& body : tree.bodies) {
//    if (visual){
//      for (const auto& element : body->get_visual_elements()){
//        if (element.hasGeometry()){
//          vector<string> full_name = path;
//          full_name.push_back(body->get_name());
//          publishGeometry(element.getGeometry(),
//            tree.relativeTransform(kinematics_cache, 0, body->get_body_index()) * element.getLocalTransform(),
//            color,
//            full_name);
//        }
//      }
//    } else {
//      for (const auto& collision_elem_id : body->get_collision_element_ids()) {
//        auto element = tree.FindCollisionElement(collision_elem_id);
//        if (element->hasGeometry()){
//          vector<string> full_name = path;
//          full_name.push_back(body->get_name());
//          publishGeometry(element->getGeometry(),
//            tree.relativeTransform(kinematics_cache, 0, body->get_body_index()) * element->getLocalTransform(),
//            color,
//            full_name);
//        }
//      }
//    }
//  }
//}

//void RemoteTreeViewerWrapper::publishRigidBody(const RigidBody<double>& body, const Affine3d& tf, const Vector4d& color, const vector<string>& path){
//}

//void RemoteTreeViewerWrapper::publishGeometry(const Geometry& geometry, const Affine3d& tf, const Vector4d& color, const vector<string>& path){
//  long long int now = getUnixTime() * 1000 * 1000;
//
//  // Short-circuit to points if the passed geometry is a set of mesh points
//  if (geometry.getShape() == MESH_POINTS){
//    Matrix3Xd pts;
//    geometry.getPoints(pts);
//    publishPointCloud(pts, path);
//  }
//
//  // Extract std::vector-formatted translation and quaternion from the tf
//  vector<double> translation = {
//    tf.translation()[0],
//    tf.translation()[1],
//    tf.translation()[2]};
//  Quaterniond q(tf.rotation());
//  vector<double> rotation = { q.w(), q.x(), q.y(), q.z() };
//
//  // Format a JSON string for this bit of geometry (in a setgeometry call)
//  // and its transformation (in a settransform call)
//  json j = {
//    {"timestamp", now},
//    {"setgeometry",
//      {{
//        {"path", path},
//        {"geometry",
//          {
//            {"color", {color[0], color[1], color[2], color[3]}}
//          }
//        }
//      }},
//    },
//    {"settransform",
//      {{
//        {"path", path},
//        {"transform",
//          {
//            {"translation", translation},
//            {"quaternion", rotation}
//          }
//        }
//      }}
//    },
//    {"delete", json({})}
//  };
//
//  // Fill in the setgeometry call based on what kind of geometry this is
//  switch (geometry.getShape()){
//    case BOX:
//      {
//      const Box * box = static_cast<const Box *>(&geometry);
//      j["setgeometry"][0]["geometry"]["type"] = string("box");
//      j["setgeometry"][0]["geometry"]["lengths"] = vector<double>({box->size[0], box->size[1], box->size[2]});
//      }
//      break;
//    case SPHERE:
//      {
//      const Sphere * sphere = static_cast<const Sphere *>(&geometry);
//      j["setgeometry"][0]["geometry"]["type"] = string("sphere");
//      j["setgeometry"][0]["geometry"]["radius"] = sphere->radius;
//      }
//      break;
//    case CYLINDER:
//      {
//      const Cylinder * cylinder = static_cast<const Cylinder *>(&geometry);
//      j["setgeometry"][0]["geometry"]["type"] = string("cylinder");
//      j["setgeometry"][0]["geometry"]["radius"] = cylinder->radius;
//      j["setgeometry"][0]["geometry"]["length"] = cylinder->length;
//      }
//      break;
//    case MESH:
//      {
//      const Mesh * mesh = static_cast<const Mesh *>(&geometry);
//      j["setgeometry"][0]["geometry"]["type"] = string("mesh_file");
//      j["setgeometry"][0]["geometry"]["filename"] = mesh->resolved_filename_;
//      j["setgeometry"][0]["geometry"]["scale"] = vector<double>({mesh->scale_[0], mesh->scale_[1], mesh->scale_[2]});
//      }
//      break;
//    case CAPSULE:      {
//      const Capsule * capsule = static_cast<const Capsule *>(&geometry);
//      j["setgeometry"][0]["geometry"]["type"] = string("capsule");
//      j["setgeometry"][0]["geometry"]["radius"] = capsule->radius;
//      j["setgeometry"][0]["geometry"]["length"] = capsule->length;
//      }
//      break;
//    default:
//      cout << "Unsupported geometry type " << geometry.getShape() << ", sorry!" << endl;
//      return;
//  }
//
//
//  auto msg = viewer2_comms_t();
//  msg.utime = now;
//  msg.format = "treeviewer_json";
//  msg.format_version_major = 1;
//  msg.format_version_minor = 0;
//  msg.data.clear();
//  for (auto& c : j.dump())
//    msg.data.push_back(c);
//  msg.num_bytes = j.dump().size();
//  // Use channel 0 for remote viewer communications.
//  lcm_.publish("DIRECTOR_TREE_VIEWER_REQUEST_<0>", &msg);
//}

}// RemoteTreeViewer