#pragma once

//lcm
#include "lcm/lcm-cpp.hpp"

//Eigen
#include <Eigen/Dense>

//RemoteTreeViewer
#include "RemoteTreeViewer/geometry.h"
#include "RemoteTreeViewer/json.hpp"


namespace RemoteTreeViewer {
using json = nlohmann::json;

class RemoteTreeViewerWrapper {
 public:
  lcm::LCM lcm_;
  RemoteTreeViewerWrapper();

  void publishJson(json j);

  void publishPointCloud(
      const Eigen::Matrix3Xd &pts,
      const std::vector <std::string> &path,
      const std::vector <std::vector<double>> &color = {{1.0, 1.0, 1.0}});
  void publishLine(
      const Eigen::Matrix3Xd &pts,
      const std::vector <std::string> &path);
  void publishRawMesh(
      const Eigen::Matrix3Xd &verts,
      const std::vector <Eigen::Vector3i> &tris,
      const std::vector <std::string> &path);

  void deletePath(const std::vector <std::string> &path);

  void publishGeometryContainer(RemoteTreeViewer::geometry::GeometryContainer& container, const std::vector <std::string> &path);

  static std::vector<std::string> makePath(std::vector<std::string> path_1, std::string path_2){
    path_1.push_back(path_2);
    return path_1;
  }

  static std::vector<std::string> makePath(std::vector<std::string> path_1, std::vector<std::string> path_2){
    path_1.insert(path_1.end(), path_2.begin(), path_2.end());
    return path_1;
  }

//    void publishRigidBodyTree(
//            const RigidBodyTree<double>& tree,
//            const Eigen::VectorXd& q,
//            const Eigen::Vector4d& color,
//            const std::vector<std::string>& path,
//            bool visual = true);
//    void publishRigidBody(
//            const RigidBody<double>& body,
//            const Eigen::Affine3d& tf,
//            const Eigen::Vector4d& color,
//            const std::vector<std::string>& path);
//    void publishGeometry(
//            const DrakeShapes::Geometry& geometry,
//            const Eigen::Affine3d& tf,
//            const Eigen::Vector4d& color,
//            const std::vector<std::string>& path);

 private:
  std::string publish_channel_;


};

}// RemoteTreeViewer