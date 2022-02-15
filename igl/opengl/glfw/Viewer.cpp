// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(true),
    score(0)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

  void Viewer::Move() {
      double velocity = 0.01;
      for (auto& data : data_list)
      {
          if (!data.pause) {
              switch (data.direction)
              {
              case 1: // up
                  data.MyTranslate(Eigen::Vector3d(0, velocity, 0), true);
                  data.center_dif += Eigen::Vector3d(0, velocity, 0);
                  break;
              case 2: // down
                  data.MyTranslate(Eigen::Vector3d(0, -velocity, 0), true);
                  data.center_dif += Eigen::Vector3d(0, -velocity, 0);
                  break;
              case 3: // left
                  data.MyTranslate(Eigen::Vector3d(-velocity, 0, 0), true);
                  data.center_dif += Eigen::Vector3d(-velocity, 0, 0);
                  break;
              case 4: // right
                  data.MyTranslate(Eigen::Vector3d(velocity, 0, 0), true);
                  data.center_dif += Eigen::Vector3d(velocity, 0, 0);
                  break;
              case 5: //inward
                  data.MyTranslate(Eigen::Vector3d(0, 0, velocity), true);
                  data.center_dif += Eigen::Vector3d(0, 0, velocity);
                  break;
              case 6: //outward
                  data.MyTranslate(Eigen::Vector3d(0, 0, -velocity), true);
                  data.center_dif += Eigen::Vector3d(0, 0, -velocity);
                  break;
              default:
                  break;
              }
          }
      }
  }

  double Viewer::sign(int i, int j) {
      if ((i == 0 && j == 1) || (i == 1 && j == 2) || (i == 2 && j == 0)) {
          return 1.0;
      }
      else {
          return -1.0;
      }
  }

  double Viewer::c_j(Eigen::RowVector3d Ai0, Eigen::RowVector3d Ai1, Eigen::RowVector3d Bj, double sign) {
      return sign * Ai0.dot(Ai1.cross(Bj));
  }

  double Viewer::c_i(Eigen::RowVector3d Bj0, Eigen::RowVector3d Ai, Eigen::RowVector3d Bj1, double sign) {
      return sign * Bj0.dot(Ai.cross(Bj1));
  }

  bool Viewer::does_intersect(Eigen::AlignedBox<double, 3> box1, Eigen::AlignedBox<double, 3> box2, Eigen::Matrix3d rotation1, Eigen::Matrix3d rotation2,
      Eigen::Vector3d center_dif1, Eigen::Vector3d center_dif2) {
      //std::cout << "does_intersect " << std::endl;
      Eigen::RowVector3d D = (box1.center() + center_dif1) - (box2.center() + center_dif2);

      // box 1 axis
      Eigen::RowVector3d A0 = rotation1.row(0);
      Eigen::RowVector3d A1 = rotation1.row(1);
      Eigen::RowVector3d A2 = rotation1.row(2);

      // box 1 extents
      Eigen::RowVector3d sizes = box1.sizes();
      double a0 = sizes(0) / 2;
      double a1 = sizes(1) / 2;
      double a2 = sizes(2) / 2;

      // box 2 axis
      Eigen::RowVector3d B0 = rotation2.row(0);
      Eigen::RowVector3d B1 = rotation2.row(1);
      Eigen::RowVector3d B2 = rotation2.row(2);

      // box 2 extents
      sizes = box2.sizes();
      double b0 = sizes(0) / 2;
      double b1 = sizes(1) / 2;
      double b2 = sizes(2) / 2;

      // checking intersection
      double c00 = A0.dot(B0);
      double c01 = A0.dot(B1);
      double c02 = A0.dot(B2);

      // 1st check
      if (std::abs(A0.dot(D)) > a0 + (b0 * std::abs(c00) + b1 * std::abs(c01) + b2 * std::abs(c02))) {
          return false;
      }

      double c10 = A1.dot(B0);
      double c11 = A1.dot(B1);
      double c12 = A1.dot(B2);

      // 2nd check
      if (std::abs(A1.dot(D)) > a1 + (b0 * std::abs(c10) + b1 * std::abs(c11) + b2 * std::abs(c12))) {
          return false;
      }

      double c20 = A2.dot(B0);
      double c21 = A2.dot(B1);
      double c22 = A2.dot(B2);

      // 3rd check
      if (std::abs(A2.dot(D)) > a2 + (b0 * std::abs(c20) + b1 * std::abs(c21) + b2 * std::abs(c22))) {
          return false;
      }

      // 4th check
      if (std::abs(B0.dot(D)) > b0 + (a0 * std::abs(c00) + a1 * std::abs(c10) + a2 * std::abs(c20))) {
          return false;
      }

      // 5th check
      if (std::abs(B1.dot(D)) > b1 + (a0 * std::abs(c01) + a1 * std::abs(c11) + a2 * std::abs(c21))) {
          return false;
      }

      // 6th check
      if (std::abs(B2.dot(D)) > b2 + (a0 * std::abs(c02) + a1 * std::abs(c12) + a2 * std::abs(c22))) {
          return false;
      }

      // 7th check
      if (std::abs(c10 * A2.dot(D) - c20 * A1.dot(D)) > a1 * std::abs(c20) + a2 * std::abs(c10) + b1 * std::abs(c02) + b2 * std::abs(c01)) {
          return false;
      }

      // 8th check
      if (std::abs(c11 * A2.dot(D) - c21 * A1.dot(D)) > a1 * std::abs(c21) + a2 * std::abs(c11) + b0 * std::abs(c02) + b2 * std::abs(c00)) {
          return false;
      }

      // 9th eck
      if (std::abs(c12 * A2.dot(D) - c22 * A1.dot(D)) > a1 * std::abs(c22) + a2 * std::abs(c12) + b0 * std::abs(c01) + b1 * std::abs(c00)) {
          return false;
      }

      // 10th eck
      if (std::abs(c20 * A0.dot(D) - c00 * A2.dot(D)) > a0 * std::abs(c20) + a2 * std::abs(c00) + b1 * std::abs(c12) + b2 * std::abs(c11)) {
          return false;
      }

      // 11th eck
      if (std::abs(c21 * A0.dot(D) - c01 * A2.dot(D)) > a0 * std::abs(c21) + a2 * std::abs(c01) + b0 * std::abs(c12) + b2 * std::abs(c10)) {
          return false;
      }

      // 12th eck
      if (std::abs(c22 * A0.dot(D) - c02 * A2.dot(D)) > a0 * std::abs(c22) + a2 * std::abs(c02) + b0 * std::abs(c11) + b1 * std::abs(c10)) {
          return false;
      }

      // 13th eck
      if (std::abs(c00 * A1.dot(D) - c10 * A0.dot(D)) > a0 * std::abs(c10) + a1 * std::abs(c00) + b1 * std::abs(c22) + b2 * std::abs(c21)) {
          return false;
      }

      // 14th eck
      if (std::abs(c01 * A1.dot(D) - c11 * A0.dot(D)) > a0 * std::abs(c11) + a1 * std::abs(c01) + b0 * std::abs(c22) + b2 * std::abs(c20)) {
          return false;
      }

      // 15th eck
      if (std::abs(c02 * A1.dot(D) - c12 * A0.dot(D)) > a0 * std::abs(c12) + a1 * std::abs(c02) + b0 * std::abs(c21) + b1 * std::abs(c20)) {
          return false;
      }

      return true;
  }


  bool Viewer::CheckCollisionRec(igl::opengl::ViewerData* obj1, igl::opengl::ViewerData* obj2, igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2) {
      if (tree1->is_leaf() && tree2->is_leaf()) {
          if (does_intersect(tree1->m_box, tree2->m_box, obj1->GetRotation(), obj2->GetRotation(), obj1->center_dif, obj2->center_dif)) {
              std::cout << "collision" << std::endl;
              obj1->draw_box(tree1->m_box, Eigen::RowVector3d(1, 0, 0));
              obj2->draw_box(tree2->m_box, Eigen::RowVector3d(1, 0, 0));

              return true;
          }

          return false;
      }

      if (does_intersect(tree1->m_box, tree2->m_box, obj1->GetRotation(), obj2->GetRotation(), obj1->center_dif, obj2->center_dif)) {

          return CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_right)
              || CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_left)
              || CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_right)
              || CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_left);
      }

      return false;
  }

  bool Viewer::CheckCollision() {
      for (int i = 0; i < data_list.size() - 1; i++)
      {
          for (int j = i + 1; j < data_list.size(); j++)
          {
              if (!data_list[i].pause || !data_list[j].pause) {
                  if (CheckCollisionRec(&data_list[i], &data_list[j], data_list[i].tree, data_list[j].tree)) {
                      data_list[i].pause = true;
                      data_list[j].pause = true;
                      return true;
                  }
              }
          }
      }
      return false;
  }

} // end namespace
} // end namespace
}
