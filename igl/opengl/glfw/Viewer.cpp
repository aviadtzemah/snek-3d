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

// our imports
#include <igl/readTGF.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/PI.h>
#include <math.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>

// our addition
typedef
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
RotationList;

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
	isActive(false),
    tipPosition(Eigen::Vector3d(0, 0, -0.8)), //TODO: see about tip position if there's problems with the collision
    linkOffset(Eigen::Vector3d(0, 0, -1.6)),
    linkLength(1.6),
    prevParent(-1),
    linkNum(0),
    direction(0)
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

  IGL_INLINE bool Viewer::arrange_links_and_set_parents() {
      // TODO: set up the camera so it would be around the head of the snake
    
      // scaling addition
      data_list[0].MyScale(Eigen::Vector3d(1, 1, 1/1.6)); // normalizing the length to 1
      data_list[0].MyScale(Eigen::Vector3d(1, 1, (linkNum-1)* linkLength));
     

      // setting the links
      for (int i = 1; i < linkNum; i++) {
          data_list[i].MyTranslate(linkOffset, true);
          tipPosition += linkOffset;
      }

      // setting the parents
      for (int i = 1; i < linkNum-1; i++) {
          parents[i] = i+1;
      }

      data_list[linkNum - 1].MyTranslate(-linkOffset * ((linkNum)/2), true);
      tipPosition += -linkOffset * ((linkNum) / 2); // TODO: not sure about the calculation of this
      data_list[linkNum - 1].MyScale(Eigen::Vector3d(0.5, 0.5, 1)); // for some reason only scaling the base sacles the whole skeleton. so we'll take it.

      return true;
  }

  IGL_INLINE int Viewer::smallest_index(Eigen::VectorXd vec) {
      double smallest = std::numeric_limits<double>::max();
      int index = -1;
      for (int i = 0; i < vec.size(); i++) {
          if ((vec(i) < smallest && vec(i) > 0)) {
              smallest = vec(i);
              index = i;
          }
      }
      return index;
  }

  IGL_INLINE void Viewer::calculate_and_write_weights() {
      Eigen::MatrixXd W = Eigen::MatrixXd::Zero(data().V.rows(), data().C.rows());

      //std::cout << data().C.rows() << std::endl;

      // calculating the top 4 closets joints for each vertex
      for (int i = 0; i < data().V.rows(); i++) {
          for (int j = 0; j < data().C.rows(); j++) {
              W(i, j) = 1/sqrt(pow((data().V(i, 0) - data().C(j, 0)), 2) + pow((data().V(i, 1) - data().C(j, 1)), 2) + pow((data().V(i, 2) - data().C(j, 2)), 2));

              if (j >= 4) {
                  int small_index = smallest_index(W.row(i));
                  W(i, small_index) = 0;
              }
          }
      }

      // nomalizing so the weights would add up to 1 for each vertex
      for (int i = 0; i < W.rows(); i++) {
          double row_sum = W.row(i).sum();
          for (int j = 0; j < W.cols(); j++) {
              W(i, j) = W(i, j) / row_sum;
          }
      }

      writeDMAT("C:/Users/aviad/Desktop/snek_weights.dmat", W);
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

    
    // currently assuming the snake is the only object
    // scaling the snake object
    data().MyScale(Eigen::Vector3d(1, 1, 1 / 1.6)); // normalizing the length to 1
    data().MyScale(Eigen::Vector3d(1, 1, 16 * linkLength));

    // skinning additions
    data().U = data().V;

    igl::readTGF("D:/University/Animation/Project/snek-3d/tutorial/data/snake.tgf", data().C, data().BE);
    // retrieve parents for forward kinematics
    igl::directed_edge_parents(data().BE, data().P);
    igl::directed_edge_orientations(data().C, data().BE, data().rest_pose);

    // TODO: write it properly 
    /*data().poses.resize(4, RotationList(4, Eigen::Quaterniond::Identity()));
    const Eigen::Quaterniond bend(Eigen::AngleAxisd(-igl::PI * 0.7, Eigen::Vector3d(0, 0, 1)));
    data().poses[3][2] = data().rest_pose[2] * bend * data().rest_pose[2].conjugate();*/

    // calculate skinning weights
    //calculate_and_write_weights();
    igl::readDMAT("D:/University/Animation/Project/snek-3d/tutorial/data/snake_weights.dmat", data().W);

    data().set_edges(data().C, data().BE, Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.));

    // fabrik addition
    /*data().SetCenterOfRotation(Eigen::Vector3d(0, 0, 0.8));
    linkNum++;*/

    /*std::cout << data().C << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << data().V << std::endl;*/

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

  // our functions

  IGL_INLINE bool Viewer::AnimateFabrik() {
      return true;
  }

} // end namespace
} // end namespace
}
