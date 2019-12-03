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
#include <igl/circulation.h>
#include <igl/igl_inline.h>

//Ass2:
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <igl/collapse_edge.h>
#include <vector>
//---------------------

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;
using namespace std;

namespace igl
{
namespace opengl
{
namespace glfw
{

  IGL_INLINE void Viewer::init()
  {
   

  }

  //IGL_INLINE void Viewer::init_plugins()
  //{
  //  // Init all plugins
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->init(this);
  //  }
  //}

  //IGL_INLINE void Viewer::shutdown_plugins()
  //{
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->shutdown();
  //  }
  //}

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1)
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
  :       Toggle face labels
  space	  Decimate model)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  void Viewer::reset() {
	Eigen::MatrixXi F = data().OF;
	Eigen::MatrixXd V = data().OV;
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	Eigen::MatrixXd C;
	PriorityQueue* Q = new PriorityQueue;
	std::vector<PriorityQueue::iterator >* Qit = new std::vector<PriorityQueue::iterator >;
	int num_collapsed = 0;
	edge_flaps(F, E, EMAP, EF, EI);
	Qit->resize(E.rows());

	C.resize(E.rows(), V.cols());
	Q->clear();
	for (int e = 0; e < E.rows(); e++)
	{
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		C.row(e) = p;
		(*Qit)[e] = Q->insert(std::pair<double, int>(cost, e)).first;
	}
	data().num_collapsed = num_collapsed;
	data().E = E;
	data().EF = EF;
	data().EI = EI;
	data().EMAP = EMAP;
	data().Q = Q; 
	data().Qit = Qit;
	data().C = C;
	data().set_mesh(V, F);
  }

  bool Viewer::simplify() {
	bool something_collapsed = false;
	// collapse edge

	const int max_iter = std::ceil(0.05*data().Q->size());
	for (int j = 0; j < max_iter; j++)
	{
		if (!collapse_edge(shortest_edge_and_midpoint, data().V, data().F, data().E, data().EMAP, data().EF, data().EI, *(data().Q), *(data().Qit), data().C))
		{
			break;
		}
		something_collapsed = true;
		data().num_collapsed++;
	}

	if (something_collapsed)
	{
		Eigen::MatrixXd V = data().V; 
		Eigen::MatrixXi F = data().F;
		data().clear();
		data().set_mesh(V, F);
		data().set_face_based(true);
	}
	  
	return false;
  }

  void Viewer::initEdgeErrors() {
	  Eigen::MatrixXi F = data().F = data().OF;
	  Eigen::MatrixXd V = data().V = data().OV;
	  Eigen::VectorXi EMAP;
	  Eigen::MatrixXi E, EF, EI;
	  Eigen::MatrixXd C;
	  PriorityQueue* Q = new PriorityQueue;
	  std::vector<PriorityQueue::iterator >* Qit = new std::vector<PriorityQueue::iterator >;
	  int num_collapsed = 0;
	  edge_flaps(F, E, EMAP, EF, EI);
	  Qit->resize(E.rows());

	  C.resize(E.rows(), V.cols());
	  Q->clear();
	  face_normals_dec = data().F_normals;
	  for (int e = 0; e < E.rows(); e++)
	  {
		  double cost = e;
		  Eigen::RowVectorXd p(1, 3);
		  edgeErrorAndOptimalPlacement(e, V, F, E, EMAP, EF, EI, cost, p);
		  C.row(e) = p;
		  //std::cout << "Edge error (" << e << "): " << cost << endl;
		  (*Qit)[e] = Q->insert(std::pair<double, int>(cost, e)).first;
	  }
	  data().num_collapsed = num_collapsed;
	  data().E = E;
	  data().EF = EF;
	  data().EI = EI;
	  data().EMAP = EMAP;
	  data().Q = Q;
	  data().Qit = Qit;
	  data().C = C;
	  data().set_mesh(V, F);
  }

  bool Viewer::optimalSimplify() {
	  bool something_collapsed = false;
	  // collapse edge

	  const int max_iter = std::ceil(0.05*data().Q->size());

	  face_normals_dec = data().F_normals;
	  for (int j = 0; j < max_iter; j++)
	  {
		  if (!collapse_edge(edgeErrorAndOptimalPlacement, data().V, data().F, data().E, data().EMAP, data().EF, data().EI, *(data().Q), *(data().Qit), data().C))
		  {
			  break;
		  }
		  something_collapsed = true;
		  data().num_collapsed++;
	  }

	  if (something_collapsed)
	  {
		  Eigen::MatrixXd V = data().V;
		  Eigen::MatrixXi F = data().F;
		  data().clear();
		  data().set_mesh(V, F);
		  data().set_face_based(true);
	  }

	  return false;
  }

  bool  Viewer::load_configuration()
  {
	  ifstream myReadFile;
	  myReadFile.open("configuration.txt");
	  string path;
	  if (myReadFile.is_open()) {
		  while (!myReadFile.eof()) {
			  getline(myReadFile, path);
			  cout << "Loading: " << path;
			  cout << "\n";
			  load_mesh_from_file(path);
		  }
	  }
	  else {
		  cout << "Error loading configuartion file\n";
		return false;
	  }
	  cout << "Done loading\n";
	  myReadFile.close();
	  return true;
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
      data().set_uv(UV_V,UV_F);

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
	data().OV = data().V;
	data().OF = data().F;
	//reset();
	initEdgeErrors();
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

 

} // end namespace
} // end namespace
}
