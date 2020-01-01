#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
using namespace std;
static void drawDotsAndLines(igl::opengl::glfw::Viewer &viewer) {
	Eigen::Matrix4f parents = Eigen::Matrix4f().Identity();
	for (int i = 1; i <= 4; i++) {
		viewer.data_list[i].MyTranslate(Eigen::Vector3f(0, 1.6, 0));
		viewer.data_list[i].SetCenterOfRotation(Eigen::Vector3f(0, -0.8, 0));

		viewer.data_list[i].show_overlay_depth = false;
		viewer.data_list[i].show_lines = false;
		if (i != 4){
			viewer.data_list[i].add_points(Eigen::RowVector3d(0, 0.8, 0), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 2.4, 0), Eigen::RowVector3d(0, 255, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0.8, 0), Eigen::RowVector3d(1.6, 0.8, 0), Eigen::RowVector3d(255, 0, 0));
			viewer.data_list[i].add_edges(Eigen::RowVector3d(0, 0.8, -1.6), Eigen::RowVector3d(0, 0.8, 1.6), Eigen::RowVector3d(0, 0, 255));
			viewer.data_list[i].point_size = 10;
			viewer.data_list[i].line_width = 3;
		}
	}
}

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Welcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  viewer.load_configuration();

  viewer.MyTranslate(Eigen::Vector3f(0, -3, -8));
  viewer.data_list[0].MyTranslate(Eigen::Vector3f(5, 0, 0));
  viewer.data_list[0].show_lines = false;

  drawDotsAndLines(viewer);

  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);

  delete disp;
}

